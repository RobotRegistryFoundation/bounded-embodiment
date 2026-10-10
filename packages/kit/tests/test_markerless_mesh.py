"""STL reading and surface sampling for the markerless instrument (no robot meshes needed)."""

from __future__ import annotations

import math
import struct

import numpy as np
import pytest
from bounded_embodiment_kit.instruments.markerless import mesh
from bounded_embodiment_kit.kinematics import URDFModel, Visual


def cube(lo=(0.0, 0.0, 0.0), hi=(1.0, 1.0, 1.0)) -> np.ndarray:
    """12 outward-wound (counter-clockwise from outside) triangles of an axis-aligned box."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def binary_stl(tris, header=b"binary stl") -> bytes:
    out = bytearray(header.ljust(80, b" ")[:80] + struct.pack("<I", len(tris)))
    for t in tris:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        out += struct.pack("<12fH", *n, *np.asarray(t).ravel(), 0)
    return bytes(out)


def ascii_stl(tris) -> str:
    lines = ["solid box"]
    for t in tris:
        lines += ["  facet normal 0 0 0", "    outer loop"]
        lines += [f"      vertex {x:.9g} {y:.9g} {z:.9g}" for x, y, z in t]
        lines += ["    endloop", "  endfacet"]
    return "\n".join([*lines, "endsolid box"]) + "\n"


def test_binary_and_ascii_stl_read_the_same(tmp_path):
    tris = cube(hi=(0.5, 0.25, 0.125))
    (tmp_path / "b.stl").write_bytes(binary_stl(tris))
    (tmp_path / "a.stl").write_text(ascii_stl(tris), encoding="ascii")
    b, a = mesh.read_stl(tmp_path / "b.stl"), mesh.read_stl(tmp_path / "a.stl")
    assert b.shape == a.shape == (12, 3, 3)
    assert np.allclose(a, tris) and np.allclose(b, tris, atol=1e-7)


def test_binary_stl_whose_header_says_solid_is_still_binary(tmp_path):
    tris = cube()
    (tmp_path / "x.stl").write_bytes(binary_stl(tris, header=b"solid exported by a CAD tool"))
    assert np.allclose(mesh.read_stl(tmp_path / "x.stl"), tris)


def test_not_an_stl_is_refused(tmp_path):
    (tmp_path / "bad.stl").write_bytes(b"\x00\x01 this is not a mesh" * 10)
    with pytest.raises(mesh.MeshError):
        mesh.read_stl(tmp_path / "bad.stl")
    (tmp_path / "short.stl").write_text("solid x\n facet normal 0 0 1\n outer loop\n vertex 0 0 0\n vertex 1 0 0\n")
    with pytest.raises(mesh.MeshError):
        mesh.read_stl(tmp_path / "short.stl")


def test_read_stl_rereads_a_changed_file(tmp_path):
    p = tmp_path / "m.stl"
    p.write_bytes(binary_stl(cube()))
    assert mesh.read_stl(p).shape[0] == 12
    p.write_bytes(binary_stl(cube()[:6]))
    assert mesh.read_stl(p).shape[0] == 6


def test_volume_area_and_normals():
    tris = cube(hi=(2.0, 3.0, 4.0))
    areas, normals = mesh.triangle_areas_normals(tris)
    assert math.isclose(mesh.signed_volume(tris), 24.0)
    assert math.isclose(areas.sum(), 2 * (6 + 8 + 12))
    centre = np.array([1.0, 1.5, 2.0])
    assert (np.einsum("ij,ij->i", tris.mean(axis=1) - centre, normals) > 0).all()     # outward
    assert math.isclose(mesh.signed_volume(tris[:, ::-1]), -24.0)                    # inside out


def test_sample_surface_is_deterministic_on_the_surface_and_outward():
    tris = cube()
    p1, n1 = mesh.sample_surface(tris, 0.05, seed=3)
    p2, n2 = mesh.sample_surface(tris, 0.05, seed=3)
    p3, _ = mesh.sample_surface(tris, 0.05, seed=4)
    assert np.array_equal(p1, p2) and np.array_equal(n1, n2)
    assert not np.array_equal(p1, p3)
    on_face = (np.abs(p1) < 1e-12) | (np.abs(p1 - 1.0) < 1e-12)
    assert on_face.any(axis=1).all()                                                   # on the surface
    assert (np.einsum("ij,ij->i", p1 - 0.5, n1) > 0).all()                             # outward normals
    assert np.allclose(np.linalg.norm(n1, axis=1), 1.0)
    n_expected = 6 / 0.05 ** 2                                                         # one per surface cell
    assert 0.7 * n_expected < len(p1) < 1.05 * n_expected


def test_thin_keeps_one_point_per_voxel_in_order():
    P = np.array([[0.01, 0.01, 0.0], [0.02, 0.02, 0.0], [0.11, 0.0, 0.0], [0.015, 0.0, 0.0]])
    kept, extra = mesh.thin(P, 0.1, np.arange(4))
    assert extra.tolist() == [0, 2]
    assert np.array_equal(kept, P[[0, 2]])


def _visual(path, origin=None, scale=(1.0, 1.0, 1.0)):
    return Visual("l", path, np.eye(4) if origin is None else origin, scale)


def test_visual_triangles_apply_scale_then_origin(tmp_path):
    p = tmp_path / "box.stl"
    p.write_bytes(binary_stl(cube(hi=(10.0, 20.0, 30.0))))                            # millimetres
    T = np.eye(4)
    T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]                                    # 90 deg about z
    T[:3, 3] = [1.0, 2.0, 3.0]
    v = mesh.visual_triangles(_visual(p, T, (0.001, 0.001, 0.001)))
    lo, hi = v.reshape(-1, 3).min(axis=0), v.reshape(-1, 3).max(axis=0)
    assert np.allclose(lo, [1.0 - 0.020, 2.0, 3.0]) and np.allclose(hi, [1.0, 2.010, 3.030])
    assert mesh.signed_volume(v) > 0
    assert not v.flags.writeable


def test_visual_triangles_fix_inside_out_and_mirrored_meshes(tmp_path):
    inside_out = tmp_path / "inv.stl"
    inside_out.write_bytes(binary_stl(cube()[:, ::-1]))
    assert mesh.signed_volume(mesh.visual_triangles(_visual(inside_out))) > 0
    plain = tmp_path / "plain.stl"
    plain.write_bytes(binary_stl(cube()))
    mirrored = mesh.visual_triangles(_visual(plain, scale=(-1.0, 1.0, 1.0)))
    assert mesh.signed_volume(mirrored) > 0
    assert mirrored.reshape(-1, 3)[:, 0].min() == pytest.approx(-1.0)


URDF_TWO_VISUALS = """<robot name="r">
  <link name="base"><visual><origin xyz="0 0 0"/><geometry><mesh filename="a.stl"/></geometry></visual></link>
  <link name="arm">
    <visual><origin xyz="0 0 0"/><geometry><mesh filename="a.stl"/></geometry></visual>
    <visual><origin xyz="0 0 2"/><geometry><mesh filename="a.stl" scale="0.5 0.5 0.5"/></geometry></visual>
  </link>
  <joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>"""


def test_link_clouds_and_mesh_bookkeeping(tmp_path):
    (tmp_path / "a.stl").write_bytes(binary_stl(cube()))
    u = URDFModel.from_string(URDF_TWO_VISUALS, base_dir=tmp_path)
    clouds = mesh.link_clouds(u, 0.1, seed=0)
    assert set(clouds) == {"base", "arm"}
    arm = clouds["arm"]
    assert arm.area == pytest.approx(6.0 + 6.0 * 0.25)
    assert arm.points[:, 2].max() == pytest.approx(2.5) and arm.points[:, 2].min() == pytest.approx(0.0)
    again = mesh.link_clouds(u, 0.1, seed=0)
    assert np.array_equal(again["arm"].points, arm.points)
    assert mesh.mesh_files(u) == [tmp_path / "a.stl"]
    assert mesh.missing_meshes(u) == []
    assert set(mesh.describe(u)) == {"a.stl"}


def test_missing_meshes_say_how_to_fetch_them(tmp_path):
    u = URDFModel.from_string(URDF_TWO_VISUALS, base_dir=tmp_path)
    assert mesh.missing_meshes(u) == [tmp_path / "a.stl"]
    with pytest.raises(mesh.MeshError, match="fetch_robot_assets"):
        mesh.check_meshes(u)
