# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The scrubber: what it replaces, what it leaves alone, and that it is deterministic."""
from __future__ import annotations

import json
import socket

import pytest
from bounded_embodiment_kit.submission.scrub import Identity, Scrubber

ID = Identity(hostnames=("bob-lab.example.org", "robot"), usernames=("ana", "pi"),
              homes=("/srv/home/ana", "/"), redact=("Building 4",))


@pytest.fixture
def s():
    return Scrubber(ID)


@pytest.mark.parametrize(("text", "expected"), [
    ("urdf /home/ana/be/examples/so101.urdf.", "urdf ~/.../so101.urdf."),
    ("in /home/ana and /home/ana/x.urdf", "in ~ and ~/x.urdf"),
    ("(see /home/ana/x.urdf)", "(see ~/x.urdf)"),
    ("file:///home/ana/notes.md", "file://~/notes.md"),
    ("/Users/Ana/Desktop/run/notes.md", "~/.../notes.md"),
    ("/var/home/ana/a and /export/home/ana/b and /usr/home/ana/c", "~/a and ~/b and ~/c"),
    (r"C:\Users\Ana\Documents\x.urdf", "~/.../x.urdf"),
    (r"C:\\Users\\Ana\\Documents\\x.urdf", "~/.../x.urdf"),
    ("/mnt/c/Users/ana/x", "~/x"),
    ("/root/.ssh/id_ed25519 but /rooted/x", "~/.../id_ed25519 but /rooted/x"),
    ("~ana/x/y and ~/x/y", "~/.../y and ~/x/y"),
    ("/srv/home/ana/a/b/c.json and /srv/home/anabel/x", "~/.../c.json and /srv/home/anabel/x"),
])
def test_home_directory_paths_keep_only_their_last_component(s, text, expected):
    assert s.text(text) == expected


@pytest.mark.parametrize(("text", "expected"), [
    ("udp 192.168.1.20:5005 bound to 0.0.0.0", "udp [ip]:5005 bound to [ip]"),
    ("10.0.0.7.", "[ip]."),
    ("300.1.1.1 and 1.2.3.4.5 and 0.25,0.5,1.0", "300.1.1.1 and 1.2.3.4.5 and 0.25,0.5,1.0"),
    ("fe80::1%eth0, 2001:db8::8a2e:370:7334 and ::ffff:10.0.0.1", "[ip], [ip] and [ip]"),
    ("::1 at 18:00:00 or 12:34:56.789, mac aa:bb:cc:dd:ee:ff", "::1 at 18:00:00 or 12:34:56.789, mac aa:bb:cc:dd:ee:ff"),
    ("2026-10-10T18:00:00Z sha256:5e3ab5d7ec59cb53004dcbd6f6ae2f5852abc8af253a346c1bc179868dc295ec",
     "2026-10-10T18:00:00Z sha256:5e3ab5d7ec59cb53004dcbd6f6ae2f5852abc8af253a346c1bc179868dc295ec"),
])
def test_ip_addresses(s, text, expected):
    assert s.text(text) == expected


@pytest.mark.parametrize(("text", "expected"), [
    ("rcan://robot.local:8081/bob and printer.LAN, nas.home.arpa", "rcan://[host]:8081/bob and [host], [host]"),
    ("ssh pi@bob-lab.example.org, then bob-lab and BOB-LAB", "ssh pi@[host], then [host] and [host]"),
    ("the robot arm on a Raspberry Pi 5", "the robot arm on a Raspberry Pi 5"),
    ("Ana Smith signed; ana ran it as ana_2 and x-ana; anabel and ana5 did not",
     "Ana Smith signed; [user] ran it as [user]_2 and x-[user]; anabel and ana5 did not"),
    ("in Building 4, room 2; building 4 again", "in [redacted], room 2; [redacted] again"),
])
def test_hostnames_usernames_and_redactions(s, text, expected):
    assert s.text(text) == expected


def test_generic_and_short_names_are_not_looked_for():
    s = Scrubber(Identity(hostnames=("localhost", "raspberrypi", "pi", "ab"),
                          usernames=("root", "ubuntu", "runner", "jo")))
    text = "root on localhost (raspberrypi), user ubuntu, runner jo; ab"
    assert s.text(text) == text


def test_deterministic_idempotent_and_independent_of_order():
    text = "ana@bob-lab: /home/ana/a/b.txt from 10.1.2.3 via robot.local in Building 4"
    a, b = Scrubber(ID), Scrubber(Identity(*(tuple(reversed(f)) for f in
                                             (ID.hostnames, ID.usernames, ID.homes, ID.redact))))
    once = a.text(text)
    assert once == b.text(text) == Scrubber(ID).text(text)
    assert a.text(once) == once
    assert once == "[user]@[host]: ~/.../b.txt from [ip] via [host] in [redacted]"


def test_counts_say_what_was_replaced_where_never_the_values(s):
    s.text("ana on 10.0.0.1 and 10.0.0.2", "notes/NOTES.md")
    s.text("/home/ana/x", "raw/instrument/a.json")
    assert s.summary() == ["notes/NOTES.md: 2 ip, 1 username", "raw/instrument/a.json: 1 home path"]
    assert s.finds("ana at robot.local") == {"username": 1, "hostname": 1}
    assert s.summary() == ["notes/NOTES.md: 2 ip, 1 username", "raw/instrument/a.json: 1 home path"]


def test_json_strings_and_keys_are_scrubbed_and_an_untouched_file_keeps_its_bytes(s):
    raw = b'{"path": "\\/home\\/ana\\/s1\\/fit", "/home/ana/k": 1, "x": NaN, "n": [1.5, "ana"]}'
    out = json.loads(s.json_bytes(raw, "a.json").decode().replace("NaN", "null"))
    assert out == {"path": "~/.../fit", "~/k": 1, "x": None, "n": [1.5, "[user]"]}
    clean = b'{"a": 1,   "b": "nothing here"}\n'
    assert s.json_bytes(clean, "b.json") is clean
    assert s.json_bytes(b'{"a": "ana", "a": "x"}', "dup.json") == b'{"a": "[user]", "a": "x"}'


def test_csv_text_cells_only_never_numbers_headers_or_protected_columns(s):
    data = (b"seq,t_sent_s,strategy,payload_json,result,error_kind\n"
            b'0,0.02,uniform,"{""ana"":1.5}",applied/ok by ana,\n'
            b"1,192.168.1.2,boundary,{},not_applied,exception:Error on robot.local\n")
    out = s.csv_bytes(data, "raw/commands.csv", ("strategy", "payload_json")).decode()
    assert out == ('seq,t_sent_s,strategy,payload_json,result,error_kind\n'
                   '0,0.02,uniform,"{""ana"":1.5}",applied/ok by [user],\n'
                   '1,[ip],boundary,{},not_applied,exception:Error on [host]\n')
    numbers = b"t_s,point,x_m\n0.1,tool,1e-05\n"
    assert s.csv_bytes(numbers, "raw/samples.csv", ("point",)) is numbers


def test_this_machine_knows_its_own_names():
    me = Identity.this_machine(redact=["Lab 3B"])
    assert socket.gethostname() in me.hostnames and me.redact == ("Lab 3B",)
    assert Scrubber(me).text(f"on {socket.gethostname()}") in (f"on {socket.gethostname()}", "on [host]")
