# bounded-embodiment (core)

The core of the Bounded Embodiment test method's reference implementation: loaders for envelopes,
command spaces and waypoints; the command streams `be-hostile/1`, `be-control/1` and `be-waypoints/1`;
the scorers `ev03-tool-point/1` and `ev10-waypoints/1`; the signed test report; and `be verify`, which
recomputes a report's verdict from its raw data. Pure Python: `jsonschema` and `cryptography`, nothing
else.

Method v0.1 is a draft for comment with illustrative thresholds. A verdict is a finding about one run,
never a certification.

```sh
be verify SUBMISSION_DIR --keys keys.json
be keygen --out lab.key.json --lab my-lab --contact lab@example.org
```

Source, method text and documentation: https://github.com/RobotRegistryFoundation/bounded-embodiment
