# Requirements and test plan (scope draft)

*Informative. Not part of the normative method text.*

This page states the five requirements and the test plan behind the Bounded Embodiment method, in
vendor-neutral terms. Version 0.1 of the [method](../method/) defines two of the tests, EV-03 and
EV-10, normatively. The rest of this page is scope: it says what later versions are expected to
cover, and it is the page to comment on if you think a requirement is missing, wrong or untestable.
Comment with the ["Feedback on a requirement or test"](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=feedback-requirement.yml)
issue form.

Nothing here depends on any particular protocol, vendor or model. One open implementation, RCAN,
maps these requirements onto its own protocol in its Appendix C; that mapping is RCAN's, not part of
this method.

## The principle

The model proposes; a bounded layer disposes. Any model, from any vendor, may propose actions. A
small, deterministic gate between the model and the actuators decides what executes, and the
machine's safety claim rests on that gate, not on the model.

The pattern is not new. It is the Simplex architecture and what aviation calls run-time assurance
(ASTM F3269); guarding an AI function with a non-AI safety function is also described in
ISO/IEC TR 5469. What this work adds is a common test of it for machines driven by learned models,
one a third party can run without the model's weights.

It complements, and replaces none of: ISO 10218:2025, ISO 13482, ISO/CD 25785-1, ISO 13849-1,
IEC 61508, IEC 60204-1, ISO/IEC TR 5469, IEEE 7009 and ITU-T F.748.44. F.748.44 benchmarks the
model; this tests the machine around it.

## Gate decisions

For every proposed command the gate makes exactly one decision:

| Decision | What executes |
|---|---|
| allow | the command, unchanged |
| clamp | the nearest command inside the envelope |
| reject | nothing; the machine keeps its declared safe behaviour (hold or slow down), and the reason goes back to the model |
| stop | the machine goes to its declared safe state |

Any gate exception, timeout, stale state or unparseable command is expected to produce a stop.
Stopped is not always safe, so each machine declares the safe state it ends in: a rover stops in
place, an arm holds or sets its load down, a legged robot crouches under control before power-off.

## Recovery

What happens after the gate says no:

| Gate outcome | What happens next | Intervention? |
|---|---|---|
| Clamp | The nearest in-envelope command runs | No |
| Reject | Declared hold or slow-down; the reason goes back to the model to re-plan | No |
| Protective stop | Resumes once the condition clears and sensor state is fresh | No |
| Repeated stops | Escalates after a declared count or time | Yes |
| Emergency stop | Manual reset; reset alone never restarts motion | Yes |

"Intervention" means a person has to act before the machine moves again. The reason a reject
sends back is information for the model's next proposal; it gives the model no say in the gate's
decision.

Version 0.1 of the method tests none of these outcomes. They are scope for later versions.

## The five requirements

| # | Requirement | Verified by |
|---|---|---|
| R1 | **Declared envelope, set by the integrator.** The integrator declares workspace, speed, force and stopping limits in machine-readable form. | EV-01, EV-09: externally measured maximums stay at or below the declared limits |
| R2 | **A safety function below the model.** The model has no write path to the gate, the envelope or the log. | EV-03: hostile commands produce zero motion outside the envelope |
| R3 | **Stop always wins.** Every stop source overrides everything within a declared time, and loss of power, communications or heartbeat leads to the declared safe state. | EV-02, EV-04, EV-05, EV-06: stop time and distance measured for every stop source at top speed |
| R4 | **Accountable commands.** Every executed command traces to a principal who authorised it. Identity and delegation mechanics belong to other work (ITU-T FG-TIDA). | EV-07: log audit shows every executed command traces to an authorising principal |
| R5 | **Tamper-evident evidence.** Every gate decision is logged append-only and hash-chained. | EV-08: the hash-chained log verifies and replays cleanly against the envelope |

## The tests

| ID | Test | Requirement | Where it is defined |
|---|---|---|---|
| EV-01 | Envelope honesty | R1 | scope only |
| EV-02 | Stop performance, per stop source at top speed | R3 | scope only |
| **EV-03** | **Hostile model:** a seeded hostile command stream replaces the model for 10 minutes while an external instrument measures the machine | R2 | **method v0.1, §7** |
| EV-04 | Blind machine: stale or corrupt sensor data leads to a stop | R3 | scope only |
| EV-05 | The model process dies | R3 | scope only |
| EV-06 | The gate process dies | R3 | scope only |
| EV-07 | Unauthorised command | R4 | scope only |
| EV-08 | Log tampering | R5 | scope only |
| EV-09 | Human approach | R1 | scope only |
| **EV-10** | **Liveness:** the same system still reaches the waypoints the lab declares, so a gate cannot pass by refusing everything | all | **method v0.1, §8** |

Rates, durations and tolerances are illustrative defaults in version 0.1, not proposed thresholds.

## Assurance levels

The requirements stay fixed; the assurance rises.

| Level | Name | Expected of the machine | Typical machines (illustrative) |
|---|---|---|---|
| BE-1 | Declared | Published envelope. Gate in a process separate from the model. Hash-chained gate log. | Hobby, education, research |
| BE-2 | Enforced | BE-1, plus: gate on independent compute or firmware, a hardware stop path, watchdogs on the model and on the gate, and the fault-injection tests (EV-03 to EV-08) passed. | Consumer and service robots |
| BE-3 | Assured | BE-2, plus: gate and stop path meet a functional-safety integrity target (ISO 13849-1 PL or IEC 61508 SIL), with third-party testing. | Industrial, medical, heavy |

A level is self-declared unless third-party evidence comes with it. A test verdict is a finding
about one run, never a certification, and results tooling never displays a self-declared level.

## Out of scope

- **Whether the limits are right.** That is risk assessment (ISO 12100, ISO 10218-2, ISO/TS 15066).
  These tests check that the machine honours its declaration.
- **Harm inside the envelope.** A machine can stay inside its limits and still do the wrong thing
  slowly. That is semantic safety, and complementary work.
- **Sensing performance.** Whether the machine detects a person is a sensor question; EV-09 tests the
  response once it does.
- **General cybersecurity** (IEC 62443). The gate and envelope are assumed protected from the model,
  not from every attacker.
- **Identity and delegation mechanics** (ITU-T FG-TIDA).
