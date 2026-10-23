# Governance

This document says who decides what in the Bounded Embodiment project, and how. It is written to
work today, with one maintainer and two of the steering group's three seats vacant, and it tightens
as seats are filled. Where it and [CONTRIBUTING.md](CONTRIBUTING.md) differ, this document applies.
"Method §N" is a section of the method text, version 0.1.

## 1 Scope

This document covers:

- the method text in [`method/`](method/), and the named generators, scorers, profiles and formats
  it relies on (method §1.5, §11);
- core (`packages/core`) and the kit (`packages/kit`);
- the reference gates (`examples/reference-gates`), which are examples, not safety software;
- the rest of this repository; and
- the policy of [bounded-embodiment-results](https://github.com/RobotRegistryFoundation/bounded-embodiment-results):
  its [`POLICY.md`](https://github.com/RobotRegistryFoundation/bounded-embodiment-results/blob/main/POLICY.md)
  and the CI checks that enforce it.

It does not cover RCAN or the Robot Registry Foundation's registry, which have their own processes.
The method does not depend on either (method §1.6).

## 2 Roles

### 2.1 Maintainer

The maintainer, Craig Merry, runs the project day to day: triage, review, merges, releases and the
record of decisions. He administers this repository, bounded-embodiment-results and the two
packages' PyPI projects, and uses that access only to carry out decisions made under this document.
The first member other than the maintainer to be seated is offered the same access, so that it does
not rest with one person.

If the maintainer steps down, or does not respond on the repository for 90 days, the other members
appoint a new maintainer by group approval (section 3.2).

### 2.2 Steering group

The steering group decides method changes, results-policy changes, changes to this document and
appointments (section 3). It has three seats.

| Seat | Member | Since | Term ends | Declaration of interests |
|---|---|---|---|---|
| 1 | Craig Merry, as maintainer | adoption | held while maintainer | [method §1.6](method/bounded-embodiment-method-v0.1.md#16-disclosure) |
| 2 | vacant | | | |
| 3 | vacant | | | |

**Affiliation.** Two people are affiliated if either employs, pays or directs the other, directly
or through a project they lead; they work for the same organization; they are family; or either has
a financial stake in a project the other leads. The maintainer's projects include RCAN and the Robot
Registry Foundation registry. Unpaid contributions to a project make no one affiliated. An
**outsider** is someone not affiliated with the maintainer.

**Composition.** No two members may be affiliated with each other. While the maintainer holds seat
1, that makes the members in seats 2 and 3 outsiders, so the group has at least one outsider as soon
as a second seat is filled.

**Terms.** Seats 2 and 3 have two-year terms, and a member may serve at most three terms in a row.
The first member appointed to seat 3 serves one year, so that the terms are staggered. Seat 1 is
held by whoever is maintainer.

**Filling a seat.** Anyone may nominate someone, themselves included, by opening an issue that names
the seat. The nominee accepts on the issue and posts their declaration of interests there. After a
14-day comment period the other members decide by group approval, or, while the maintainer is the
only member, the chair decides. The appointment takes effect when the table above is updated.

**Leaving a seat.** A member leaves their seat by resigning on the repository, at the end of their
term unless reappointed, on ceasing to be eligible, or after being absent from three decisions in a
row (section 3.2). A member may be removed for a breach of the Code of Conduct or an undisclosed
conflict of interest, with the support of two members other than the one concerned and the reason
recorded. No member, the maintainer included, can remove another alone.

**Conflicts of interest.** Every member, the maintainer included, posts a declaration of interests
and updates it within 30 days of a change: employer, who funds their work on robots or AI, projects
they lead, and any machine, gate, lab or product the method could be used to test. A member recuses
from a decision in which they, or an organization they are affiliated with, have a particular
interest, such as their own appointment or removal, or a question about their own lab's results. A
recused member may comment but is not counted. The maintainer's standing interests (method §1.6) do
not by themselves require him to recuse.

### 2.3 Chair

The chair opens comment periods, asks members for decisions, records outcomes (section 3.3) and
keeps the table in section 2.2 current. The members choose the chair from among themselves each
year, by group approval; while the maintainer is the only member, he is the chair. The chair has no
casting vote, and no power beyond any other member's except under section 4.

### 2.4 Committers

Committers merge pull requests in this repository and in bounded-embodiment-results under section
3, and their reviews count toward its review requirements. Today the only committer is the
maintainer.

A contributor can be appointed committer once, over at least 60 days, they have:

1. had at least three pull requests merged in either repository, or posted at least three reviews
   that found a problem the author then fixed;
2. signed off every commit (section 6); and
3. posted a declaration of interests (section 2.2).

Anyone may nominate a contributor, themselves included, in an issue. After a 7-day comment period
the steering group decides by group approval, or the chair decides while the maintainer is the only
member. A committer inactive for 12 months becomes emeritus and loses merge access until they ask
to return. A committer may be removed for a breach of the Code of Conduct by group approval, with
the reason recorded.

### 2.5 Contributors and labs

Anyone may open issues and pull requests, and comment in any comment period. Comments are weighed on
their reasons, not on who makes them.

Labs take part by submitting results to bounded-embodiment-results under its `POLICY.md`.
Registering a lab, submitting results or offering a testbed does not make a lab or its people
members or committers, and gives no vote. No seat is held by, or reserved for, an organization. A
person who works for a lab may hold a seat as an individual, and recuses from decisions about that
lab's results.

### 2.6 AI agents

AI agents write some changes and reviews here for the maintainer. Under this document, a change an
agent writes is the maintainer's change: he is its author and signs it off (section 6). An agent is
never a member or committer, and never counts toward group approval. Its review can serve only as
the recorded second review of section 3.4, and says what it is.

## 3 Decisions

### 3.1 Kinds of change

| Kind | What it covers | Comment period | Decided by |
|---|---|---|---|
| Method change | Freezing a method version (method §1.1). Any change to the normative text of a draft version. A new version of a named generator, scorer, profile or format (method §11). Any change to the output of a released rule set or to its vectors | 14 days | group approval |
| Erratum | An error found in a frozen method version | none | recorded as an issue and corrected in the next version; the frozen text never changes (method §1.1) |
| Editorial and documentation | Wording that changes no requirement, in draft method text or in documentation | none | any committer merges |
| Code | Core, the kit apart from the safety-critical code, tools, tests, examples and the reference gates | none | any committer merges once CI passes; once there is a second committer, after a review by someone other than the author |
| Safety-critical code | The kit's runner, including its watchdog and interlocks, and its adapters (`runner/` and `adapters/` in `packages/kit/src/bounded_embodiment_kit/`), and their tests (`packages/kit/tests/test_runner_*`, `test_adapter_*`) | none | section 3.4 |
| Results policy | `POLICY.md` in bounded-embodiment-results, and any change to that repository's CI that changes which submissions it accepts | 14 days (none for editorial changes) | group approval |
| Result submission | A pull request to bounded-embodiment-results that adds runs, a lab registration or a withdrawal | none | any committer merges once its CI passes and it follows `POLICY.md`; see section 5, item 3 |
| This document | Any change | 14 days | group approval, within the limits of section 7 |
| Appointments | Seats and committers | 14 days for a seat, 7 for a committer | sections 2.2 and 2.4 |

A results-policy change applies only to submissions made after it is merged. If anyone says on a
pull request, with a reason, that it belongs to a kind with a longer comment period or a stricter
rule, it is handled as that kind.

A fix for a security or run-safety problem ([SECURITY.md](SECURITY.md)) in code, safety-critical
code or the results repository's CI may be merged before the reviews or comment period it needs, if
waiting would leave labs exposed. The pull request says why, the reviews are recorded within 14
days, and any comment period runs afterwards; if the decision then goes against the fix, it is
reverted.

### 3.2 Group approval

A decision by group approval is made when more than half of the eligible members support it: both,
if two are eligible; two, if three are.

- **Eligible** members are the seated members, less any who recuse or are absent.
- **Support** is an approving review, or a comment saying so, on the pull request or issue. A member
  who opened the pull request supports it by opening it. No rule here needs anyone to approve their
  own pull request.
- A member is **absent** from a decision if they have neither supported it nor objected 14 days
  after the chair asked them on the pull request or issue. The chair asks when the comment period
  opens.
- If only one member is eligible, that member decides under section 4.

A decision without the support it needs fails, and the change is not made. Repository settings, such
as branch protection, never require more approvals than this section does.

### 3.3 Comment periods and records

A comment period opens when the chair says so on the pull request or issue, with a closing date at
least the required number of days later, and links it from this repository's Discussions. A
revision during the period that adds, removes or changes a requirement or rule, or changes how a
verdict is reached, restarts it.

Every decision other than a merge of editorial, documentation, code or result-submission changes is
recorded in a comment on the pull request or issue. The record gives the kind of change and the rule
applied; the dates of the comment period; who supported it, who recused and who was absent; and each
objection, with how it was answered. A decision made under section 4 says "interim rule".

### 3.4 Safety-critical code

A change to safety-critical code (section 3.1) needs two reviews, recorded on the pull request, by
people other than its author, at least one of whom is not affiliated with the author. Reviewers need
not be members or committers.

While there are fewer than two members and committers other than the author, or none of them is
unaffiliated with the author, the change needs instead:

1. the maintainer's review, recorded on the pull request, including when the change is his own; and
2. a recorded second review: one that says who or what made it, and that it is not a maintainer
   approval.

Today the second review is made by an AI agent and posted from the maintainer's account. It is a
second reading, not an independent one: it does not stand in for the unaffiliated reviewer above,
and it does not lift the kit's draft notice (README, "Status"; the method's known gaps). That notice
stays until a line-by-line review of the runner, watchdog and adapters by a person other than the
maintainer is recorded and linked from the change that removes it.

## 4 Interim rule

This section applies while the maintainer is the only member, and whenever recusals or absences
leave one member eligible for a decision. A decision that needs group approval is then made by that
member when:

1. its comment period (section 3.1) has closed;
2. no sustained objection is open; and
3. the decision is recorded under section 3.3, marked "interim rule".

An objection is **sustained** if it is posted on the pull request or issue before the comment period
closes, says what is wrong with the change and why, and has not been withdrawn. The author answers
each objection there. An objection lapses if the objector does not reply within 14 days of being
asked whether the answer meets it. While a sustained objection is open, the change waits until the
objection is withdrawn or lapses, the change is revised to meet it (which may restart the comment
period, section 3.3), or a second member is eligible and the group decides. An objection that
breaches the Code of Conduct is not counted.

Appointments of members and committers do not wait for objections, but an objection that the
nominee is not eligible is answered on the issue, with facts, before the appointment is made.

**Sunset.** While the other seats are vacant, the maintainer can decide alone under this section
only until 30 April 2027. If no other member has been seated by then, from that date no method
version is frozen, and no method change, results-policy change or change to this document is
decided, until one is. Errata, editorial and documentation changes, code, safety-critical code
under section 3.4, result submissions and appointments continue.

## 5 Independence and honesty

1. The project does not certify, approve, accredit, rate or endorse any machine, gate, lab, product
   or person, and no role in this document can do so on its behalf. Results are evidence, not
   certification: a verdict is a finding about one run (method §10 item 1), and results are not to
   be cited as certification evidence (POLICY.md, "Not certification evidence").
2. Anything the project, a member or a committer writes about results follows method §10 and
   POLICY.md, "Claim language": self-run; illustrative thresholds; "not reproduced" until a
   reproduction exists; and none of the words those rules exclude.
3. Results from the maintainer's labs are labelled "maintainer's own" (method §10 item 4; POLICY.md,
   "The maintainer's own results"). All his labs count as one lab, so a re-run by one of them is
   never a reproduction (method §9.7 item 2). Once there is a committer other than the maintainer,
   that committer, not the maintainer, merges results labelled "maintainer's own".
4. Nobody says or implies, in the project's name, that a person, organization or standards body
   endorses, supports, uses or has reviewed this work, unless that party has said so publicly and
   the statement links to it. A presentation, or the receipt of a contribution, is not an
   endorsement. A member's or committer's role here is not an endorsement by their employer, and
   employers are not listed as supporters.
5. No method change may make running a test need a particular protocol, vendor or model, RCAN
   included.

## 6 Conduct, sign-off and licences

Everyone follows the [Code of Conduct](CODE_OF_CONDUCT.md). The community leaders it names are the
steering group's members, and a report about one member goes to the others. While the maintainer is
the only member, there is no one else to receive a report about him; this gap closes when a second
member is seated.

Every commit is signed off under the Developer Certificate of Origin, as
[CONTRIBUTING.md](CONTRIBUTING.md#sign-off-dco) describes. An AI agent cannot make that
certification, so the maintainer signs off an agent's commits before they are merged.

Contributions are licensed like the files they change, CC BY 4.0 or Apache-2.0, as
[REUSE.toml](REUSE.toml) records path by path; the texts are in [`LICENSES/`](LICENSES/).
CONTRIBUTING.md's terms on standards bodies and patents apply to every contribution. A change to the
licences of new contributions is decided like a change to this document.

## 7 Changing this document

A change to this document is decided as section 3.1 says. A change that does any of the following
needs group approval with at least two eligible members, and cannot be made under section 4:

- moves the sunset date in section 4 later, or removes it;
- weakens the composition rule in section 2.2;
- lowers a support count, shortens a comment period, or adds to what one member may decide alone; or
- narrows section 5.

This document is adopted under its own section 4: the pull request that adds it has a 14-day comment
period, and is also the nomination of any member it names. Pull requests open when it is adopted
follow it; their comment periods open when the chair opens them.

If the work moves to a neutral steward, or a standards body takes on the method, that body's process
replaces this one for what it takes over, and this document will say so.
