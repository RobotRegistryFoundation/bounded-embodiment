# Governance

This document says who decides what in the Bounded Embodiment project, and how. It is written to
work with one maintainer and the other seats vacant, and it tightens as seats are filled. Where it
differs from CONTRIBUTING.md, or from POLICY.md, README.md, labs/README.md or CODEOWNERS in
bounded-embodiment-results, this document applies until they are brought into line. "Method §N" is
a section of the method text, version 0.1.

**Draft.** Items marked [TO DECIDE: …] or [TO COMPLETE: …] are the maintainer's to settle before
this document is adopted.

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
Nothing in the method requires RCAN (method §1.6) or a registry number (method §9.3 item 3). The
envelope 0.1 and gate-decision 0.1 schemas that core ships are copies of rcan-spec schemas; the
copies are governed here and do not change when rcan-spec does. RCAN's Appendix C relies on
`docs/requirements.md` for the requirements and the levels BE-1 to BE-3; this document, not RCAN's
process, decides changes to them.

## 2 Roles

### 2.1 Maintainer

The maintainer, Craig Merry, runs the project day to day: triage, review, merges, releases and the
record of decisions. He administers this repository, bounded-embodiment-results and the two
packages' PyPI projects, and uses that access only to carry out decisions made under this document.
The first member other than the maintainer to be seated is offered the same access, so that it does
not rest with one person.

**What rests with the maintainer today.** Until a second member is seated: every decision (section
4); every merge, and the only human review of safety-critical code; administration of both
repositories, in a GitHub organization that also holds RCAN, and of both PyPI projects; the homepage
and results table, which robotregistryfoundation.org builds from his own repository
`craigm26/RobotRegistryFoundation`, outside this document; the only contact for conduct and security
reports (CODE_OF_CONDUCT.md, SECURITY.md); POLICY.md's removal of personal or unlawful material,
which may rewrite the results repository's history; and succession, since no one can appoint a new
maintainer. The Robot Registry Foundation is a proposal in a draft charter, not a legal entity, and
it does not govern this project.

If the maintainer steps down, is removed (section 2.2), or does not respond on the repository for
90 days, the other members appoint a new maintainer by group approval (section 3.2).

### 2.2 Steering group

The steering group decides method changes, requirements and results-policy changes, changes to this
document and appointments (section 3). It has three seats.

| Seat | Member | Since | Term ends | Declaration of interests |
|---|---|---|---|---|
| 1 | Craig Merry, as maintainer | adoption | held while maintainer | [craig-merry.md](docs/declarations/craig-merry.md) [TO COMPLETE: before adoption] |
| 2 | [TO DECIDE: an outside member, needed before adoption] | [TO DECIDE] | [TO DECIDE] | [TO DECIDE] |
| 3 | vacant [TO DECIDE: a name, or leave vacant] | | | |

**Affiliation.** Two people are affiliated if either employs, pays or directs the other, directly
or through a project they lead; they work for the same organization; they are family; or either has
a financial stake in a project the other leads. The maintainer's projects include RCAN, the Robot
Registry Foundation registry and OpenCastor; his declaration lists them all. Unpaid contributions to
a project make no one affiliated. An **outsider** is someone not affiliated with the maintainer.

**Composition.** No two members may be affiliated with each other. While the maintainer holds seat
1, that makes the members in seats 2 and 3 outsiders, so the group has at least one outsider as soon
as a second seat is filled.

**Terms.** [TO DECIDE: term length, limit and stagger. Proposed: seats 2 and 3 have two-year terms;
a member may serve at most three in a row; the first member appointed to an outside seat serves one
year, so that the terms are staggered; someone appointed to a seat vacated early serves the rest of
its term.] Seat 1 is held by whoever is maintainer.

**Filling a seat.** Anyone may nominate someone, themselves included, by opening an issue that names
the seat. The nominee accepts on the issue and posts their declaration of interests there. After a
14-day comment period, a seat is filled by group approval with at least two eligible members, not
under section 4. The one exception: while the maintainer is the only member, he decides one
appointment, as chair. The appointment takes effect when the decision is recorded (section 3.3).
The chair then updates the table. An update that records a decision already made (an appointment, a
resignation, the end of a term, a removal or a new declaration) is a record, not a change to this
document.

**Leaving a seat.** A member leaves their seat by resigning on the repository, at the end of their
term unless reappointed, on ceasing to be eligible, or after being absent from every decision for
90 days, and from at least three decisions in that time. A member may be removed for a breach of
the Code of Conduct or an undisclosed conflict of interest, with the support of two members other
than the one concerned and the reason recorded. No member, the maintainer included, can remove
another alone. The other two members together may remove the maintainer for the same reasons, with
the reason recorded; he then ceases to be maintainer, and they appoint a new one (section 2.1).
While he alone holds the access in section 2.1, carrying this out depends on him.

**Conflicts of interest.** Every member, the maintainer included, posts a declaration of interests
and updates it within 30 days of a change: employer, who funds their work on robots or AI, projects
they lead, any machine, gate, lab or product the method could be used to test, and a contact
address for decisions (section 3.2). A declaration lists the person's contributions to the
maintainer's projects in the last two years. A member recuses from a decision in which they, or an
organization they are affiliated with, have a particular interest, such as their own appointment or
removal, or a question about their own lab's results. A recused member may comment but is not
counted. The maintainer's standing interests, as declared, do not by themselves require him to
recuse.

### 2.3 Chair

The maintainer is the first chair. A chair serves until the members choose another member by group
approval, which they may do at any time. If the chair leaves the group, the member who has served
longest is chair until the members choose. The chair has no casting vote. Beyond any member's
powers, the chair opens comment periods, asks members for decisions, records outcomes (section 3.3)
and keeps the table in section 2.2 current, and, while the maintainer is the only member, decides
appointments (sections 2.2 and 2.4).

### 2.4 Committers

Committers, and every member while seated, merge pull requests in this repository and in
bounded-embodiment-results under section 3, and their reviews count toward its review requirements.
A change decided under section 3 is merged by the chair or any committer within 7 days of the
record; if it is not, any member may merge it. Today the only committer is the maintainer.

A contributor can be appointed committer once, over at least 60 days, they have:

1. had at least three pull requests merged in either repository, not counting result submissions
   and lab registrations, or posted at least three reviews that found a problem the author then
   fixed;
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

AI agents write some changes and reviews here. A change an agent writes is the change of the person
who directs the agent, who is its author and signs it off (section 6). Today that is always the
maintainer. An agent is never a member or committer, and never counts toward group approval. Its
review can serve only as the recorded second review of section 3.4. An agent's review is headed as
an agent review. It is independent, at most, of the session that wrote the change, never of the
person who directs it.

## 3 Decisions

### 3.1 Kinds of change

| Kind | What it covers | Comment period | Decided by |
|---|---|---|---|
| Method change | Freezing a method version (method §1.1). Any change to the normative text of a draft version. A new named generator, scorer, profile or format, or a new version of one (method §11) | 14 days | group approval |
| Released versions | A frozen method text; a released rule set, profile or format, and its vectors | none | never changed (method §1.1, §1.5, §11). A pull request that would change one is refused, and the change is made as a new version, which is a method change |
| Erratum | An error found in a frozen method version | none | recorded as an issue and corrected in the next version; the frozen text never changes (method §1.1) |
| Requirements and scope | `docs/requirements.md`: a change to a requirement, a test in the test plan, or the assurance levels | 14 days | group approval |
| Editorial and documentation | Wording that changes no requirement or rule, in draft method text, in documentation or in `POLICY.md` | none | any committer merges |
| Code | Core, the kit apart from the safety-critical code, tools, tests, examples and the reference gates | none | any committer merges once CI passes; once there is a second committer, after a recorded review by someone other than the author, who need not be a committer |
| Safety-critical code | The kit's runner, including its watchdog and interlocks, its adapters and its kinematics (`runner/`, `adapters/` and `kinematics/` in `packages/kit/src/bounded_embodiment_kit/`), any other code the runner calls to decide whether to stop (today including core's `Envelope.clearance`), and their tests | none | section 3.4 |
| Results policy | `POLICY.md` in bounded-embodiment-results; any change to that repository's CI or tools that changes which submissions it accepts or how its table labels or counts a result, such as removing a handle or lab from `tools/maintainer.json` or changing `tools/claim-words.txt`. Adding a released core version to `kits.lock` is a code change | 14 days | group approval |
| Result submission | A pull request to bounded-embodiment-results that adds runs, a lab registration or a withdrawal | none | any committer merges once its CI passes and it follows `POLICY.md`; see section 5, item 3 |
| This document | Any change, other than updating the table in section 2.2 to record a decision | 14 days | group approval, within the limits of section 7 |
| Appointments | Seats and committers | 14 days for a seat, 7 for a committer | sections 2.2 and 2.4 |

A results-policy change applies only to submissions made after it is merged. A change that fits no
row is handled as the kind with the longest comment period that could apply. If anyone says, with a
reason, on a pull request or within 14 days after it was merged, that it belongs to a stricter kind,
it is handled as that kind; for a merged change the comment period runs afterwards, and if the
decision goes against it, it is reverted.

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
  after they were asked (section 3.3), on the pull request or issue and by email at the contact in
  their declaration.
- If only one member is eligible, that member decides under section 4, except where a rule needs at
  least two eligible members.
- If no member is eligible, nothing is decided and nothing changes until one is. A lab may always
  withdraw its own results (POLICY.md).

A decision without the support it needs fails, and the change is not made. Repository settings, such
as branch protection, never require more approvals than this section does.

### 3.3 Comment periods and records

A comment period opens when the chair opens it, or 14 days after the author asked for it on the pull
request or issue, whichever comes first, with a closing date at least the required number of days
later. The chair, or the author if the chair has not, asks the members (section 3.2) and links it
from this repository's Discussions. A revision during the period that adds, removes or changes a
requirement or rule, or changes how a verdict is reached, restarts it.

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

1. the maintainer's review, recorded on the pull request, including when the change is his own;
2. a review by each other member and committer, unless they have not given one 14 days after being
   asked; and
3. if those and any other reviews give fewer than two by people other than the author, a recorded
   second review that says who or what made it, and that it is not a maintainer approval.

Every review of safety-critical code says which files it covered and whether it read them line by
line.

Today the second review is made by an AI agent and posted from the maintainer's account. It is a
second reading, not an independent one: it does not stand in for the unaffiliated reviewer above,
and it does not lift the kit's draft notice (README, "Status"; the method's known gaps). That notice
stays until a line-by-line review of the runner, watchdog and adapters by a person not affiliated
with the maintainer is recorded, names the commit it covers, and is linked from the change that
removes it. Code changed after that commit carries the notice again until it has had the same
review.

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
objector withdraws it, it lapses, or a second member is eligible and the group decides. Revising the
change does not by itself end an objection: the author asks the objector whether the revision meets
it, and the lapse rule applies to that question. An objection that breaches the Code of Conduct is
not counted. The record lists every objection, including any not counted, and why.

Appointments of members and committers do not wait for objections, but an objection that the
nominee is not eligible is answered on the issue, with facts, before the appointment is made.

**Sunset.** From [TO DECIDE: date; proposed 30 April 2027], freezing a method version, a method
change, a requirements-and-scope change, a results-policy change and a change to this document need
group approval with at least two eligible members, and cannot be decided under this section; until
two members are eligible, they wait. Everything else continues: errata are recorded (and corrected
only in a later version), and editorial and documentation changes, code, safety-critical code under
section 3.4, result submissions, appointments and removals go on as before. A physical run needs a
frozen text (method §1.1), so a version not frozen by then cannot be used for one until a second
member takes part.

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
   never a reproduction (method §9.7 item 2). No committer merges a result from a lab they are
   affiliated with if a committer who is not can. Results labelled "maintainer's own" are merged by
   a committer not affiliated with the maintainer; if none has done so 14 days after being asked,
   the maintainer may merge them and says so on the pull request.
4. Nobody says or implies, in the project's name or as a member or committer, that a person,
   organization or standards body endorses, supports, uses or has reviewed this work, unless that
   party has said so publicly and the statement links to it. A presentation, or the receipt of a
   contribution, is not an endorsement. A member's or committer's role here is not an endorsement by
   their employer, and employers are not listed as supporters.
5. No method, requirements or results-policy change may make running a test, or submitting a
   result, need a particular protocol, vendor, model or registry, RCAN and the Robot Registry
   Foundation registry included.

## 6 Conduct, sign-off and licences

Everyone follows the [Code of Conduct](CODE_OF_CONDUCT.md). The community leaders it names are the
steering group's members, and a report about one member goes to the others. While the maintainer is
the only member, there is no one else to receive a report about him; this gap closes when a second
member is seated.

Every commit is signed off under the Developer Certificate of Origin, as
[CONTRIBUTING.md](CONTRIBUTING.md#sign-off-dco) describes. An AI agent cannot make that
certification, so the person who directs the agent signs off its commits before they are merged.

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
period, and is the nomination of any member it names. Section 2.2 applies to those nominations: the
maintainer decides one appointment alone, and any other is decided by group approval once that
member is seated. Pull requests open when it is adopted follow it; their comment periods open as
section 3.3 says.

Moving the work to a steward, or handing the method to a standards body, is a change to this
document that needs at least two eligible members. A body that the maintainer, or anyone affiliated
with him, controls is not a neutral steward; the Robot Registry Foundation is not one while its
draft charter makes the maintainer its founding chair. A standards body's process governs what it
publishes; this repository follows this document until the change is made.
