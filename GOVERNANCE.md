# Governance

This document says who decides what in the Bounded Embodiment project, and how. It works with one
maintainer and vacant seats, and tightens as seats are filled. Where it differs from
CONTRIBUTING.md, or from POLICY.md, README.md, labs/README.md or CODEOWNERS in
bounded-embodiment-results, this document applies until they are brought into line. "Method §N" is a
section of the method text, version 0.1.

**Draft.** Items marked [TO DECIDE: …] or [TO COMPLETE: …] are the maintainer's to settle before
this document is adopted.

## 1 Scope

This document covers:

- the method text in [`method/`](method/), and the named generators, scorers, profiles and formats
  it relies on (method §1.5, §11);
- core (`packages/core`) and the kit (`packages/kit`);
- the reference gates (`examples/reference-gates`), which are examples, not safety software;
- the rest of this repository; and
- in [bounded-embodiment-results](https://github.com/RobotRegistryFoundation/bounded-embodiment-results):
  its [`POLICY.md`](https://github.com/RobotRegistryFoundation/bounded-embodiment-results/blob/main/POLICY.md),
  the CI and tools that decide which submissions it accepts and how its table labels or counts them,
  its `kits.lock`, and the merging of its pull requests.

It does not cover RCAN or the Robot Registry Foundation's registry, which have their own processes.
Nothing in the method requires RCAN (method §1.6) or a registry number (method §9.3 item 3). The
envelope 0.1 and gate-decision 0.1 schemas that core ships are copies of rcan-spec schemas; the
copies are governed here and do not change when rcan-spec does. RCAN's Appendix C relies on
`docs/requirements.md` for the requirements and the levels BE-1 to BE-3; this document, not RCAN's
process, decides changes to them.

## 2 Roles

### 2.1 Maintainer

The maintainer, Craig Merry, runs the project day to day. He administers this repository,
bounded-embodiment-results and the two packages' PyPI projects. Anyone holding this access uses it
only to carry out decisions made under this document. The first other member to be seated is
offered the same access.

**What rests with the maintainer today.** Until a second member is seated: every decision (section
4); every merge, and the only human review of safety-critical code; administration of both
repositories, in a GitHub organization that also holds RCAN, and of both PyPI projects; the only
contact for conduct and security reports (CODE_OF_CONDUCT.md, SECURITY.md); POLICY.md's removal of
personal or unlawful material, which may rewrite the results repository's history; and succession,
since no one can appoint a new maintainer. After that too, the homepage and results table, which
robotregistryfoundation.org builds from his own repository `craigm26/RobotRegistryFoundation`, stay
with him, outside this document. The Robot Registry Foundation is a proposal in a draft charter, not
a legal entity, and it does not govern this project.

If the maintainer steps down, is removed or leaves seat 1, or, while another member is seated, does
not respond on the repository or by email for 90 days, he ceases to be maintainer and holds no seat,
and the other members appoint a new maintainer as they fill any seat (section 2.2).

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

**Composition.** No two members may be affiliated with each other, so the members in seats 2 and 3
are outsiders.

**Terms.** [TO DECIDE: term length and limit for seats 2 and 3, how their terms are staggered, and
whether a member whose term ends stays until the seat is filled.] Seat 1 is held by whoever is
maintainer.

**Filling a seat.** Anyone may nominate someone, themselves included, by opening an issue that names
the seat. The nominee accepts on the issue and posts their declaration of interests there. After a
14-day comment period, a seat is filled by group approval with at least two eligible members, not
under section 4. The one exception: while only one member is seated, that member decides one
appointment at a time, as chair; another seat waits until that appointee is seated. The appointment
takes effect when the decision is recorded (section 3.3), and the chair then updates the table. An
update that records a decision already made (an appointment, a resignation, the end of a term, a
removal or a new declaration) is a record, not a change to this document.

**Leaving a seat.** A member leaves their seat by resigning on the repository, at the end of their
term unless reappointed, when a change in their circumstances makes them affiliated with another
member, or when 90 days have passed since the first of three or more absences (section 3.2) and
they have not supported or objected to any decision since that first absence. Recusals don't count
as absences and don't break the run. The only seated member does not leave at the end of a term or
for absence. [TO DECIDE: whether the absence rule applies to seat 1.]

**Removal.** A member may be removed for a breach of the Code of Conduct or an undisclosed conflict
of interest, with the support of two members other than the one concerned. A removal, of a member or
a committer, is decided no sooner than 14 days after the person concerned is told the reason by
email, and they may answer. It takes effect when any member who supported it records it, and the
record gives the reason without details that would identify a reporter. A removed maintainer also
ceases to be maintainer and a committer; while he alone holds the access in section 2.1, carrying
this out depends on him.

**Conflicts of interest.** Every member, the maintainer included, posts a declaration of interests
and updates it within 30 days of a change: employer, who funds their work on robots or AI, projects
they lead, any financial stake in a project another member leads, any machine, gate, lab or product
the method could be used to test, and a contact address (sections 3.2 and 6). Every other member's
declaration also lists their contributions to the maintainer's projects in the last two years, and
every nominee's states that they are not affiliated with any member. A member recuses from a
decision in which they, or an organization they are affiliated with, have a particular interest,
such as their own appointment or removal, or a question about their own lab's results. A recused
member may comment but is not counted. The maintainer's standing interests, as declared, do not by
themselves require him to recuse.

### 2.3 Chair

The maintainer is the first chair. A chair serves until the members choose another member by group
approval, which they may do at any time, or until they leave the group; then the longest-serving
member, or of two equal the one in the lower-numbered seat, chairs until the members choose. The
longest-serving other member, chosen the same way, chairs any decision in which the chair has a
particular interest. The chair has no casting vote. Beyond any member's powers, the chair opens
comment periods, asks members for decisions, records outcomes (section 3.3) and keeps the table in
section 2.2 current. If the chair has not recorded a decision 7 days after it was made, any member
who supported it may record it.

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
the steering group decides by group approval, or the chair decides while only one member is seated.
A committer inactive for 12 months becomes emeritus and loses merge access until they ask to
return. A committer may be removed for a breach of the Code of Conduct by group approval (section
2.2, "Removal").

### 2.5 Contributors and labs

Anyone may open issues and pull requests, and comment in any comment period. Registering a lab,
submitting results or offering a testbed does not make a lab or its people members or committers,
and gives no vote. No seat is held by, or reserved for, an organization. A person who works for a
lab may hold a seat as an individual, and recuses from decisions about that lab's results.

### 2.6 AI agents

A change an AI agent writes is the change of the person who directs the agent, who is its author and
signs it off. Today that is always the maintainer. An agent is never a member or committer, and
never counts toward group approval. Its review can serve only as the added review in section 3.4,
item 3. An agent's review is headed as an agent review. It is independent, at most, of the session
that wrote the change, never of the person who directs it.

## 3 Decisions

### 3.1 Kinds of change

| Kind | What it covers | Comment period | Decided by |
|---|---|---|---|
| Method change | Freezing a method version (method §1.1). Any change to the normative text of a draft version. A new named generator, scorer, profile or format, or a new version of one (method §11) | 14 days | group approval |
| Released versions | A frozen method text; a released rule set, profile or format, and its vectors | none | never changed (method §1.1, §1.5, §11); a change is made as a new version, which is a method change |
| Erratum | An error found in a frozen method version | none | recorded as an issue and corrected in the next version |
| Requirements and scope | `docs/requirements.md`: a change to a requirement, a test in the test plan, or the assurance levels | 14 days | group approval |
| Editorial and documentation | Wording that changes no requirement or rule, in draft method text, in documentation or in `POLICY.md` | none | any committer merges |
| Code | Core, the kit apart from the safety-critical code, tools, tests, examples and the reference gates | none | any committer merges once CI passes; once there is a second committer, after a recorded review by someone other than the author, who need not be a committer |
| Safety-critical code | In `packages/kit/src/bounded_embodiment_kit/`: `runner/` (with the watchdog and interlocks), `adapters/` and `kinematics/`; any other code the runner calls to decide whether to stop, today including `packages/core/src/bounded_embodiment/envelope/`; and their tests | none | section 3.4 |
| Results policy | `POLICY.md` in bounded-embodiment-results; any change to that repository's CI or tools that changes which submissions it accepts or how its table labels or counts a result, such as removing a handle or lab from `tools/maintainer.json` or changing `tools/claim-words.txt`. Adding a handle or lab to `tools/maintainer.json`, or a released core version to `kits.lock`, is a code change | 14 days | group approval |
| Result submission | A pull request to bounded-embodiment-results that adds runs, a lab registration or a withdrawal | none | any committer merges once its CI passes and it follows `POLICY.md`; see section 5, item 3 |
| Removals under POLICY.md | Material removed under its privacy and legal rules; a `test-` lab | none | any committer; recorded (section 3.3) and reported to the other members |
| This document | Any change, other than updating the table in section 2.2 to record a decision | 14 days | group approval, within the limits of section 7 |
| Appointments and removals | Seats and committers | 14 days for a seat, 7 for a committer; a removal waits 14 days after the person is told why (section 2.2); none under section 6 | sections 2.2 and 2.4 |

A pull request that is more than one kind meets the rules of each. A results-policy change applies
only to submissions made after it is merged. A change that fits no row is handled as the strictest
kind that could apply. If anyone says, with a reason, on a pull request or within 14 days after it
was merged, that it belongs to a stricter kind, it is handled as that kind; for a merged change, its
comment period opens on the day that is said, and if the change has not been decided in its favour
when the period closes, it is reverted until it is.

A fix for a security or run-safety problem ([SECURITY.md](SECURITY.md)) in code, safety-critical
code or the results repository's CI may be merged before the reviews or comment period it needs, if
waiting would leave labs exposed. The pull request says why, the reviews are recorded within 14
days, and any comment period runs afterwards; if the decision then goes against the fix, it is
reverted.

### 3.2 Group approval

A decision by group approval is made, once its comment period has closed, when more than half of the
eligible members support it: both, if two are eligible; two, if three are.

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
from this repository's Discussions. A revision made before the decision is recorded that adds,
removes or changes a requirement or rule, or changes how a verdict is reached, restarts the comment
period. Members are asked again, and support given before the restart, other than by opening the
pull request, no longer counts.

Every decision other than a merge of editorial, documentation, code or result-submission changes is
recorded in a comment on the pull request or issue. The record gives the kind of change and the rule
applied; the commit decided, where there is one; the dates of the comment period; who supported it,
who recused and who was absent; and each objection, with how it was answered.

### 3.4 Safety-critical code

A change to safety-critical code (section 3.1) needs two reviews, recorded on the pull request, by
people other than its author, at least one of whom is not affiliated with the author. Reviewers need
not be members or committers.

While there are fewer than two members and committers other than the author, or none of them is
unaffiliated with the author, the change needs instead:

1. the maintainer's review, recorded on the pull request, including when the change is his own;
2. a review by each other member and committer, unless they have not given one 14 days after being
   asked; and
3. if those and any other reviews give fewer than two by people other than the author, one added
   recorded review that says who or what made it, and that it is not a maintainer approval.

Every review of safety-critical code says which files it covered and whether it read them line by
line. The kit's draft notice (README, "Status"; the method's known gaps) stays until a line-by-line
review of the safety-critical code by a person not affiliated with the maintainer is recorded, names
the commit it covers, and is linked from the change that removes it. Code changed after that commit
carries the notice again until it has had the same review.

## 4 Interim rule

This section applies whenever only one member is eligible for a decision (section 3.2). A decision
that needs group approval is then made by that member when:

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
nominee does not qualify (sections 2.2 and 2.4) is answered on the issue, with facts, before the
appointment is made.

**Sunset.** From [TO DECIDE: date; proposed 30 April 2027], freezing a method version, a method
change, a requirements-and-scope change, a results-policy change and a change to this document need
group approval with at least two eligible members, and cannot be decided under this section; until
two members are eligible, they wait. Everything else continues.

## 5 Independence and honesty

1. The project does not certify, approve, accredit, rate or endorse any machine, gate, lab, product
   or person, and no role in this document can do so on its behalf. Results are evidence, not
   certification: a verdict is a finding about one run (method §10 item 1), and results are not to
   be cited as certification evidence (POLICY.md, "Not certification evidence").
2. Anything the project, a member or a committer writes about results follows method §10 and
   POLICY.md, "Claim language".
3. Results from the maintainer's labs are labelled "maintainer's own" (method §10 item 4; POLICY.md,
   "The maintainer's own results"). All his labs count as one lab, so a re-run by one of them is
   never a reproduction (method §9.7 item 2). No committer merges a result from a lab they are
   affiliated with if a committer who is not can. Results labelled "maintainer's own" are merged by
   a committer or member not affiliated with the maintainer; if there is none, or none has done so
   14 days after being asked, the maintainer may merge them and says so on the pull request.
4. Nobody says or implies, in the project's name or as a member or committer, that a person,
   organization or standards body endorses, supports, uses or has reviewed this work, unless that
   party has said so publicly and the statement links to it. A presentation, or the receipt of a
   contribution, is not an endorsement. A member's or committer's role here is not an endorsement by
   their employer, and employers are not listed as supporters.
5. No change may make running a test, or submitting a result, need a particular protocol, vendor,
   model or registry, RCAN and the Robot Registry Foundation registry included.

## 6 Conduct, sign-off and licences

Everyone follows the [Code of Conduct](CODE_OF_CONDUCT.md). The community leaders it refers to are
the steering group's members. A report about a member goes to the other members, who decide what to
do, removal included, by group approval among themselves, with no comment period; the member
concerned recuses. When a member is seated, CODE_OF_CONDUCT.md and SECURITY.md are updated, as an
editorial change, to give their contact. While only one member is seated, no one else receives a
report about them.

Every commit is signed off under the Developer Certificate of Origin, and contributions are licensed
like the files they change ([CONTRIBUTING.md](CONTRIBUTING.md#sign-off-dco),
[REUSE.toml](REUSE.toml), [`LICENSES/`](LICENSES/)). An agent cannot make the DCO certification, so
the person who directs it signs off its commits (section 2.6). A change to the licences of new
contributions is decided like a change to this document.

## 7 Changing this document

A change to this document is decided as section 3.1 says. A change that does any of the following
needs group approval with at least two eligible members, and cannot be made under section 4:

- moves the sunset date in section 4 later, or removes it;
- weakens the composition rule in section 2.2, section 3.4, or the released-versions row of section
  3.1;
- lowers a support count, shortens a comment period, or adds to what one member may decide alone; or
- narrows section 5.

This document is adopted under its own section 4: the pull request that adds it has a 14-day comment
period, and is the nomination of any member it names. Pull requests open when it is adopted follow
it; their comment periods open as section 3.3 says, and one opened that way before adoption counts.

Moving the work to a steward, or handing the method to a standards body, is a change to this
document that needs at least two eligible members. The work moves only to a neutral steward: a body
that neither the maintainer nor anyone affiliated with him controls. The Robot Registry Foundation
is not one until it is incorporated and has a seated board that neither controls. A standards body's
process governs what it publishes; this repository follows this document until the change is made.
