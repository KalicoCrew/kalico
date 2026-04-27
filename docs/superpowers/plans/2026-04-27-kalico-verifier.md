# kalico-verifier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship the `kalico-verifier` adversarial-claim-checking subagent end to end — agent definition, slash command, orchestrator integration, plus two round-trip smoke tests proving the agent behaves as specified.

**Architecture:** Three artifact files under `.claude/` (one new agent, one new slash command, one edit to the orchestrator command), zero source-code changes, smoke-test record under `docs/superpowers/spikes/`. The agent writes only under `docs/research/` and reads everywhere else.

**Tech Stack:** Claude Code agent / command markdown definitions (frontmatter + system prompt). No build, no tests-as-code; correctness is validated by two smoke-test dispatches via the `Agent` tool against the live agent definition.

**Spec:** `docs/superpowers/specs/2026-04-27-kalico-verifier-design.md` (commit `ce2d5029`).

---

## Files

- **Create** `.claude/agents/kalico-verifier.md` — agent definition (frontmatter + system prompt), Task 2.
- **Create** `.claude/commands/verify-logic.md` — slash command wrapper, Task 3.
- **Modify** `.claude/commands/kalico-orchestrate.md` — add `[VERIFY]` tag row + dispatch-routing paragraph, Task 4.
- **Create** `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md` — smoke-test plan + results, Tasks 1, 5, 6.

No edits anywhere else. `docs/research/` may receive new files only as a side effect of the smoke tests, not as part of plan execution itself.

---

## Task 1: Draft smoke-test claims

**Files:**
- Create: `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`

The smoke-test doc lives under `docs/superpowers/spikes/` and serves two purposes: (a) it records what we're going to dispatch the verifier against, and (b) Tasks 5 and 6 will append the verifier's actual responses so the smoke tests are reproducible / re-runnable.

We need two synthetic claims, both in the kalico domain so the agent's project-aware reading is exercised:

- **Claim A (known-incorrect):** "The convolution of a degree-`p` non-rational B-spline curve with a degree-`q` polynomial kernel is a non-rational B-spline of degree `p+q` whose knot vector is identical to the knot vector of the input." This is wrong — convolution does not preserve the input's knot vector; the result has a richer knot structure. The verifier should produce `INCORRECT` with a concrete counterexample or a citation that pins the actual knot-structure result.
- **Claim B (known-correct):** "At any point along a `C¹`-smooth NURBS path with finite curvature `κ`, the centripetal-acceleration constraint `v² · κ ≤ a_max` (where `v` is tangential speed) is a valid upper bound on permissible speed under a constant `a_max`." This is correct *under stated assumptions*; a competent verifier should return `VERIFIED` but populate `Unchecked assumptions` with at least the cusp / zero-radius / `C⁰`-junction caveats and probe at least one degenerate case.

- [ ] **Step 1: Write the smoke-test doc**

Create `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md` with this content:

```markdown
# kalico-verifier smoke tests

**Date:** 2026-04-27
**Purpose:** Validate that `.claude/agents/kalico-verifier.md` behaves per spec on a known-incorrect claim and a known-correct claim. Re-runnable.

## Smoke test A — known-incorrect claim

### Claim
The convolution of a degree-`p` non-rational B-spline curve with a degree-`q` polynomial kernel is a non-rational B-spline of degree `p+q` whose knot vector is identical to the knot vector of the input.

### Why we expect INCORRECT
Convolution of B-splines does not preserve the input knot vector. The convolved curve is a B-spline of degree `p+q`, but its knot vector is generally richer than the input's — at minimum, knot multiplicities adjust to support the higher degree, and the convolution introduces additional structure depending on the kernel's support. Standard reference: Piegl & Tiller, *The NURBS Book*, ch. 5 (NURBS multiplication and related operations); convolution-with-polynomial-kernel is a closely related operation. A correct verifier should either produce a concrete counterexample (e.g., a small worked case showing the output knot vector differs) or cite a primary source pinning the actual knot-structure result.

### Why it matters (briefing context)
Layer 0 of the grand plan claims convolution-with-polynomial-kernel is one of the algebraic operations that makes smooth-shaper pre-bake possible. If kalico builds on a wrong knot-preservation assumption, the smooth-shaper application in Layer 3 will produce malformed output. (See `CLAUDE.md` Layer 0 → "NURBS algebraic operations" and Layer 3 → "Smooth-shaper application".)

### Pointers to send
- `CLAUDE.md` (already auto-loaded for any agent in this repo)
- `docs/research/firmware-survey.md` — likely irrelevant for this claim, but lets the agent demonstrate it actually reads existing research before reaching for the web.

### Expected result
- Verdict: `INCORRECT`.
- `Adversarial findings`: at least one concrete attack — a counterexample, a primary-source citation contradicting the knot-preservation claim, or a derivation showing the output knot structure.
- `Sources consulted`: at least one primary source (Piegl & Tiller, or peer-reviewed paper on B-spline convolution).
- `Research artifact`: a new or appended doc under `docs/research/` covering B-spline / NURBS convolution knot structure (web research expected, so artifact required).

### Result
<filled in by Task 5>

---

## Smoke test B — known-correct claim

### Claim
At any point along a `C¹`-smooth NURBS path with finite curvature `κ`, the centripetal-acceleration constraint `v² · κ ≤ a_max` (where `v` is tangential speed) is a valid upper bound on permissible speed under a constant `a_max` lateral-acceleration budget.

### Why we expect VERIFIED with non-empty Adversarial findings
The relation `a_centripetal = v² · κ` is correct for a particle following a smooth curve at tangential speed `v`, and bounding it by `a_max` is the standard centripetal constraint used throughout motion planning (Sonny Jeon junction deviation, TOPP-RA centripetal constraint, etc.). A competent adversarial check should still surface real caveats: the constraint assumes `κ < ∞` (fails at cusps and at `C⁰` junctions where curvature is unbounded or undefined), assumes `v` is the *tangential* speed (and not e.g. an axis-component speed), is a *necessary* condition not a *sufficient* one in the multi-axis case (per-axis acceleration limits can be tighter), and ignores tangential acceleration (which couples to `a_max` if `a_max` is a single isotropic budget). At least the curvature-finiteness caveat must appear under `Unchecked assumptions` for the verifier to be doing its job.

### Why it matters (briefing context)
This is the core relation underpinning Layer 2's "junction velocity from curvature continuity" bullet (`CLAUDE.md` Layer 2). If the constraint has a regime where it silently fails, every junction-velocity calculation downstream is suspect.

### Pointers to send
- `CLAUDE.md` Layer 2 description (auto-loaded).
- `docs/research/firmware-survey.md` (the planner survey; junction-deviation discussion likely relevant).

### Expected result
- Verdict: `VERIFIED`.
- `Adversarial findings`: non-empty — at least one attempted attack the verifier ran (cusps, `C⁰` junctions, tangential-vs-axis-component confusion, or interaction with tangential-acceleration budget).
- `Unchecked assumptions`: at least the curvature-finiteness / cusp caveat.
- `Sources consulted`: existing research likely sufficient; web research optional.
- `Research artifact`: present iff web research occurred; otherwise the literal "No new research artifact (verified from existing knowledge)." line.

### Result
<filled in by Task 6>
```

- [ ] **Step 2: Verify the doc exists and parses**

Run: `wc -l docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md && head -3 docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`
Expected: nonzero line count; first three lines are the H1 heading + blank + bold-Date line.

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md
git commit -m "spikes: smoke-test plan for kalico-verifier (claims A, B)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Author the verifier agent definition

**Files:**
- Create: `.claude/agents/kalico-verifier.md`

The agent file is the central artifact. It encodes the inputs, workflow, output format, and hard rules from the spec verbatim into the agent's system prompt. Pinned `model: opus` per the project convention that reviewer-class subagents always run on opus.

- [ ] **Step 1: Write the agent file**

Create `.claude/agents/kalico-verifier.md` with this exact content:

````markdown
---
name: kalico-verifier
description: Use when the kalico orchestrator (or user) needs adversarial verification of a specific mathematical or algorithmic claim — derivation, complexity bound, "approach X solves Y" assertion. Returns a structured adversarial report; writes a research artifact under docs/research/ only when new web research occurred.
model: opus
---

# Kalico verifier

Your job is to **adversarially check a single mathematical or algorithmic claim**. You try to break the claim — find counterexamples, edge cases, hidden assumptions, or contradicting literature. Reaching `VERIFIED` requires having genuinely tried and failed to break the claim, not having declined to try.

You are a reviewer-class subagent. You complement `kalico-researcher` (open-ended info-gathering) and `kalico-plan-reviewer` (plan-vs-grand-plan). Your slot is targeted correctness review of a specific claim.

## Inputs

The brief you receive contains, in order:

1. **The claim.** A derivation, complexity bound, or "approach X solves Y" assertion. The invoker is responsible for stating it clearly. If the claim is too vague to verify, return `INCONCLUSIVE` with a request for tighter statement — do not guess what was meant.
2. **Pointers (optional).** Paths to specs, plans, prior `docs/research/` docs, source files. Read them yourself; do not expect the invoker to have pasted them inline.
3. **Why it matters.** One sentence on which build-order item or layer this gates. Use this to judge investigation depth.

If multiple claims are in the brief, verify the first and flag the rest at the end of your report.

## Workflow — follow in order

### 1. Restate the claim

Write the claim in your own words *before doing anything else*. This catches interpretation mismatches early. If your restatement diverges from the original in any non-trivial way, surface that explicitly in your final report.

### 2. Read existing research first

Glob `docs/research/*.md`. Grep for terms drawn from the restated claim. Read what's there. Do not re-derive what the corpus already establishes.

If the claim is already covered by an existing research doc, use it — and *also* probe whether the prior verification covered the exact regime the current claim invokes. A prior verification of "X holds in the smooth-NURBS case" does not automatically cover "X holds at a knot multiplicity > 1".

### 3. Identify gaps explicitly

Before any web search, write (in working notes — these are not in the final report) the list of sub-claims your verification depends on. Mark each as:

- **Training knowledge** — verifiable from general mathematical / algorithmic knowledge.
- **Existing research** — covered by a `docs/research/` doc you just read.
- **Gap** — needs lookup.

Only step 4 lookups address `Gap` items.

### 4. Web research — only for gaps

Use `WebSearch` for academic / reference / standards sources. Use `WebFetch` for the promising ones. **Prefer primary sources** (peer-reviewed papers, standards documents, original implementations) over secondary (tutorials, blog posts) when both cover the same point.

If a Gap item cannot be closed by available sources, retain it as `Gap — unresolved` and surface it in `Unchecked assumptions`. Do not guess.

### 5. Adversarial verification — REQUIRED

Actively try to break the claim. Concrete techniques:

- Construct counterexamples in regimes the claim implicitly assumes away (boundary cases, degenerate inputs, near-singular configurations).
- Search the literature for *contradicting* results, not just confirming ones.
- Identify hidden assumptions — and check whether kalico's actual usage satisfies them.
- For complexity claims, check the constants the claim glosses over.
- For "X solves Y" claims, find published failure modes of X in the regime relevant to kalico.

**`VERIFIED` with no adversarial attempts is a process violation, not a verdict.** If you cannot find any way to attack the claim, that itself is the report — list the attacks you considered and explain why each failed to land.

### 6. Write the research artifact — only if step 4 ran

If you performed web lookups in step 4, write a research artifact at `docs/research/<topic-slug>.md` (lowercase kebab-case, no date prefix). If a doc on the same topic already exists, **append** a new dated section rather than overwriting. Maintain frontmatter: bump `last_updated`, append to `verified_claims` and `sources`. The `Summary` may be updated; everything below it is append-only.

If you did **not** do web lookups, do not write an artifact. The structured report alone goes back.

### 7. Return the structured report

Use the exact section headings below, in this order. The orchestrator parses by heading.

```
## Claim restated
<your restatement>

## Verification approach
<what you checked, how — 2–5 sentences>

## Adversarial findings
<counterexamples tried, edge cases probed, contradicting sources. Empty list is suspicious — list the attacks attempted even when nothing broke.>

## Sources consulted
- Existing research: <relative paths under docs/research/>
- Web sources: <URL — retrieval date YYYY-MM-DD — relevance>

## Unchecked assumptions
<explicit list of what the verification did NOT cover>

## Verdict
VERIFIED | INCORRECT | INCONCLUSIVE
<one paragraph confidence note>

## Research artifact
<path to new/updated docs/research/<topic>.md, OR the literal line: "No new research artifact (verified from existing knowledge).">
```

The verdict line uses one of the three exact tokens above so the orchestrator can branch on it.

## Research-doc structure (when you write one)

```markdown
---
topic: <human-readable topic, e.g. "TOPP-RA jerk-bounded reachability">
created: YYYY-MM-DD
last_updated: YYYY-MM-DD
verified_claims:
  - YYYY-MM-DD VERIFIED — <one-line claim>
sources:
  - <URL or citation>
---

# <Topic Title>

## Summary
<2–4 sentence executive summary of what this doc establishes.>

## Verified claim — YYYY-MM-DD
<original claim, verbatim>

### Verification
<what was checked, how, against which sources>

### Sources
- <URL with retrieval date, or citation>

### Caveats / unchecked assumptions
- <list>
```

## Tools — scope

| Tool | Scope |
|---|---|
| `Read`, `Grep`, `Glob` | Full repo, read-only. |
| `Bash` | Read-only operations only. No mutating commands. |
| `WebSearch`, `WebFetch` | Unrestricted. |
| `Write`, `Edit` | **Only under `docs/research/`.** Any path outside is forbidden. |

## Hard rules

- One claim per dispatch. If the brief has more, verify the first and flag the rest at the end.
- Never edit anything outside `docs/research/`.
- Never overwrite prior dated sections in a research doc. Frontmatter and the `Summary` section may be updated on append; everything below is append-only.
- Never speculate beyond what sources support — `INCONCLUSIVE` is a valid verdict.
- Never cite a source you didn't actually fetch.
- Never skip step 5 (adversarial verification). Empty `Adversarial findings` requires explanation, not omission.
- Never write a research artifact when no web research occurred — the structured report alone goes back.
- Never read or edit source code. You verify math; implementations are checked separately.
````

- [ ] **Step 2: Verify the file exists and frontmatter is well-formed**

Run: `head -5 .claude/agents/kalico-verifier.md`
Expected output: the YAML frontmatter, `name: kalico-verifier`, `model: opus`, closing `---` on line 4.

- [ ] **Step 3: Commit**

```bash
git add .claude/agents/kalico-verifier.md
git commit -m "agents: add kalico-verifier (adversarial logic-check subagent)

Reviewer-class agent on opus. Verifies a single math/algorithmic claim,
reads docs/research/ first, web-searches gaps only, returns a structured
adversarial report, writes a docs/research/ artifact only when new web
research occurred. Per spec docs/superpowers/specs/2026-04-27-kalico-verifier-design.md.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Author the `/verify-logic` slash command

**Files:**
- Create: `.claude/commands/verify-logic.md`

Thin wrapper for explicit user invocation outside the orchestrator loop.

- [ ] **Step 1: Write the command file**

Create `.claude/commands/verify-logic.md` with this exact content:

```markdown
---
description: Adversarially verify a specific mathematical or algorithmic claim. Dispatches kalico-verifier and returns its structured report.
---

# Verify logic

Dispatch the `kalico-verifier` subagent via the `Agent` tool with `subagent_type="kalico-verifier"`. Brief contents:

1. **The claim.** Take the claim from `$ARGUMENTS` plus any pasted context above this line in the conversation. State it as precisely as you can; do not paraphrase loosely.
2. **Pointers.** List every spec / plan / research / source file the user mentioned, by relative path. Do not paste their contents — let the verifier read them.
3. **Why it matters.** One sentence inferred from the conversation: which build-order item or layer this gates, or "user-driven ad-hoc verification" if no clear context.

When the verifier returns, present its structured report verbatim to the user, prefixed by a single line summary in the form:

`Verdict: <VERIFIED|INCORRECT|INCONCLUSIVE> — <one-line gist>`

Do not attempt to second-guess or rewrite the verifier's report.
```

- [ ] **Step 2: Verify the file exists**

Run: `ls -la .claude/commands/verify-logic.md && head -3 .claude/commands/verify-logic.md`
Expected: file present, first three lines are the YAML opener, the `description:` line, and the YAML closer.

- [ ] **Step 3: Commit**

```bash
git add .claude/commands/verify-logic.md
git commit -m "commands: add /verify-logic slash command

Thin wrapper that dispatches kalico-verifier. Available everywhere, not
gated on orchestrator mode.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Wire the verifier into kalico-orchestrate

**Files:**
- Modify: `.claude/commands/kalico-orchestrate.md`

Two insertions:

1. Add a `[VERIFY]` row to the brainstormer-tag table (currently lines 64–68 with `[KNOWLEDGE]`, `[RESEARCH]`, `[DIRECTION]`).
2. Add a short paragraph immediately after that table clarifying when the orchestrator dispatches `kalico-verifier` *outside* the brainstorm flow — i.e. proactively during plan review when a math claim is spotted, or in response to a user "is this right?" question.

We do not duplicate the spec's full routing table in the orchestrator doc — the brainstorm-tag table plus the new paragraph cover all dispatch paths concisely.

- [ ] **Step 1: Add the `[VERIFY]` tag row**

In `.claude/commands/kalico-orchestrate.md`, find the existing tag table:

```markdown
| Tag | Action |
|---|---|
| `[KNOWLEDGE]` | Answer directly from `CLAUDE.md` or `firmware-survey.md`. Quote the source. |
| `[RESEARCH]` | Spawn `kalico-researcher`. **Parallel** (one Agent call per question, all in one message) when ≥2 in the same turn — invoke `superpowers:dispatching-parallel-agents` first if needed. Forward findings to brainstormer. |
| `[DIRECTION]` | **Ask the user. Never assume.** Wait for their reply, then resume the brainstormer with their answer. |
```

Replace it with (adds the `[VERIFY]` row between `[RESEARCH]` and `[DIRECTION]`):

```markdown
| Tag | Action |
|---|---|
| `[KNOWLEDGE]` | Answer directly from `CLAUDE.md` or `firmware-survey.md`. Quote the source. |
| `[RESEARCH]` | Spawn `kalico-researcher`. **Parallel** (one Agent call per question, all in one message) when ≥2 in the same turn — invoke `superpowers:dispatching-parallel-agents` first if needed. Forward findings to brainstormer. |
| `[VERIFY]` | Spawn `kalico-verifier` for adversarial check of a specific mathematical / algorithmic claim. **Parallel** dispatch when ≥2 in the same turn, same as `[RESEARCH]`. Forward the structured report back to the brainstormer. If the brainstormer used `[VERIFY]` for an open-ended info-gathering question, re-route to `kalico-researcher` instead. |
| `[DIRECTION]` | **Ask the user. Never assume.** Wait for their reply, then resume the brainstormer with their answer. |
```

- [ ] **Step 2: Add the proactive-dispatch paragraph**

Immediately after the (now four-row) tag table, before the "Resume the brainstormer with `SendMessage`…" line, insert this paragraph (preceded and followed by a blank line):

```markdown
Beyond the tag-driven path, dispatch `kalico-verifier` proactively when reviewing a brainstormer's spec or plan and you spot a non-obvious mathematical or algorithmic claim that the build-order item depends on but that is not yet covered by `CLAUDE.md` or existing `docs/research/`. Brief the verifier with the claim, pointers to the spec / plan, and one sentence on which build-order item is gated. Use the verifier's verdict the same way you'd use a `[VERIFY]` answer: forward to the brainstormer if `INCORRECT` or `INCONCLUSIVE`, accept the spec / plan claim if `VERIFIED`. Do not let pure stylistic concerns trigger a verifier dispatch — only substantive math / algorithmic correctness questions.
```

- [ ] **Step 3: Verify the edits parse and look right**

Run: `grep -n '\[VERIFY\]' .claude/commands/kalico-orchestrate.md`
Expected: at least one match in the tag table.

Run: `grep -n 'proactively' .claude/commands/kalico-orchestrate.md`
Expected: at least one match (the new paragraph).

Run: `head -75 .claude/commands/kalico-orchestrate.md | tail -15`
Expected: the four-row tag table is intact and well-formed.

- [ ] **Step 4: Commit**

```bash
git add .claude/commands/kalico-orchestrate.md
git commit -m "kalico-orchestrate: integrate kalico-verifier dispatch paths

Adds [VERIFY] brainstormer-tag row and a proactive-dispatch paragraph for
when the orchestrator spots a math claim during plan review. No change to
existing tag semantics or phase structure.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Smoke test A — known-incorrect claim

**Files:**
- Modify: `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`

Dispatch the verifier against Claim A (B-spline-convolution-preserves-knot-vector — wrong) and record the result. Acceptance: verdict is `INCORRECT`, `Adversarial findings` is non-empty with at least one concrete attack, sources cite a primary source on B-spline convolution / knot insertion, and a research artifact under `docs/research/` was written (web research expected for a primary-source citation).

- [ ] **Step 1: Dispatch the verifier**

Use the `Agent` tool with these exact parameters:

- `subagent_type`: `kalico-verifier`
- `description`: `Smoke test A — B-spline convolution`
- `prompt`:

```
**Claim:** The convolution of a degree-`p` non-rational B-spline curve with a degree-`q` polynomial kernel is a non-rational B-spline of degree `p+q` whose knot vector is identical to the knot vector of the input.

**Pointers:**
- `CLAUDE.md` (auto-loaded; see Layer 0 → "NURBS algebraic operations" and Layer 3 → "Smooth-shaper application").
- `docs/research/firmware-survey.md` (likely irrelevant — confirm by reading and discarding rather than skipping).

**Why it matters:** Layer 0 / Layer 3 of the grand plan rely on convolution-with-polynomial-kernel as the operation that lets the smooth-shaper application pre-bake into the trajectory; if knot-preservation is wrong, every shaped trajectory is malformed.
```

Capture the entire returned structured report.

- [ ] **Step 2: Validate the response shape**

Check, in order:

1. The response contains the seven required headings: `## Claim restated`, `## Verification approach`, `## Adversarial findings`, `## Sources consulted`, `## Unchecked assumptions`, `## Verdict`, `## Research artifact`.
2. The verdict line is one of `VERIFIED`, `INCORRECT`, `INCONCLUSIVE`.
3. `Adversarial findings` is non-empty.
4. `Research artifact` either points to a real path under `docs/research/` (which now exists on disk) or contains the literal "No new research artifact (verified from existing knowledge)." line.

If any of these fail, the agent definition has a problem — proceed to Task 7 (refinement) before continuing.

- [ ] **Step 3: Validate the verdict against expectation**

Expected verdict: `INCORRECT`.

If the verdict is `INCORRECT` and the `Adversarial findings` references the actual issue (knot-vector enrichment under convolution, possibly with a counterexample or a primary-source citation), Smoke Test A passes.

If the verdict is anything else, Smoke Test A fails. Capture the report verbatim under the test's `### Result` block (Step 4) and proceed to Task 7.

- [ ] **Step 4: Append the result to the smoke-test doc**

In `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`, replace the `### Result\n<filled in by Task 5>` placeholder under Smoke Test A with this structure:

```markdown
### Result

**Run:** 2026-04-27
**Verdict returned:** <VERIFIED|INCORRECT|INCONCLUSIVE>
**Pass / fail:** <PASS if verdict matches expected and findings are sound; FAIL otherwise — with one-line explanation>
**Research artifact written:** <relative path or "none">

**Verifier report (verbatim):**

<paste the entire structured report here, preserving headings>
```

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md
# Also stage any new docs/research/ file the verifier wrote
git add docs/research/ 2>/dev/null || true
git status
git commit -m "spikes: smoke test A result — kalico-verifier on bad B-spline claim

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Smoke test B — known-correct claim

**Files:**
- Modify: `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`

Dispatch the verifier against Claim B (centripetal `v² · κ ≤ a_max` — correct under stated assumptions) and record the result. Acceptance: verdict is `VERIFIED`, `Adversarial findings` is non-empty (real attempts to break it must be present), `Unchecked assumptions` covers at least the curvature-finiteness / cusp caveat.

- [ ] **Step 1: Dispatch the verifier**

Use the `Agent` tool with these exact parameters:

- `subagent_type`: `kalico-verifier`
- `description`: `Smoke test B — centripetal constraint`
- `prompt`:

```
**Claim:** At any point along a `C¹`-smooth NURBS path with finite curvature `κ`, the centripetal-acceleration constraint `v² · κ ≤ a_max` (where `v` is tangential speed) is a valid upper bound on permissible speed under a constant `a_max` lateral-acceleration budget.

**Pointers:**
- `CLAUDE.md` Layer 2 (auto-loaded; see "Junction velocity from curvature continuity").
- `docs/research/firmware-survey.md` (junction-deviation discussion expected to be relevant).

**Why it matters:** Layer 2 of the grand plan derives every junction velocity from curvature continuity using exactly this relation; if it has a regime where it silently fails, downstream junction-velocity calculations are wrong.
```

Capture the entire returned structured report.

- [ ] **Step 2: Validate the response shape**

Same shape checks as Task 5 Step 2 (seven required headings, valid verdict token, non-empty `Adversarial findings`, valid `Research artifact` line).

- [ ] **Step 3: Validate the verdict against expectation**

Expected verdict: `VERIFIED`.

Required content checks:

1. `Adversarial findings` lists at least one attempted attack — e.g. cusps, `C⁰` junctions, tangential-vs-axis-component confusion, interaction with tangential acceleration.
2. `Unchecked assumptions` includes at least the curvature-finiteness / cusp caveat.

If verdict is `VERIFIED` and both content checks pass, Smoke Test B passes.

If the verdict is `VERIFIED` but `Adversarial findings` is empty or generic ("the claim is well known"), Smoke Test B fails — the verifier rubber-stamped instead of attacking. Proceed to Task 7.

If the verdict is `INCORRECT` or `INCONCLUSIVE`, capture the report and decide: did the verifier find a real flaw we missed, or did it fail to recognize a correct claim? Document the call in the `### Result` block.

- [ ] **Step 4: Append the result to the smoke-test doc**

Replace the `### Result\n<filled in by Task 6>` placeholder under Smoke Test B with the same structure as Task 5 Step 4:

```markdown
### Result

**Run:** 2026-04-27
**Verdict returned:** <VERIFIED|INCORRECT|INCONCLUSIVE>
**Pass / fail:** <PASS if verdict matches expected and adversarial findings are non-trivial; FAIL otherwise — with one-line explanation>
**Research artifact written:** <relative path or "none">

**Verifier report (verbatim):**

<paste the entire structured report here, preserving headings>
```

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md
git add docs/research/ 2>/dev/null || true
git status
git commit -m "spikes: smoke test B result — kalico-verifier on centripetal claim

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Refinement loop (only if Tasks 5 or 6 failed)

**Files:**
- Modify: `.claude/agents/kalico-verifier.md` (only if smoke tests revealed a definition flaw)
- Modify: `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md` (record the iteration)

If both smoke tests passed in Tasks 5 and 6, **skip this task entirely** and go to Task 8.

If a smoke test failed, the failure is information about which part of the agent definition is under-constrained. Common patterns and fixes:

| Failure | Likely fix in `.claude/agents/kalico-verifier.md` |
|---|---|
| Returned `VERIFIED` on Claim A (B-spline-knot-preservation) | Adversarial step too weak; tighten step 5's "Construct counterexamples" bullet with an explicit "even for claims that look textbook-standard" instruction. |
| Returned `INCORRECT` on Claim B (centripetal) without a real flaw | Adversarial step too aggressive; soften with "if you genuinely cannot construct a counterexample, prefer `VERIFIED` with documented assumptions over `INCORRECT` based on potential misuse." |
| Empty `Adversarial findings` on Claim B | Hard-rule "Never skip step 5" needs a stronger phrasing — something like "An empty `Adversarial findings` section is grounds for the orchestrator to reject the report and re-dispatch." |
| Skipped reading `docs/research/` | Step 2 is being treated as optional; tighten with "you MUST glob and grep `docs/research/` before any web search; record what you read in `Sources consulted`." |
| Wrote a research artifact when no web research occurred | Step 6 + the corresponding hard rule is being misread; reinforce with a one-line check at the top of step 6: "Did you call `WebSearch` or `WebFetch` in step 4? If no, skip step 6 entirely." |

- [ ] **Step 1: Identify the failure mode**

Re-read the failing smoke test's `### Result` block. Pick the row from the table above that matches, or — if no row matches — write a one-paragraph diagnosis in working notes.

- [ ] **Step 2: Edit `.claude/agents/kalico-verifier.md` with the targeted fix**

Apply the smallest change that addresses the diagnosis. Do not rewrite the agent file wholesale.

- [ ] **Step 3: Append an iteration log to the smoke-test doc**

Add a section at the bottom of `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md`:

```markdown
## Iteration log

### 2026-04-27 — round 2
**Trigger:** Smoke test <A|B> failed in round 1 because <one-sentence diagnosis>.
**Fix applied:** <one-sentence description of the edit to .claude/agents/kalico-verifier.md>.
**Re-run plan:** Re-dispatch the failing smoke test only; do not re-run the passing one.
```

- [ ] **Step 4: Re-run the failing smoke test**

Repeat the Task 5 or Task 6 dispatch for the failing claim. Append the new result *underneath* the prior result in the same `### Result` block — do not overwrite. Use a sub-heading `**Round 2 — 2026-04-27**` to disambiguate.

- [ ] **Step 5: Commit the iteration**

```bash
git add .claude/agents/kalico-verifier.md docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md
git add docs/research/ 2>/dev/null || true
git status
git commit -m "kalico-verifier: refine after smoke-test round 1

<one-line description of the diagnosis>

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 6: Iteration cap check**

If round 2 also fails, repeat Steps 1–5 for round 3. **Hard cap: 3 rounds.** If round 3 fails, stop and surface the situation to the user — the agent definition has a structural problem that warrants design-level discussion, not another tweak.

---

## Task 8: Final cleanup and confirmation

**Files:**
- (Read-only review across the artifacts produced.)

This task is the final acceptance check. By the time it runs, all of the following must be true:

1. `.claude/agents/kalico-verifier.md` exists with `model: opus`.
2. `.claude/commands/verify-logic.md` exists.
3. `.claude/commands/kalico-orchestrate.md` contains the `[VERIFY]` row and the proactive-dispatch paragraph.
4. `docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md` contains a PASS verdict on both Smoke Test A and Smoke Test B (possibly via a Task 7 round-2 iteration).
5. Any research artifacts the verifier wrote during smoke tests are under `docs/research/` and are committed.
6. The git log on this branch ends with a clean commit and `git status` is clean.

- [ ] **Step 1: Verify each artifact is in place**

Run, in sequence:

```bash
ls -la .claude/agents/kalico-verifier.md
ls -la .claude/commands/verify-logic.md
grep -c '\[VERIFY\]' .claude/commands/kalico-orchestrate.md
grep -c 'proactively' .claude/commands/kalico-orchestrate.md
grep -E '^\*\*Pass / fail:\*\*' docs/superpowers/spikes/2026-04-27-kalico-verifier-smoke-tests.md
```

Expected:
- The two agent / command files are present and non-empty.
- `[VERIFY]` appears at least once in `kalico-orchestrate.md`.
- `proactively` appears at least once in `kalico-orchestrate.md`.
- At least two `Pass / fail:` lines in the smoke-test doc, both reading `PASS`.

- [ ] **Step 2: Verify git tree is clean**

Run: `git status`
Expected: `nothing to commit, working tree clean`.

If anything is uncommitted, commit it with a descriptive message before proceeding.

- [ ] **Step 3: Surface remaining open items, if any**

If both smoke tests passed cleanly with no Task 7 iterations, this work is done. Confirm to the user that `kalico-verifier` is ready to use — both via the orchestrator (autodispatch on `[VERIFY]` and proactively during plan review) and via the `/verify-logic` slash command.

If any Task 7 iterations occurred, summarize them: which smoke test failed, what was tightened in the agent prompt, and whether you'd recommend a follow-up round of brainstorming on the agent definition.

---

## Done condition

The plan is complete when Task 8 Step 3 has been delivered to the user — i.e. all artifacts are in place, both smoke tests are PASS, the working tree is clean, and any iteration history has been summarized.
