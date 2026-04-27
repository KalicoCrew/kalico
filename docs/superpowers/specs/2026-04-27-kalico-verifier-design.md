# kalico-verifier design

**Date:** 2026-04-27
**Status:** Design (pre-plan)
**Author:** Brainstormed with the user this session.

## Overview

A reviewer-class subagent — `kalico-verifier` — whose job is to **adversarially check a specific mathematical or algorithmic claim** before kalico commits to it. It complements the existing `kalico-researcher` (open-ended info gathering) and `kalico-plan-reviewer` (plan-vs-grand-plan check) by filling the gap between them: targeted correctness review of a single derivation, complexity bound, or "approach X solves problem Y" assertion.

The verifier's loop is: read existing project research → identify gaps → web-research only the gaps → adversarially attempt to break the claim → return a structured report → write a new `docs/research/<topic>.md` artifact **only if** new web research happened, so the kalico research corpus grows over time on a one-doc-per-non-trivial-lookup basis.

## Why this is needed

The kalico grand plan rests on multiple non-trivial mathematical commitments (jerk-bounded TOPP-RA, NURBS algebraic closure under convolution, B-spline knot insertion identities, smooth-shaper convolution preserving piecewise-polynomial structure, curvature-continuity at NURBS junctions, etc.). The current dispatch options handle adjacent problems but not this one:

- `kalico-researcher` answers *what does the literature say about X?* — synthesis, not correctness.
- `kalico-plan-reviewer` checks *does this plan match the grand plan and the project's standards?* — process review, not math.
- `superpowers:code-reviewer` checks the implementation, after the math is committed.

What's missing is *is this derivation actually right?* applied at the moment a brainstormer or plan-reviewer is about to commit kalico to a multi-week implementation path on the strength of that derivation. `kalico-verifier` fills exactly that slot.

## Artifacts

Three files ship as part of this work:

1. `.claude/agents/kalico-verifier.md` — the verifier subagent definition (frontmatter + system prompt).
2. `.claude/commands/verify-logic.md` — thin slash command for explicit user invocation; dispatches `kalico-verifier`.
3. Edits to `.claude/commands/kalico-orchestrate.md` — adds the verifier to the subagent roster, defines the routing rule for when to dispatch it, and adds a `[VERIFY]` brainstormer-question tag so brainstormers can request a verification mid-Q&A.

No source-code changes. No new directories — `docs/research/` already exists.

## Subagent contract

### Frontmatter

```yaml
---
name: kalico-verifier
description: Use when the kalico orchestrator (or user) needs adversarial verification of a specific mathematical or algorithmic claim — derivation, complexity bound, "approach X solves Y" assertion. Returns a structured adversarial report; writes a research artifact under docs/research/ only when new web research occurred.
model: opus
---
```

`model: opus` matches the project rule that reviewer-class subagents always run on opus (memory `feedback_reviewers_opus.md`).

### Inputs (from invoker)

The invoker dispatches with a brief containing, in this order:

1. **The claim**, stated as precisely as possible. A derivation, a complexity bound, an "approach X solves Y" assertion. The invoker is responsible for stating it clearly; vague claims are not the verifier's problem to resolve.
2. **Relevant pointers** (optional but encouraged): paths to spec / plan / research files, code locations, prior `docs/research/` docs the invoker thinks are relevant. Pointers preferred over inline pastes — the verifier reads files itself.
3. **Why it matters** (one sentence): which build-order item or layer this gates. Lets the verifier judge the right depth of investigation.

The invoker should **not** paste large derivations inline — pointing the verifier at the spec file is preferred. Inline statement of the *headline claim* is fine and expected.

### Tools available

| Tool | Scope |
|---|---|
| `Read`, `Grep`, `Glob` | Full repo, read-only. |
| `Bash` | Read-only operations. No mutating commands. |
| `WebSearch`, `WebFetch` | Unrestricted. |
| `Write`, `Edit` | **Only under `docs/research/`.** Any path outside `docs/research/` is forbidden. |

The system prompt enforces the `docs/research/`-only write scope explicitly. The verifier never edits source code, specs, plans, or `CLAUDE.md`.

### Outputs (returned to invoker)

A markdown report with these sections, in this order:

```markdown
## Claim restated
<The verifier's own restatement of the claim. Catches interpretation drift.>

## Verification approach
<What was checked, how. 2–5 sentences.>

## Adversarial findings
<Counterexamples tried, edge cases probed, contradicting sources found. Empty list is suspicious — the verifier reports what it tried even when nothing broke. If genuinely nothing broke, list the breaking attempts that were made and why they didn't break.>

## Sources consulted
- Existing research: <relative paths under docs/research/>
- Web sources: <URL — retrieval date YYYY-MM-DD — relevance>

## Unchecked assumptions
<Explicit list of what the verification did NOT cover. Examples: "assumes f64 precision is sufficient", "assumes convergence in the strictly convex case only", "did not check behavior at the boundary u=1 of a clamped knot vector".>

## Verdict
VERIFIED | INCORRECT | INCONCLUSIVE
<One paragraph of confidence note.>

## Research artifact
<Path to new or updated docs/research/<topic>.md, OR the literal line: "No new research artifact (verified from existing knowledge).">
```

The orchestrator parses by section heading. The verdict line uses one of the three exact tokens above so the orchestrator can branch on it.

## Subagent workflow

The system prompt enforces this order:

1. **Restate the claim** in the verifier's own words, before doing anything else. Surfaces interpretation mismatch early.
2. **Search existing research first.** Glob `docs/research/*.md`, grep for relevant terms drawn from the restated claim, read what's there. Don't re-derive what the corpus already establishes.
3. **Identify gaps explicitly.** Before any web search, the verifier writes (in working notes, not the final report) the list of sub-claims the verification depends on, marking each as "covered by training knowledge", "covered by existing research", or "gap — needs lookup".
4. **Web research only for gaps.** `WebSearch` for academic / reference / standards sources; `WebFetch` for the promising ones. Prefer primary sources (peer-reviewed papers, standards documents, original implementations) over secondary (tutorials, blog posts) when both cover the same point.
5. **Adversarial verification.** Actively try to break the claim. Concrete techniques the system prompt enumerates:
   - Construct counterexamples in regimes the claim implicitly assumes away (boundary, degenerate, near-singular).
   - Search the literature for *contradicting* results, not just confirming ones.
   - Identify hidden assumptions and check whether kalico's actual usage satisfies them.
   - For complexity claims, check the constants the claim glosses over.
   - For "X solves Y" claims, find published failure modes of X.
6. **Write the research artifact** *iff* step 4 actually performed web lookups. Path: `docs/research/<topic-slug>.md`, no date prefix in the filename. If a doc on the same topic exists, **append a new dated section to it** rather than overwriting; the verifier maintains the frontmatter accordingly.
7. **Return the structured report** to the invoker.

The system prompt explicitly forbids skipping step 5 with the line:

> "VERIFIED with no adversarial attempts is a process violation, not a verdict. If you cannot find any way to attack the claim, that itself is the report — list the attacks you considered and explain why each failed to land. An empty `Adversarial findings` section is grounds for the orchestrator to dispatch a fresh verifier."

## Research-doc structure

Frontmatter + body. New file per topic; existing files get appended dated sections rather than rewritten.

```markdown
---
topic: <human-readable topic, e.g. "TOPP-RA jerk-bounded reachability">
created: 2026-04-27
last_updated: 2026-04-27
verified_claims:
  - 2026-04-27 VERIFIED — <one-line claim>
sources:
  - <URL or citation>
---

# <Topic Title>

## Summary
<2–4 sentence executive summary of what this doc establishes.>

## Verified claim — 2026-04-27
<Original claim, verbatim if possible.>

### Verification
<What was checked, how, against which sources.>

### Sources
- <URL with retrieval date, or citation>

### Caveats / unchecked assumptions
- <list>
```

When appending to an existing doc:

- Add a new `## Verified claim — YYYY-MM-DD` section at the bottom (do not edit prior sections).
- Bump `last_updated`.
- Append to `verified_claims` and `sources` in the frontmatter.
- The `Summary` section may be updated if the new finding materially changes what the doc establishes; otherwise leave it.

The frontmatter is intentionally minimal so the verifier can maintain it on append without parsing complex YAML.

## Filename convention

`docs/research/<topic-slug>.md`, lowercase kebab-case, no date prefix.

Rationale: research docs are *reference* material — long-lived, edited over time, indexed by topic. This contrasts with `docs/superpowers/specs/` and `docs/superpowers/plans/` which are dated artifacts (one per episode of brainstorming / planning). The existing `docs/research/firmware-survey.md` already follows this convention.

If two distinct verification episodes produce material that belongs in the same topic doc, they share the file via dated sections (per the structure above). Verification episodes on genuinely distinct topics get distinct files.

## Integration with kalico-orchestrate

Three changes to `.claude/commands/kalico-orchestrate.md`:

### 1. Subagent roster

Add `kalico-verifier` to the subagents listed at the top of the orchestrator's process documentation, with a one-line description matching the agent's frontmatter `description`.

### 2. Routing rule

Add a routing table for which subagent to dispatch in which situation:

| Situation | Dispatch |
|---|---|
| Open-ended technical question, gathering info | `kalico-researcher` |
| Specific claim / derivation / bound the orchestrator is about to commit to | `kalico-verifier` |
| Implementation plan to check against grand plan and standards | `kalico-plan-reviewer` |
| Code to review against a plan | `superpowers:code-reviewer` |

Concretely, the orchestrator dispatches `kalico-verifier` when:

- A brainstormer's plan rests on a derivation, complexity bound, or non-obvious math identity that the orchestrator cannot confirm from `CLAUDE.md` + existing research.
- A reviewer is about to accept a plan that contains a non-obvious mathematical argument.
- The user explicitly asks "is this right?" about a specific technical assertion.

### 3. New brainstormer-question tag: `[VERIFY]`

Brainstormers presently tag questions `[KNOWLEDGE]`, `[RESEARCH]`, or `[DIRECTION]`. Add a fourth tag:

| Tag | Action |
|---|---|
| `[VERIFY]` | Spawn `kalico-verifier` with the claim and relevant pointers. Forward the structured report back to the brainstormer. **Parallel** dispatch when ≥2 in the same turn, same as `[RESEARCH]`. |

This lets a brainstormer say "I'm relying on this derivation; please get it adversarially checked" without the orchestrator having to second-guess whether the math warrants verification.

`[VERIFY]` and `[RESEARCH]` are different intents: `[RESEARCH]` is "fill a gap in my knowledge", `[VERIFY]` is "adversarially check a claim I've already formulated". The brainstormer chooses; if it picks the wrong one, the orchestrator may rewrap (e.g. an open-ended question dressed up as `[VERIFY]` gets re-routed to `kalico-researcher`).

## Slash command — `/verify-logic`

Thin wrapper for explicit user invocation outside the orchestrator loop:

```markdown
---
description: Adversarially verify a specific mathematical or algorithmic claim. Dispatches kalico-verifier and returns its structured report.
---

# Verify logic

Dispatch the `kalico-verifier` subagent with:

- The claim from `$ARGUMENTS` (and / or pasted context above).
- Relevant pointers: any spec / plan / research / source paths the user mentioned.
- "Why it matters": one sentence inferred from the conversation, or "user-driven ad-hoc verification" if no clear context.

Return the verifier's structured report verbatim, plus a one-line summary at the top with the verdict token.
```

Available everywhere, not gated on orchestrator mode.

## Hard rules (in the verifier's system prompt)

- One claim per dispatch. If the brief contains multiple, verify the first and flag the rest at the end.
- Never edit anything outside `docs/research/`.
- Never speculate beyond what sources support — `INCONCLUSIVE` is a valid verdict.
- Never cite a source that wasn't actually fetched.
- Never skip step 5 (adversarial verification) — empty `Adversarial findings` requires explanation, not omission.
- Never overwrite prior dated sections in a `docs/research/` doc. Frontmatter and the `Summary` section may be updated on append; everything below is append-only.
- Never write a research artifact when no web research occurred — the report alone goes back.

## What is explicitly out of scope

- **Code-level correctness.** That belongs to `superpowers:code-reviewer`. The verifier checks the math; the implementation is checked separately.
- **Engineering trade-off review** ("is this the right architectural choice given constraints X, Y, Z?"). That's brainstormer + plan-reviewer territory.
- **Open-ended research.** That's `kalico-researcher`.
- **Generic / cross-project use.** This is a kalico-specific agent. A future generic version could lift the workflow into a personal skill, but is not in scope here.

## Acceptance criteria

The work is done when:

1. `.claude/agents/kalico-verifier.md` exists with the frontmatter and system prompt above.
2. `.claude/commands/verify-logic.md` exists and dispatches the agent.
3. `.claude/commands/kalico-orchestrate.md` has been updated with the roster entry, the routing table, and the `[VERIFY]` tag row.
4. A round-trip smoke test passes: the orchestrator (or a hand-driven Agent dispatch) sends the verifier a known-incorrect claim from a synthetic derivation; the verifier returns `INCORRECT` with a coherent counterexample. (Smoke-test claim drafted as part of the implementation plan, not this spec.)
5. A second smoke test on a known-correct claim returns `VERIFIED` with non-empty `Adversarial findings` (i.e., the verifier actually attempted to break it).

The plan that follows this spec will enumerate concrete tasks against these criteria.
