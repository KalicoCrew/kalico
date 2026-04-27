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
