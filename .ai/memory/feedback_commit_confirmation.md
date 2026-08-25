---
name: feedback-commit-confirmation
description: "User wants explicit confirmation before each git commit during the v2 retrofit logbook workflow, not an auto-commit after every logged step"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f9f117ee-3403-4b3b-b48c-72388a07e141
  modified: 2026-08-25T04:05:41.002Z
---

Batch retrofit-logbook updates and related edits locally, and commit only after the user confirms.

**Why:** During the v3-software-v2 retrofit bring-up, I had been committing after each logbook entry / firmware fix without asking. The user asked to confirm before each commit instead. See also [[feedback_tone_style]] — the user's own phrasing preference is positive framing, applied here too.

**How to apply:** In this repo (and generally, absent other instruction), make edits and logbook updates locally, then ask "ready to commit?" or wait for the user to say so. Batch related changes into fewer, deliberate commits rather than one per micro-step.
