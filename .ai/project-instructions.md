# Project Instructions

This file provides stable repository-specific guidance to any AI assistant working in this repository.

## AI Workspace

- `AGENTS.md` is the canonical entrypoint for agent instructions in this repo.
- `.ai/coding-workspace.md` — symlink to shared conventions in `ai-common` (folder structure, memory/sessions split, working conventions). Read that first for how `.ai/` works.
- `.ai/user_profile.md` — symlink to shared preferences in `ai-common`.
- `.ai/project-instructions.md` (this file) holds stable, repo-specific facts that don't belong in `AGENTS.md`. Keep it focused on durable project facts; avoid temporary debugging notes or session logs.
- `.ai/memory/results/ptc10k-ch-interlock-design.md` is the authoritative hardware design note for the interlock sketch (settled design, matches the implemented firmware), referenced from `AGENTS.md` and indexed in `.ai/memory/MEMORY.md`. Keep it in sync with `PTC-voltage-interlock/PTC-voltage-interlock.ino` when either changes.

## Project Overview

Two independent Arduino sketches for ECDL v3 control electronics — no shared build system between them. See `AGENTS.md` for architecture and pin maps.
