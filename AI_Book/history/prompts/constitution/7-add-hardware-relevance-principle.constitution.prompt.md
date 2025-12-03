---
ID: 7
TITLE: Add Hardware Relevance principle to constitution
STAGE: constitution
DATE_ISO: 2025-12-04
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: main
USER: user
COMMAND: /speckit.constitution
LABELS:
  - constitution
  - principles
  - hardware-requirements
  - embodied-ai
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - .specify/memory/constitution.md
TESTS_YAML: []
---

## Prompt

```text
All modules must reference the Hardware Requirements defined in the prompt (e.g., the necessity of an RTX GPU, the role of the Jetson Orin) to maintain real-world relevance.
```

## Response

The project constitution has been updated to version 0.5.0 with a new principle: "Hardware Relevance". This principle mandates that all modules must reference the defined hardware requirements to ensure real-world applicability and guide hardware-software co-design.
