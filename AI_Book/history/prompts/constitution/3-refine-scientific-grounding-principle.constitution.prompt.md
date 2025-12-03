---
ID: 3
TITLE: Refine scientific grounding principle
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
  - scientific-grounding
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
All technical and theoretical claims must be backed by current research (ROS 2, Isaac Sim, VLA). Avoid speculative claims without academic citations. All concepts must be explained in a sequence that builds from fundamentals (Nodes $\rightarrow$ URDF $\rightarrow$ Simulation $\rightarrow$ Advanced AI).
```

## Response

The project constitution has been updated to version 0.1.1 with a refined principle: "Scientific Grounding and Currency". This update clarifies the expectation that all technical and theoretical claims must be backed by current research, specifies key research areas (ROS 2, Isaac Sim, VLA), and mandates a sequential explanation of concepts from fundamentals to advanced topics.
