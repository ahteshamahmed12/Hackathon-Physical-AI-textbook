---
ID: 19
TITLE: Update constitution with module content structure principle
STAGE: constitution
DATE_ISO: 2025-12-06
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: main
USER: user
COMMAND: /sp.constitution when you create a module content please add a module in a way then  a main heading then a sub heading of related content just like example main heading Module ROS2 and a sub headings one heaing is installation of ROS2
LABELS: ["constitution", "documentation", "content-guidelines"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - AI_Book/.specify/memory/constitution.md
TESTS_YAML: []
---
### Prompt

```text
when you create a module content please add a module in a way then  a main heading then a sub heading of related content just like example main heading Module ROS2 and a sub headings one heaing is installation of ROS2
```

### Response

```text
The constitution file `AI_Book/.specify/memory/constitution.md` has been updated to version `0.6.0`. I've added "Principle 12: Module Content Structure" which specifies that module content should follow a hierarchical structure with main headings and subheadings. The updated file has been staged.

New Version: 0.5.0 -> 0.6.0 (MINOR bump for new principle)
Added Principle: Principle 12: Module Content Structure
Suggested Commit Message: `docs: amend constitution to v0.6.0 (add module content structure principle)`
```
