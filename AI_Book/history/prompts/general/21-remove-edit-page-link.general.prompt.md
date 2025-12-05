---
ID: 21
TITLE: Remove 'edit this page' link from module texts
STAGE: general
DATE_ISO: 2025-12-06
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: main
USER: user
COMMAND: /sp.tasks in the end of module text there was a link edit the page please remove this link
LABELS: ["documentation", "docusaurus", "ui-fix"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - AI_Book/docusaurus.config.ts
TESTS_YAML: []
---
### Prompt

```text
in the end of module text there was a link edit the page please remove this link
```

### Response

```text
I have successfully removed the `editUrl` configuration from `AI_Book/docusaurus.config.ts`, which should remove the "edit this page" link from your module texts. The modified file has been added to the staging area.
```
