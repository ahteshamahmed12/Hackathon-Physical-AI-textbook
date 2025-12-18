---
id: 002
title: User requested to continue
stage: general
date_iso: 2025-12-17
surface: agent
model: Gemini
feature: none
branch: main
user: user
command: "System: Please continue."
labels: ["continuation"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml:
  - history/prompts/general/002-user-requested-to-continue.general.prompt.md
tests_yaml: []
prompt_text: |
  System: Please continue.
response_text: |
  I'm ready for your next command. How can I help you?
outcome: Agent acknowledged the user's request to continue and is awaiting the next specific command.
evaluation:
  relevance: 5
  completeness: 5
  clarity: 5
  accuracy: 5
---
