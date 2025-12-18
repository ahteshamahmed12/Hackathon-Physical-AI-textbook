---
id: 001
title: User greeted agent
stage: general
date_iso: 2025-12-17
surface: agent
model: Gemini
feature: none
branch: main
user: user
command: "hi"
labels: ["greeting"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml:
  - history/prompts/general/001-user-greeted-agent.general.prompt.md
tests_yaml: []
prompt_text: |
  hi
response_text: |
  Hello! I'm ready for your first command.
outcome: Agent acknowledged the user's greeting and is awaiting the first command.
evaluation:
  relevance: 5
  completeness: 5
  clarity: 5
  accuracy: 5
---
