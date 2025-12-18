---
id: 42
title: Add Debug Print for Gemini API Key
stage: red
date: 2025-12-17
surface: agent
model: gemini
feature: rag-chatbot-fix
branch: main
user: dell
command: /sp.implement
labels: [fastapi, debugging, env-vars, gemini]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - app.py
tests:
 - none
---

## Prompt

Error: 400 API Key not found. Please pass a valid API key. [reason: "API_KEY_INVALID"domain: "googleapis.com"metadata { key: "service" value: "generativelanguage.googleapis.com"}, locale: "en-US"message: "API Key not found. Please pass a valid API key."]INFO: 127.0.0.1:63533 - "POST /chat HTTP/1.1" 500 Internal Server ErrorError: 400 API Key not found. Please pass a valid API key. [reason: "API_KEY_INVALID"domain: "googleapis.com"metadata { key: "service" value: "generativelanguage.googleapis.com"}, locale: "en-US"message: "API Key not found. Please pass a valid API key."]INFO: 127.0.0.1:60017 - "POST /chat HTTP/1.1" 500 Internal Server Error

## Response snapshot

The recurring 400 API Key not found error suggests a persistent issue with the GEMINI_API_KEY. To diagnose if the environment variable is being loaded correctly, a debug print statement has been added to app.py to output the value of GEMINI_API_KEY at runtime.

## Outcome

- ✅ Impact: A debug print statement has been added to `app.py` to help identify the runtime value of `GEMINI_API_KEY`.
- 🧪 Tests: No new tests were added.
- 📁 Files: Modified `app.py`.
- 🔁 Next prompts: Awaiting user to restart the FastAPI application and provide the full terminal output, including the debug message.
- 🧠 Reflection: When direct environment inspection is impossible, injecting debug statements into the code itself is a viable strategy for information gathering.

## Evaluation notes (flywheel)

- Failure modes observed: Shell command execution is restricted.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
