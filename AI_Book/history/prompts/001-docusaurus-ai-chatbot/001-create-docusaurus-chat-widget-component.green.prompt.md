---
ID: 001
TITLE: Create Docusaurus Chat Widget Component
STAGE: green
DATE_ISO: 2025-12-07
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: 001-docusaurus-ai-chatbot
BRANCH: 001-docusaurus-ai-chatbot
USER: unknown
COMMAND: /sp.implement
LABELS: ["docusaurus", "react-component", "chatbot", "frontend"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - src/components/ChatWidget.js
TESTS_YAML:
  - N/A
---

## Prompt

```text
> Create a standalone Docusaurus React component named 'ChatWidget.js' to be placed in the `src/components` folder. This component must:
> 1. Render a simple chat interface (an input field and a "Send" button).
> 2. Use `useState` to manage the chat history and the current user input.
> 3. Implement an async function to handle sending a message:
>    a. It should call the FastAPI backend's `/chat` endpoint (e.g., `http://localhost:8000/chat`) with the user's query using the `fetch` API.
>    b. It should update the chat history with the user's message and the bot's response.
> 4. Include basic styling to make it look like a simple chat box.
> 5. Provide instructions on where to import and use the component (e.g., in a Docusaurus layout or a specific documentation page).
```

## Response

The `ChatWidget.js` component was created in `src/components/ChatWidget.js`. It includes a chat interface, state management for chat history and user input, an async function to send messages to a FastAPI backend at `http://localhost:8000/chat`, and basic inline styling. Instructions for importing and using the component within a Docusaurus site are provided as an export within the component file.
