---
ID: 002
TITLE: Create app.py script
STAGE: misc
DATE_ISO: 2025-12-07
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: docusaurus-ai-chatbot
BRANCH: 001-docusaurus-ai-chatbot
USER: User
COMMAND: /sp.implement
LABELS: ["fastapi", "chatbot", "qdrant", "openai"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - app.py
  - specs/001-docusaurus-ai-chatbot/tasks.md
TESTS_YAML: []
---

### PROMPT_TEXT

```
> Create the full Python code for a FastAPI application named 'app.py' that:
> 1. Installs and imports necessary libraries: `fastapi`, `uvicorn`, `qdrant-client`, `openai`, `pydantic`, `python-dotenv`, and `langchain`.
> 2. Loads environment variables and initializes the Qdrant client and OpenAI embeddings for retrieval.
> 3. Includes `CORSMiddleware` configured to allow requests from your local Docusaurus development server (e.g., `http://localhost:3000`).
> 4. Defines a simple Pydantic model for the incoming chat request (e.g., `ChatQuery(query: str)`).
> 5. Creates a POST endpoint `/chat` that accepts the user query.
> 6. Inside the endpoint:
>    a. Use the Qdrant client and embeddings to perform a similarity search (`client.search`) on the `docusaurus_docs` collection to retrieve the top 3-5 relevant document chunks.
>    b. Construct a prompt that includes the retrieved document chunks as **context** and the user's **query**.
>    c. Use the **OpenAI Agents SDK** (or a simple call to the Anthropic/OpenAI API) to generate a response.
```

### RESPONSE_TEXT

Created the `app.py` script for a FastAPI application that serves as an AI chatbot backend. It handles environment variable loading, Qdrant integration for document retrieval, OpenAI embeddings for similarity search, and uses `CORSMiddleware` for cross-origin requests. It defines a `/chat` endpoint to process user queries, retrieve relevant documentation context, and generate responses using OpenAI. Also updated `specs/001-docusaurus-ai-chatbot/tasks.md` to mark the task as completed.

