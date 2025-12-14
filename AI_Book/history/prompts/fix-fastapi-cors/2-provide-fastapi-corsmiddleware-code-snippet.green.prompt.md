---
id: 2
title: Provide FastAPI CORSMiddleware Code Snippet
stage: green
date: 2025-12-15
surface: agent
model: gemini-1.5-pro
feature: fix-fastapi-cors
branch: 5-fix-fastapi-cors
user: none
command: /sp.implement "Provide the Python code snippet for FastAPI's CORSMiddleware. It must allow cross-origin requests from the Docusaurus URL 'https://your-docusaurus-site.vercel.app' and also allow standard methods and headers."
labels: [code-snippet, fastapi, cors, middleware]
links:
  spec: specs/005-fix-fastapi-cors/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - none
tests:
  - none
---

## Prompt

/sp.implement "Provide the Python code snippet for FastAPI's CORSMiddleware. It must allow cross-origin requests from the Docusaurus URL 'https://your-docusaurus-site.vercel.app' and also allow standard methods and headers."

## Response snapshot

Here is the Python code snippet for FastAPI's CORSMiddleware, allowing cross-origin requests from your Docusaurus site on Vercel and standard methods and headers. You can integrate this into your FastAPI application (e.g., in `app.py`).

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

origins = [
    "https://your-docusaurus-site.vercel.app",  # Your Docusaurus frontend URL
    "http://localhost",
    "http://localhost:3000",
    "http://localhost:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],  # Allows all standard methods including GET, POST, PUT, DELETE, OPTIONS
    allow_headers=["*"],  # Allows all standard headers
)

@app.get("/")
async def root():
    return {"message": "Hello World"}

# Add your other FastAPI routes here
```

## Outcome

- ✅ Impact: Provided the necessary code snippet to address the CORS issue in the FastAPI backend, enabling communication with the Docusaurus frontend.
- 🧪 Tests: No tests were provided as this was a code snippet.
- 📁 Files: None (code snippet provided, not a file modification).
- 🔁 Next prompts: The user can now integrate this snippet into their FastAPI application.
- 🧠 Reflection: Providing the exact code snippet helps the user quickly resolve the CORS configuration.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
