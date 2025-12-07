---
id: 1
title: Update Project Constitution
stage: constitution
date_iso: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: AI Chatbot
branch: 001-docusaurus-ai-chatbot
user: user
command: /sp.constitution
labels: ["constitution", "project-setup", "ai-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
---

## Prompt

```text
> I am building an **AI-powered documentation chat bot** for my Docusaurus website.
>
> The bot will use a **Retrieval-Augmented Generation (RAG)** system with a **Qdrant** vector database and an **OpenAI Embeddings** model to index my Docusaurus markdown files.
>
> The backend will be a **FastAPI** server that utilizes an **Anthropic (Claude)** model for the final chat response, using the retrieved context from Qdrant.
>
> The frontend will be a **React component** (the chat widget) integrated directly into the Docusaurus site.
```

## Response

The project constitution has been updated to version 0.7.0, incorporating new principles for the AI chatbot architecture, including RAG system, Qdrant, OpenAI Embeddings, FastAPI, Anthropic Model, and React Chat Widget. Several templates require pending updates. A commit message is suggested.
