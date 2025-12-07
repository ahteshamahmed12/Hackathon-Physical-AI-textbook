# Tasks for FastAPI Application Creation

## Phase: Core Development

### Task: Create app.py script
- **Description:** Create the full Python code for a FastAPI application named 'app.py' that:
    1. Installs and imports necessary libraries: `fastapi`, `uvicorn`, `qdrant-client`, `openai`, `pydantic`, `python-dotenv`, and `langchain`.
    2. Loads environment variables and initializes the Qdrant client and OpenAI embeddings for retrieval.
    3. Includes `CORSMiddleware` configured to allow requests from your local Docusaurus development server (e.g., `http://localhost:3000`).
    4. Defines a simple Pydantic model for the incoming chat request (e.g., `ChatQuery(query: str)`).
    5. Creates a POST endpoint `/chat` that accepts the user query.
    6. Inside the endpoint:
        a. Use the Qdrant client and embeddings to perform a similarity search (`client.search`) on the `docusaurus_docs` collection to retrieve the top 3-5 relevant document chunks.
        b. Construct a prompt that includes the retrieved document chunks as **context** and the user's **query**.
        c. Use the **OpenAI Agents SDK** (or a simple call to the Anthropic/OpenAI API) to generate a response.
- **File:** `app.py`
- [X] Implement the script

### Task: Create ChatWidget.js component
- **Description:** Create a standalone Docusaurus React component named 'ChatWidget.js' to be placed in the `src/components` folder. This component must:
    1. Render a simple chat interface (an input field and a "Send" button).
    2. Use `useState` to manage the chat history and the current user input.
    3. Implement an async function to handle sending a message:
        a. It should call the FastAPI backend's `/chat` endpoint (e.g., `http://localhost:8000/chat`) with the user's query using the `fetch` API.
        b. It should update the chat history with the user's message and the bot's response.
    4. Include basic styling to make it look like a simple chat box.
    5. Provide instructions on where to import and use the component (e.g., in a Docusaurus layout or a specific documentation page).
- **File:** `src/components/ChatWidget.js`
- [ ] Implement the component
