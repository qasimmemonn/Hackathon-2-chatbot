# Docusaurus Book with Integrated AI RAG Chatbot (Cohere + Qdrant + Neon) - Technical Specification

## 1. Project Overview

**Title:** Docusaurus Book with Integrated AI RAG Chatbot (Cohere + Qdrant + Neon)

**Objective:** To create a sophisticated Retrieval-Augmented Generation (RAG) chatbot and embed it within the Docusaurus-based book website for "Physical AI Humanoid Robotics." The chatbot will serve as an intelligent assistant, allowing readers to ask general questions about the book, query specific chapters or sections, and get explanations for selected text. All responses must be strictly grounded in the book's content, with a firm prohibition on using external knowledge. This project exclusively uses Cohere APIs for embeddings and generation, Qdrant Cloud for vector storage, and Neon Serverless Postgres for relational data persistence.

## 2. Key Features

*   **General Queries:** Users can ask broad questions, and the chatbot will retrieve relevant information from the entire book.
*   **Scoped Queries:** Users can narrow their search to specific chapters or sections for more targeted answers.
*   **Text Selection Interaction:** A "Ask AI" button appears when a user highlights text, allowing them to ask a question about the selected text.
*   **Strict Grounding Policy:** If the answer to a question cannot be found within the provided context (the book's content), the chatbot must respond with: "This question is not answered in the book content." Do not use any external knowledge or information.
*   **Deployment:** The backend will be a FastAPI application, and the frontend will consist of custom React/JS components integrated into the existing Docusaurus site.
*   **Content Ingestion:** The book's content will be ingested by parsing the `sitemap.xml` located at `https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml`.

## 3. Core Requirements & Architecture

### 3.1. Frontend (Docusaurus + Custom JS/React)

*   **Integration:** The chatbot will be integrated into the existing local Docusaurus project.
*   **UI Components:**
    *   **Floating "Ask AI" Button:** A small, non-intrusive button that appears when a user selects text on the page. Clicking it will trigger a prompt for the user to ask a question about the selected text.
    *   **Fixed Chat Widget:** A persistent, collapsible chat widget (e.g., in the bottom-right corner) for general and scoped queries.
*   **Component Structure:** All custom frontend code will reside in `frontend/chatbot/`.
    *   `ChatWidget.jsx`: The main React component for the chat interface.
    *   `askAI.js`: A JavaScript module to handle text selection events and API communication.
    *   `styles.css`: Styles for the chat widget and floating button.
*   **Functionality:**
    *   The chat widget will include a user input field, a display area for the conversation history (user and AI messages), loading indicators, and error messages.
    *   On text selection, `askAI.js` will capture the highlighted content, prompt the user for a question, and send the `selected_text`, `question`, and `mode=selection` to the backend `/chat` endpoint.

### 3.2. Backend (FastAPI)

*   **Technology:** Python with the FastAPI framework.
*   **Endpoints:**
    *   `POST /chat`: The main endpoint for handling user queries. It will accept a request body containing the `query`, `mode` (`general` or `selection`), and an optional `selected_text`. It orchestrates the RAG flow and returns the AI-generated response.
    *   `POST /ingest`: A protected endpoint to trigger the book ingestion pipeline.
*   **Core Logic (in `app/rag.py`):**
    *   **Embedding:** Use Cohere's `embed-english-v3.0` model to generate vector embeddings for user queries.
    *   **Vector Search:** Interface with Qdrant to perform similarity searches, applying metadata filters for scoped queries.
    *   **Generation:** Use Cohere's `command-r` or `command-r-plus` model to generate answers based on the retrieved context.
    *   **Strict Answering:** Implement logic to strictly enforce the context-only response policy.
*   **File Structure (within `backend/`):**
    *   `app/main.py`: FastAPI application setup, middleware (CORS, rate limiting), and endpoint definitions.
    *   `app/rag.py`: Contains the core RAG logic, including prompt construction and interaction with Cohere.
    *   `app/cohere_client.py`: A wrapper for all interactions with the Cohere API.
    *   `app/qdrant_client.py`: A client for performing operations on the Qdrant vector database.
    *   `app/db.py`: Functions for interacting with the Neon Serverless Postgres database (e.g., logging chat history).
    *   `app/models.py`: Pydantic models defining the structure of API requests and responses.
    *   `ingest/ingest_book.py`: The script for the book ingestion pipeline.
    *   `requirements.txt`: A list of all Python dependencies.
    *   `.env.example`: An example file showing the required environment variables.

### 3.3. Vector Database (Qdrant Cloud Free Tier)

*   **Purpose:** To store and efficiently search through vector representations of the book's content.
*   **Chunking Strategy:** Text content will be divided into chunks of 500–700 tokens with an overlap of 100 tokens to maintain contextual integrity.
*   **Metadata:** Each vector will be stored with the following metadata for filtering:
    *   `book_title`: "Physical AI Humanoid Robotics"
    *   `chapter`: The chapter title.
    *   `section`: The section or sub-heading.
    *   `source_url`: The original URL of the page.
    *   `page`: The page number, if available.
*   **Search:** Use cosine similarity to find the top-K (K=5 to 10) most relevant chunks for a given query embedding.

### 3.4. Relational Database (Neon Serverless Postgres)

*   **Purpose:** To provide relational data persistence for chat history and analytics.
*   **Database Schema:**
    *   **`chat_sessions`**:
        *   `id`: SERIAL PRIMARY KEY
        *   `user_id`: VARCHAR(255)
        *   `start_time`: TIMESTAMP WITH TIME ZONE
        *   `end_time`: TIMESTAMP WITH TIME ZONE
    *   **`chat_messages`**:
        *   `id`: SERIAL PRIMARY KEY
        *   `session_id`: INTEGER REFERENCES chat_sessions(id)
        *   `role`: VARCHAR(10) CHECK (role IN ('user', 'ai'))
        *   `content`: TEXT
        *   `timestamp`: TIMESTAMP WITH TIME ZONE
    *   **`user_selections`**:
        *   `id`: SERIAL PRIMARY KEY
        *   `session_id`: INTEGER REFERENCES chat_sessions(id)
        *   `selected_text`: TEXT
        *   `question`: TEXT
        *   `response`: TEXT
        *   `timestamp`: TIMESTAMP WITH TIME ZONE

### 3.5. LLM & Embeddings (Cohere)

*   **Exclusive Provider:** All LLM and embedding functionalities must be implemented using Cohere APIs. The use of OpenAI, Gemini, or any other provider is strictly prohibited.
*   **Embeddings Model:** `embed-english-v3.0`
*   **Generation Model:** `command-r` or `command-r-plus`

## 4. Workflows

### 4.1. Book Ingestion Pipeline (`ingest/ingest_book.py`)

1.  **Fetch Sitemap:** Retrieve and parse the `sitemap.xml` from `https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml`.
2.  **Process Pages:** For each URL in the sitemap, fetch the page content. Use a library like BeautifulSoup to parse the HTML and extract the main content, title, and headings (which map to chapters/sections).
3.  **Chunk Text:** Apply the defined chunking strategy (500–700 tokens, 100-token overlap) to the extracted text.
4.  **Generate Embeddings:** For each chunk, call the Cohere API to generate an embedding using `embed-english-v3.0`.
5.  **Store Data:**
    *   **Qdrant:** Upsert the vectors into a Qdrant collection, along with their corresponding metadata.
    *   **Neon:** Log references to the ingested content (e.g., chunk ID, source URL, timestamp) in a dedicated table for tracking and auditing.
6.  **Idempotency:** The script must be idempotent, meaning running it multiple times will not create duplicate entries.

### 4.2. RAG Flow

1.  **Receive Input:** The `/chat` endpoint receives a user question, a `mode`, and optionally `selected_text`.
2.  **Context Retrieval:**
    *   **`mode=selection`:** The `selected_text` is used as the sole context. No vector search is performed.
    *   **`mode=general`:** The user's query is embedded using Cohere. The resulting vector is used to search Qdrant for the top-K relevant chunks. If `chapter` or `section` filters are provided, they are applied during the search.
3.  **Prompt Construction:** A detailed prompt is constructed for the Cohere generation model:
    *   **System Prompt:** "You are a helpful assistant for the book 'Physical AI Humanoid Robotics'. Answer the user's question based ONLY on the provided context. If the answer is not found in the context, you MUST say: 'This question is not answered in the book content.' Do not use any external knowledge or information."
    *   **Context:** The retrieved chunks (or `selected_text`) are concatenated, with metadata included to indicate their source (e.g., "Source: Chapter 3, Section 2\n\n[...chunk text...]\n\n").
    *   **User Question:** The user's original query.
4.  **Generate Response:** The complete prompt is sent to the Cohere `command-r` (or `command-r-plus`) model to generate a response.
5.  **Log Interaction:** The user's message and the AI's response are logged to the Neon database.
6.  **Return Response:** The final, generated response is sent back to the frontend.

## 5. Security, Quality, and Best Practices

*   **Environment Variables:** All sensitive information (API keys for Cohere, Qdrant; database connection string for Neon) must be managed through a `.env` file. A `.env.example` file must be provided.
*   **CORS:** The FastAPI backend must have Cross-Origin Resource Sharing (CORS) enabled, but restricted to the Docusaurus site's domain to prevent unauthorized access.
*   **Rate Limiting:** Implement basic rate limiting on the API endpoints (e.g., 10 requests per minute per IP) using a FastAPI middleware to prevent abuse.
*   **Error Handling:** The application should gracefully handle exceptions (e.g., API failures, database errors) and return user-friendly error messages to the frontend.
*   **Code Quality:** Adhere to clean code principles. Use type hints throughout the Python codebase. Employ `async` and `await` for I/O-bound operations where applicable.
*   **Testing:** While optional for the initial delivery, creating unit and integration tests for the backend logic is highly recommended.

## 6. Project Structure

The project must follow this directory structure:

```
book-ai-chatbot/
├── backend/
│   ├── app/
│   │   ├── main.py
│   │   ├── rag.py
│   │   ├── cohere_client.py
│   │   ├── qdrant_client.py
│   │   ├── db.py
│   │   └── models.py
│   ├── ingest/
│   │   └── ingest_book.py
│   ├── requirements.txt
│   └── .env.example
├── frontend/
│   ├── docusaurus/  // Docusaurus project root for integration
│   └── chatbot/
│       ├── ChatWidget.jsx
│       ├── askAI.js
│       └── styles.css
└── README.md
```

## 7. Final Deliverable

*   A fully functional, production-ready RAG chatbot embedded in the Docusaurus website.
*   The chatbot must be powered exclusively by Cohere and integrated with Qdrant and Neon as specified.
*   The "Ask AI" on text selection feature must be fully implemented.
*   A comprehensive `README.md` file that includes:
    *   An overview of the project (using the provided text).
    *   Detailed setup instructions (environment setup, running the backend and frontend).
    *   Instructions on how to run the ingestion script.
    *   Usage examples and troubleshooting tips.
