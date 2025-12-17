Outline a comprehensive, step-by-step development plan for building and deploying the AI-powered Retrieval-Augmented Generation (RAG) chatbot integrated into the Docusaurus book website for "Physical AI Humanoid Robotics" hosted at https://physical-ai-humanoid-robotics-kohl.vercel.app. The plan must adhere to the project specifications, using Cohere APIs exclusively (embed-english-v3.0 for embeddings and command-r or command-r-plus for generation), Qdrant Cloud Free Tier for vector storage, Neon Serverless Postgres for relational data, FastAPI for the backend, and custom JS/React for frontend. Ensure the plan covers setup, development, testing, and deployment phases, with timelines, dependencies, and milestones. Focus on ensuring 100% book-grounded responses, security, and the mandatory project structure.

Phase 1: Project Setup and Environment Preparation (Days 1-2)
Step 1.1: Gather Requirements and Tools
Review all provided project details, including objectives, core requirements, ingestion pipeline, RAG flow, security, and structure.
Sign up for necessary accounts: Cohere API key, Qdrant Cloud Free Tier, Neon Serverless Postgres.
Install prerequisites: Python 3.10+, Node.js for Docusaurus, Git for version control.

Step 1.2: Initialize Project Structure
Create root directory: book-ai-chatbot/
Set up subdirectories: backend/, backend/app/, backend/ingest/, frontend/, frontend/docusaurus/, frontend/chatbot/.
Add files: backend/requirements.txt, backend/.env.example, backend/app/main.py, rag.py, cohere_client.py, qdrant_client.py, db.py, models.py; backend/ingest/ingest_book.py; frontend/chatbot/ChatWidget.jsx, askAI.js, styles.css; README.md.
Initialize Git repository and commit initial structure.

Step 1.3: Configure Environment
Create .env file with variables: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DB_URL, DOCUSAURUS_DOMAIN.
Install backend dependencies via requirements.txt: fastapi, uvicorn, cohere, qdrant-client, psycopg2-binary or asyncpg, pydantic, python-dotenv, requests (for fetching sitemap).
Set up Docusaurus: If not existing, run npx create-docusaurus@latest frontend/docusaurus classic; configure for book site.
Milestone: Project skeleton ready, environments configured.

Phase 2: Backend Development (Days 3-7)
Step 2.1: Implement Database Clients
In db.py: Connect to Neon Postgres using asyncpg or psycopg2. Create tables: chat_sessions (id, user_id, start_time, end_time), chat_messages (id, session_id, role, content, timestamp), user_selections (id, session_id, selected_text, question, response, timestamp).
Add functions for inserting/retrieving sessions, messages, selections.

Step 2.2: Implement Cohere and Qdrant Clients
In cohere_client.py: Wrapper functions for embedding (embed-english-v3.0) and generation (command-r or command-r-plus).
In qdrant_client.py: Connect to Qdrant, create collection if needed (e.g., 'book_chunks' with vector size from Cohere embed dim, cosine distance). Add upsert and search functions with metadata filtering.

Step 2.3: Build Ingestion Pipeline
In ingest_book.py: Fetch sitemap.xml from https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml using requests.
Parse XML to get page URLs, fetch each page (assuming Markdown or HTML convertible to MD), extract title, headings (for chapter/section), content using BeautifulSoup or markdown parsers.
Chunk content: Split into 500-700 token chunks with 100-token overlap (use tiktoken or simple word count for approximation since Cohere).
Generate embeddings with Cohere, upsert to Qdrant with metadata (book_title="Physical AI Humanoid Robotics", chapter, section, source_url).
Log references to Neon DB (e.g., a chunks table if needed, or integrate with chat_sessions).
Expose via POST /ingest endpoint in main.py: Trigger ingestion, return status.

Step 2.4: Implement RAG Logic
In rag.py: Define RAG flow function.
For mode=general: Embed query, search Qdrant (top-K=5-10, filter by chapter/section if provided), retrieve chunks.
For mode=selection: Use selected_text as context (no Qdrant search).
Construct prompt: System message "Answer ONLY from provided context. If answer is missing, say 'This question is not answered in the book content.'", context with metadata, user question.
Generate response with Cohere.
Log to Neon: Create/update session, add messages/selections.

Step 2.5: Set Up FastAPI App
In main.py: Initialize FastAPI app, add CORS for Docusaurus domain, basic rate limiting (e.g., using slowapi).
Add endpoints: POST /chat (body: query, mode, selected_text optional; call rag.py, return response), POST /ingest.
In models.py: Pydantic models for request/response schemas (e.g., ChatRequest, ChatResponse).
Add error handling: Catch exceptions, return {"error": "Sorry, there was an error processing your question. Please try again."}.

Step 2.6: Test Backend
Run with uvicorn, test /ingest with sample data, /chat in both modes.
Verify strict grounding: Test queries outside context to ensure exact refusal message.
Milestone: Backend functional, ingestion complete, RAG flow tested.

Phase 3: Frontend Development (Days 8-10)
Step 3.1: Integrate Chat Widget
In ChatWidget.jsx: React component for fixed chat interface. Use state for messages, loading, errors. Input field for questions, display user/AI messages.
POST to /chat with mode=general.

Step 3.2: Implement Text Selection Feature
In askAI.js: Event listener for text selection (document.onmouseup). Show floating "Ask AI" button on selection.
On click: Prompt for question (or use default "Explain this"), capture selected text, POST to /chat with mode=selection, selected_text.
Integrate with ChatWidget to display response.

Step 3.3: Styling and UI
In styles.css: CSS for button, widget (floating, responsive).
Add loading spinner, error displays.

Step 3.4: Embed in Docusaurus
Add chatbot components to Docusaurus pages (e.g., via plugin or direct import in layout).
Test on local Docusaurus server.
Milestone: Frontend integrated, text selection and general chat working.

Phase 4: Integration, Testing, and Deployment (Days 11-14)
Step 4.1: End-to-End Testing
Ingest full book via sitemap.
Test scenarios: General questions, chapter-specific (with metadata filter), text selection explanations.
Edge cases: No answer in context, errors, rate limits.
Security: Verify CORS, env vars hidden.

Step 4.2: Documentation
In README.md: Include project overview, setup instructions (clone repo, install deps, set .env, run backend with uvicorn, frontend with yarn start, ingest via curl), usage guide, troubleshooting.

Step 4.3: Deployment
Deploy backend: e.g., Vercel or Heroku for FastAPI.
Deploy frontend: Update Docusaurus site, deploy to Vercel (integrate chatbot).
Ensure cross-origin works, run ingestion post-deployment.

Step 4.4: Final Review
Confirm no OpenAI/Gemini usage, strict grounding, production-ready.
Milestone: Fully deployed, working RAG chatbot.

Dependencies and Risks
Dependencies: API keys availability, Docusaurus site access.
Risks: Token limits in free tiers; mitigate with efficient chunking. API rate limits; add retries.

Total Timeline: 14 days, assuming full-time development.

Follow this plan to deliver a robust, book-centric chatbot.