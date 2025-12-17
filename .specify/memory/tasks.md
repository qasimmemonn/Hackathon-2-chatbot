# AI-Powered RAG Chatbot: Task Breakdown

## Phase 1: Project Setup and Environment Preparation

### Task 1.1: Review Requirements and Sign Up for Services
*   **Description:** Thoroughly read all project details (objective, requirements, pipeline, flow, security, structure). Create accounts and obtain keys for Cohere, Qdrant Cloud Free Tier, Neon Serverless Postgres.
*   **Phase:** 1
*   **Dependencies:** None
*   **Estimated Effort:** 2 hours
*   **Skills/Tools:** Web browser, account management
*   **Success Criteria:** API keys obtained and noted securely.

### Task 1.2: Set Up Project Directory Structure
*   **Description:** Create the root folder `book-ai-chatbot/` and all subdirectories/files as per mandatory structure (backend/app/ with py files, ingest/, requirements.txt, .env.example; frontend/docusaurus/, chatbot/ with JSX/JS/CSS; README.md). Initialize Git repo.
*   **Phase:** 1
*   **Dependencies:** Task 1.1
*   **Estimated Effort:** 1 hour
*   **Skills/Tools:** File system, Git
*   **Success Criteria:** Structure matches exactly, initial commit done.

### Task 1.3: Configure Environment and Install Dependencies
*   **Description:** Create `.env` with placeholders for keys (COHERE_API_KEY, etc.). Populate `requirements.txt` with fastapi, uvicorn, cohere, qdrant-client, asyncpg, pydantic, python-dotenv, requests, beautifulsoup4 (for parsing). Install via pip. Set up Docusaurus if needed (npx create-docusaurus).
*   **Phase:** 1
*   **Dependencies:** Task 1.2
*   **Estimated Effort:** 3 hours
*   **Skills/Tools:** Python, Node.js, terminal
*   **Success Criteria:** Dependencies installed, .env.example committed, local Docusaurus runs.

---

## Phase 2: Backend Development

### Task 2.1: Implement Neon Postgres Database Integration
*   **Description:** In `db.py`, use `asyncpg` to connect to Neon DB. Define SQL for creating tables (chat_sessions, chat_messages, user_selections). Add async functions for CRUD operations (insert session, add message, log selection).
*   **Phase:** 2
*   **Dependencies:** Phase 1 complete
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** SQL, Python async
*   **Success Criteria:** Tables created on Neon, test inserts succeed.

### Task 2.2: Create Cohere Client Wrapper
*   **Description:** In `cohere_client.py`, import `cohere`, load API key from env. Define functions: `embed_text(texts: list[str]) -> list[list[float]]` using `embed-english-v3.0`; `generate_response(prompt: str) -> str` using `command-r-plus` (fallback to `command-r`).
*   **Phase:** 2
*   **Dependencies:** Task 2.1
*   **Estimated Effort:** 3 hours
*   **Skills/Tools:** Python, Cohere SDK
*   **Success Criteria:** Test embedding and generation with sample inputs.

### Task 2.3: Create Qdrant Client Wrapper
*   **Description:** In `qdrant_client.py`, import `qdrant-client`, connect with URL/API key. Create collection `'book_chunks'` (vector dim=1024 for Cohere embed-v3, cosine). Add `upsert_points(chunks: list[dict])` and `search(query_embedding: list[float], top_k: int=5, filters: dict=None) -> list[dict]`.
*   **Phase:** 2
*   **Dependencies:** Task 2.2
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** Python, Qdrant SDK
*   **Success Criteria:** Collection created, test upsert/search with dummy data.

### Task 2.4: Develop Book Ingestion Pipeline
*   **Description:** In `ingest_book.py`, use `requests` to fetch `sitemap.xml`. Parse XML for URLs, fetch each page, use BeautifulSoup to extract title, headings (map to chapter/section), content (convert to plain text). Chunk text (500-700 tokens, 100 overlap—use `len(text.split())/4` approx). Embed chunks, upsert to Qdrant with metadata. Log chunk refs to Neon (add `chunks` table if needed).
*   **Phase:** 2
*   **Dependencies:** Tasks 2.1-2.3
*   **Estimated Effort:** 8 hours
*   **Skills/Tools:** Python, `requests`, `BeautifulSoup`, XML parsing
*   **Success Criteria:** Run ingestion on sitemap, verify chunks in Qdrant/Neon.

### Task 2.5: Implement RAG Logic
*   **Description:** In `rag.py`, define `async def rag_query(query: str, mode: str, selected_text: str=None, filters: dict=None) -> str`. If selection: `context = selected_text`. Else: `embedding = embed(query)`, `chunks = search(embedding, filters)`, `context = format_chunks(chunks)`. `Prompt = system_msg + context + query`. `Response = generate(prompt)`. If not in context, enforce exact message.
*   **Phase:** 2
*   **Dependencies:** Task 2.4
*   **Estimated Effort:** 6 hours
*   **Skills/Tools:** Python
*   **Success Criteria:** Test both modes, confirm grounding.

### Task 2.6: Set Up FastAPI Application
*   **Description:** In `main.py`, `from fastapi import FastAPI`, add CORS, rate_limit (`slowapi`). In `models.py`, Pydantic: `ChatRequest(query=str, mode=str, selected_text=Optional[str], filters=Optional[dict])`. Endpoints: `/chat` (call `rag_query`, log to DB), `/ingest` (call `ingest_book`). Add error handlers.
*   **Phase:** 2
*   **Dependencies:** Task 2.5
*   **Estimated Effort:** 5 hours
*   **Skills/Tools:** FastAPI, Pydantic
*   **Success Criteria:** Run `uvicorn`, test endpoints with Postman/curl.

### Task 2.7: Backend Testing
*   **Description:** Write unit tests for clients, integration tests for RAG flow. Test edge cases: no context answer, filters, errors.
*   **Phase:** 2
*   **Dependencies:** Task 2.6
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** Pytest
*   **Success Criteria:** 80% coverage, all tests pass.

---

## Phase 3: Frontend Development

### Task 3.1: Develop Chat Widget Component
*   **Description:** In `ChatWidget.jsx`, use React hooks for state (messages, loading, error). Render chat history, input for query, send to `/chat` (mode=general). Handle responses.
*   **Phase:** 3
*   **Dependencies:** Phase 2 complete
*   **Estimated Effort:** 5 hours
*   **Skills/Tools:** React, JSX
*   **Success Criteria:** Widget displays, sends/receives messages.

### Task 3.2: Implement Text Selection and Ask AI
*   **Description:** In `askAI.js`, add `document.onmouseup` to detect selection, show floating button. On click, prompt question, send to `/chat` (mode=selection, `selected_text`). Integrate response into `ChatWidget`.
*   **Phase:** 3
*   **Dependencies:** Task 3.1
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** JavaScript, DOM events
*   **Success Criteria:** Button appears on select, query processes correctly.

### Task 3.3: Add Styling and UI Enhancements
*   **Description:** In `styles.css`, style widget (fixed bottom-right), button (floating), loading spinner, errors. Ensure responsive.
*   **Phase:** 3
*   **Dependencies:** Task 3.2
*   **Estimated Effort:** 3 hours
*   **Skills/Tools:** CSS
*   **Success Criteria:** UI looks clean, functional on mobile/desktop.

### Task 3.4: Integrate into Docusaurus
*   **Description:** Add `ChatWidget` and `askAI.js` to Docusaurus theme/layout (e.g., swizzle or custom plugin). Test local server.
*   **Phase:** 3
*   **Dependencies:** Task 3.3
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** Docusaurus config
*   **Success Criteria:** Chatbot appears on book pages, interacts with backend.

---

## Phase 4: Integration, Testing, and Deployment

### Task 4.1: End-to-End Testing
*   **Description:** Run full ingestion, test general/selection modes, chapter filters, no-answer cases, rate limits, CORS.
*   **Phase:** 4
*   **Dependencies:** Phase 3 complete
*   **Estimated Effort:** 6 hours
*   **Skills/Tools:** Manual testing, browser dev tools
*   **Success Criteria:** All features work, no bugs.

### Task 4.2: Write Documentation
*   **Description:** In `README.md`, add overview, setup (clone, env, run commands), usage, troubleshooting. Include proposal text.
*   **Phase:** 4
*   **Dependencies:** Task 4.1
*   **Estimated Effort:** 3 hours
*   **Skills/Tools:** Markdown
*   **Success Criteria:** Comprehensive, clear instructions.

### Task 4.3: Deploy Application
*   **Description:** Deploy backend to Vercel/Heroku, frontend/Docusaurus to Vercel. Configure env vars, run ingestion post-deploy. Verify cross-domain.
*   **Phase:** 4
*   **Dependencies:** Task 4.2
*   **Estimated Effort:** 5 hours
*   **Skills/Tools:** Vercel CLI, deployment platforms
*   **Success Criteria:** Live site with working chatbot.

### Task 4.4: Final Review and Optimization
*   **Description:** Check for Cohere-only usage, grounding enforcement, clean code. Optimize chunking if needed.
*   **Phase:** 4
*   **Dependencies:** Task 4.3
*   **Estimated Effort:** 4 hours
*   **Skills/Tools:** Code review
*   **Success Criteria:** Project meets all deliverables.

---

## Project Tracking Notes
*   **Total Estimated Effort:** ~74 hours (adjust based on experience).
*   Track progress with Git issues/branches.
*   Weekly check-ins: Review phases, mitigate risks (e.g., API limits with mocks).
*   Post-completion: Monitor usage, update for feedback.
