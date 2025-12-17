---
id: 2
title: Define RAG Chatbot Technical Specification
stage: spec
date: 2025-12-17
surface: agent
model: gemini-pro
feature: rag-chatbot
branch: master
user: user
command: /sp.specify
labels: [specification, rag, chatbot, cohere]
links:
  spec: .specify/memory/specification.md
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/specification.md
tests:
 - "N/A"
---

## Prompt

sp.specify Define the complete technical and functional specifications for implementing an AI-powered Retrieval-Augmented Generation (RAG) chatbot integrated into the Docusaurus book website for "Physical AI Humanoid Robotics" hosted at https://physical-ai-humanoid-robotics-kohl.vercel.app. The chatbot must use Cohere APIs exclusively (embed-english-v3.0 for embeddings and command-r or command-r-plus for generation), Qdrant Cloud Free Tier for vector storage, Neon Serverless Postgres for relational data, FastAPI for the backend, and custom JS/React for frontend integration. Ensure the specification covers all aspects from ingestion to user interaction, enforcing strict book-grounded responses.Project OverviewTitle: Docusaurus Book with Integrated AI RAG Chatbot (Cohere + Qdrant + Neon)Objective: Create a RAG chatbot embedded in the Docusaurus site that allows readers to ask general questions about the book, query specific chapters/sections, or select text for targeted explanations. All answers must be 100% grounded in book content, with no external knowledge. Prohibit use of OpenAI or Gemini APIs—use Cohere only.Key Features:General queries: Retrieve from entire book.Chapter/section-specific: Filter via metadata.Text selection: Answer based solely on selected text.Strict response policy: If not in context, reply "This question is not answered in the book content."Deployment: Backend via FastAPI, frontend in Docusaurus with custom components. Ingest book from sitemap.xml at https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml.Core RequirementsFrontend (Docusaurus + Custom JS/React)Use existing or local Docusaurus site.Floating "Ask AI" Button:Appears on text selection.Triggers sending selected text + user question to backend in mode=selection.Fixed Chat Widget:Always available for general queries (mode=general).UI Elements: User input field, message display (user/AI), loading spinner, error messages.Integration: Custom JS/React components in /frontend/chatbot/ (ChatWidget.jsx, askAI.js, styles.css).Handle selected text: On selection, capture text, prompt for question, send to /chat endpoint.Backend (FastAPI)Python-based, with endpoints:POST /chat: Receives query, mode (general/selection), optional selected_text. Processes RAG flow and returns response.POST /ingest: Triggers book ingestion pipeline.Handles:Embedding: Cohere embed-english-v3.0.Vector Search: Qdrant with metadata filtering (book_title, chapter, section, source_url, page if available).Generation: Cohere command-r or command-r-plus.Strict Answering: Enforce context-only responses.Files: main.py (app setup), rag.py (RAG logic), cohere_client.py (API wrappers), qdrant_client.py (vector DB ops), db.py (Neon DB ops), models.py (Pydantic models for requests/responses).Vector Database (Qdrant Cloud Free Tier)Store book chunks as vectors.Chunking Strategy: 500–700 tokens per chunk, 100-token overlap.Metadata: book_title="Physical AI Humanoid Robotics", chapter, section, source_url (from sitemap).Search: Top-K (e.g., 5-10) relevant chunks based on query embedding cosine similarity.Relational Database (Neon Serverless Postgres)Store:Chat history: User sessions, messages.Selected text logs.Tables:chat_sessions: id (PK), user_id, start_time, end_time.chat_messages: id (PK), session_id (FK), role (user/ai), content, timestamp.user_selections: id (PK), session_id (FK), selected_text, question, response, timestamp.Use for persistence; query history if needed for context (but not mandatory for basic flow).LLM & EmbeddingsExclusive Use: Cohere APIs only.Embeddings: embed-english-v3.0.Generation: command-r or command-r-plus.No integration with OpenAI, Gemini, or other providers.Book Ingestion PipelineSource: Parse Markdown files via sitemap.xml at https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml.Process (in ingest_book.py):Fetch and parse pages: Extract title, headings (for chapter/section), content.Chunk text: Apply strategy, generate Cohere embeddings.Store: Upsert vectors to Qdrant with metadata.Log references: Insert into Neon DB for tracking (e.g., chunk_id, source_url).Run via /ingest endpoint or script; ensure idempotency.RAG FlowInput: User question (with optional selected_text and mode).Steps:If mode=selection: Use selected_text as sole context (embed if needed, but generate directly).If mode=general: Embed query with Cohere, search Qdrant (filter by chapter/section if specified), retrieve top-K chunks.Construct Prompt:System: "Answer ONLY from provided context. If answer is missing, say 'This question is not answered in the book content.' Do not add external information."Context: Concatenated chunks/selected_text with metadata (e.g., "From Chapter X, Section Y: [text]").User: Question.Generate response with Cohere.Log to Neon: Save message, session.Return to frontend.Error Handling: Catch exceptions, return user-friendly errors.Security & QualityEnv Vars: Use .env for API keys (Cohere, Qdrant, Neon), via .env.example.CORS: Enable for Docusaurus domain only.Rate Limiting: Basic (e.g., using FastAPI middleware, 10 req/min per IP).Error-Safe: Graceful failures, minimal logging.Structure: Follow mandatory tree:book-ai-chatbot/├── backend/│   ├── app/ (files as listed)│   ├── ingest/ (ingest_book.py)│   ├── requirements.txt│   └── .env.example├── frontend/│   ├── docusaurus/ (integrate here)│   └── chatbot/ (JSX/JS/CSS files)└── README.md (setup instructions, overview).Best Practices: Clean code, type hints, async where possible, tests optional but recommended.Final DeliverableProduction-ready code: Fully functional chatbot embedded in Docusaurus.Powered by Cohere, integrated with Qdrant + Neon.Supports text selection "Ask AI".Clear README: Setup (env setup, run backend/frontend, ingest), usage, troubleshooting.Proposal Text: Include provided overview in README.Implement this specification to ensure a robust, book-centric RAG chatbot. Save to .specify/memory/specification.md.

## Response snapshot

Successfully created the specification file at `.specify/memory/specification.md`.

## Outcome

- ✅ Impact: TBD
- 🧪 Tests: TBD
- 📁 Files: TBD
- 🔁 Next prompts: TBD
- 🧠 Reflection: TBD

## Evaluation notes (flywheel)

- Failure modes observed: TBD
- Graders run and results (PASS/FAIL): TBD
- Prompt variant (if applicable): TBD
- Next experiment (smallest change to try): TBD
