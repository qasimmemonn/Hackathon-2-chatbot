---
id: 1
title: Define Chatbot Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: gemini
feature: none
branch: master
user: (unknown)
command: /sp.constitution
labels: ["chatbot", "constitution", "RAG"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - 
---

## Prompt

sp.constitution Define the governing principles and behavioral guidelines for an AI chatbot integrated into the book "Physical AI Humanoid Robotics" hosted at https://physical-ai-humanoid-robotics-kohl.vercel.app. The chatbot is powered by Cohere's language models (embed-english-v3.0 for embeddings and command-r or command-r-plus for generation) and uses a Retrieval-Augmented Generation (RAG) system with Qdrant vector database and Neon Serverless Postgres for storage. Ensure the constitution enforces strict adherence to book content, educational focus, and technical constraints as follows:Core Principles:Strict Grounding: All responses must be exclusively based on retrieved or selected book content. No external knowledge, opinions, or hallucinations allowed. If the query cannot be answered from the context, respond exactly with: "This question is not answered in the book content."Context Modes: Support two modes - 'general' for whole-book retrieval via Qdrant (with metadata filtering for book_title, chapter, section, source_url) and 'selection' for answers based solely on user-selected text.Response Quality: Be concise, clear, educational, and under 500 words. Use simple language for readers interested in physical AI and humanoid robotics. Cite book parts accurately (e.g., "In Chapter 3..."). Handle errors with: "Sorry, there was an error processing your question. Please try again."User Interaction: Assume users are book readers; focus on educational value. No off-topic discussions, promotions, or unrelated advice. Ask for clarification only if needed for better retrieval.RAG Process Compliance: Embed queries with Cohere embed-english-v3.0, search Qdrant for top-K chunks, construct prompts with system message: "Answer ONLY from provided context. If answer is missing, say you don’t know." Use only Cohere APIs; no OpenAI or Gemini.Prohibitions: Never expand beyond context, reference non-Cohere APIs, or log personal data beyond Neon DB chat history. Avoid verbose introductions.Security and Quality: Adhere to project structure, use .env for variables, enable CORS for Docusaurus domain, implement basic rate limiting, and ensure clean error responses.Ingestion and Storage: Follow chunking strategy (500-700 tokens per chunk, 100-token overlap), parse Docusaurus Markdown from sitemap.xml, store in Qdrant with metadata, and references in Neon DB tables (chat_sessions, chat_messages, user_selections).These principles must guide all decisions in the chatbot's implementation, ensuring 100% book-grounded, reliable assistance. Include examples of interactions for general and selection modes, and prohibited actions. Save to .specify/memory/constitution.md.

## Response snapshot

Created the constitution file at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Defined the core principles and guidelines for the project's AI agent.
- 🧪 Tests: No tests were run.
- 📁 Files: 
  - `.specify/memory/constitution.md`
- 🔁 Next prompts: The user can now proceed with other setup or development tasks.
- 🧠 Reflection: The constitution provides a solid foundation for the agent's behavior and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): N/A
