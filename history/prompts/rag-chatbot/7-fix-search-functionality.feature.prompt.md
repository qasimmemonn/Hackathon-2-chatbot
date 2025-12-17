---
id: 7
title: Fix Search Functionality
stage: feature
date: 2025-12-17
surface: agent
model: gemini-2.0-flash-exp
feature: rag-chatbot
branch: main
user: user
command: fix-search
labels: [frontend, search, docusaurus, build]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - website/package.json
tests: []
---

# Prompt

Perfect, but the search functionality is not working. When we try to search, the topics or chapters do not appear, and the error shows:

“The search index is only available when you run docusaurus build”

Fix this so that when a user searches for topics or chapters, the results are displayed correctly.

# Response

I have addressed the search functionality issue.

### Root Cause Analysis
The error *"The search index is only available when you run docusaurus build"* is the expected behavior for the `@easyops-cn/docusaurus-search-local` plugin. This plugin generates a static search index during the **production build process (`npm run build`)** and does not support on-the-fly indexing in the development server (`npm start`).

### Resolution
1.  **Built Project**: I executed `npm run build` in the `website` directory to successfully generate the static site and the search index (files located in `build/`).
2.  **Served Production Build**: I ran `npm run serve`, which serves the built artifacts.

### Result
The search functionality is now fully operational at **`http://localhost:3002/`** (since port 3000 is occupied). You can verify this by visiting that URL; typing in the search bar will now produce instant results for topics and chapters.

### Recommendation
For future use, if you need to test search, always run:
```bash
cd website
npm run build
npm run serve
```
For regular content editing, `npm start` is fine, but search availability will be limited.
