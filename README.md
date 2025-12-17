# Physical AI Humanoid Robotics - AI-Powered RAG Chatbot

This project implements an AI-powered Retrieval-Augmented Generation (RAG) chatbot integrated into a Docusaurus book website. The chatbot answers questions based *only* on the book's content, utilizing Cohere for embeddings and generation, Qdrant as a vector database, and Neon Serverless Postgres for chat history. The backend is built with FastAPI (Python), and the frontend is a Docusaurus (React) application.

## Project Structure

```
.
├── backend/                  # FastAPI application for RAG logic, database, and ingestion
│   ├── .env                  # Environment variables (local development)
│   ├── main.py               # FastAPI app, endpoints, CORS, rate limiting
│   ├── models.py             # Pydantic schemas for API requests/responses
│   ├── db.py                 # SQLAlchemy setup for Neon Postgres, table definitions
│   ├── cohere_client.py      # Cohere API interaction for embeddings and generation
│   ├── qdrant_client.py      # Qdrant client for vector database operations
│   ├── rag.py                # Core RAG (Retrieval-Augmented Generation) logic
│   ├── ingest_book.py        # Script to crawl sitemap, chunk content, embed, and store
│   └── requirements.txt      # Python dependencies
├── website/                  # Docusaurus frontend application
│   ├── src/components/       # React components (ChatWidget, AskAIButton)
│   ├── src/theme/Layout/     # Swizzled Docusaurus Layout component for integration
│   └── vercel.json           # Vercel deployment configuration for frontend
├── vercel.json               # Vercel configuration for the monorepo (routes to sub-projects)
└── README.md                 # Project README
```

## Features

-   **Contextual Chatbot:** Answers questions based on the "Physical AI Humanoid Robotics" book content.
-   **Strict Grounding:** Responds with "This question is not answered in the book content." if information is not found in the book.
-   **Text Selection Integration:** "Ask AI" button appears on text selection to query the chatbot with the selected text.
-   **Scalable Backend:** FastAPI with Cohere, Qdrant, and Neon Postgres.
-   **Interactive Frontend:** Docusaurus website with a floating chat widget.
-   **Automated Deployment:** Vercel configurations for both frontend and backend.

## Setup and Local Development

### Prerequisites

-   Python 3.9+
-   Node.js (LTS) & npm
-   Access to Cohere API, Qdrant Cloud, and Neon Serverless Postgres with corresponding API keys and URLs.
-   A Docusaurus website sitemap URL (e.g., 
```
https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml
```
)

### 1. Clone the repository

```bash
git clone <repository-url>
cd Physical-chatbot-book # Or your project root directory
```

### 2. Backend Setup

Navigate to the `backend` directory and set up Python dependencies and environment variables.

```bash
cd backend
pip install -r requirements.txt
```

Create a `.env` file in the `backend/` directory with your credentials:

```
QDRANT_URL=https://b65817a5-193e-452e-904c-fe2102f6cd4a.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.8dPl0_h87pE4TW5ZhCBzdyUI-Va-dbQJV0C3IyvesI0
POSTGRES_URL=postgresql://neondb_owner:npg_jIxF8La3EViC@ep-shy-credit-a466et01-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
SITEMAP_URL=https://physical-ai-humanoid-robotics-kohl.vercel.app/sitemap.xml
COHERE_API_KEY=YOUR_COHERE_API_KEY
COLLECTION_NAME=chatbot-physical-AI-Book
DOCUSAURUS_DOMAIN=http://localhost:3000
```
**NOTE:** The `COHERE_API_KEY` provided in the prompt's context is `3iKQrTkk1gCbWCdKkS2u0ohL2kvyYGpsHnJ0MBIL`. This should be replaced with your actual Cohere API key.

### 3. Frontend Setup

Navigate to the `website` directory and install Node.js dependencies.

```bash
cd website
npm install
```

### 4. Run Locally

#### Start the Backend

From the project root (not inside the `backend` directory), run:

```bash
cd backend
uvicorn main:app --reload --port 8000
```
The backend API will be available at `http://localhost:8000`.

#### Ingest Book Content

Once the backend is running, trigger the ingestion process. You can do this by making a POST request to the `/ingest` endpoint:

```bash
curl -X POST http://localhost:8000/ingest
```
This will crawl the Docusaurus sitemap, chunk the content, embed it, and store it in Qdrant. This step only needs to be run once or whenever your book content updates.

#### Start the Frontend

From the `website` directory, run:

```bash
cd website
npm start
```
The Docusaurus website will open in your browser, typically at `http://localhost:3000`. The chatbot widget should be visible on the page.

### 5. Using the Chatbot

-   **Chat Widget:** Click the "Open Chat" button (bottom-right) to interact with the chatbot.
-   **Ask AI (Text Selection):** Select any text on the Docusaurus page, and an "Ask AI" button will appear. Clicking it will send the selected text as a query to the chatbot.

## Deployment to Vercel

This project is configured for deployment to Vercel as a monorepo.

### 1. Vercel Project Setup

Create a new Vercel project and link it to your Git repository.

### 2. Configure Environment Variables

For the backend deployment, you **must** configure the following environment variables in your Vercel project settings (for the `backend` project):

-   `QDRANT_URL`
-   `QDRANT_API_KEY`
-   `POSTGRES_URL`
-   `SITEMAP_URL`
-   `COHERE_API_KEY`
-   `COLLECTION_NAME`
-   `DOCUSAURUS_DOMAIN` (Set this to your Vercel frontend domain, e.g., `https://your-docusaurus-site.vercel.app`)

**Note:** For the `DOCUSAURUS_DOMAIN`, ensure it matches the actual deployed URL of your frontend. For CORS, you might need to adjust the `allow_origins` in `backend/main.py` if you have a custom domain.

### 3. Deploy

Vercel should automatically detect the `vercel.json` files in both the `website` and `backend` directories and deploy them as separate projects within your monorepo.

-   The Docusaurus frontend will be deployed as a static site.
-   The FastAPI backend will be deployed as a Serverless Function.

After deployment, remember to manually trigger the `/ingest` endpoint on your deployed backend once to populate Qdrant with your book's content.

## Important Security Considerations

-   **API Keys:** Never hardcode API keys directly into your code. Always use environment variables.
-   **CORS:** In a production environment, restrict CORS `allow_origins` to only your trusted frontend domains.
-   **Ingestion Endpoint:** The `/ingest` endpoint should be protected in a production environment (e.g., with an API key or authentication) to prevent unauthorized content ingestion.

## Troubleshooting

-   **CORS Errors:** Ensure your `DOCUSAURUS_DOMAIN` environment variable is correctly set in both local `.env` and Vercel, and matches the domain from which your frontend is served.
-   **Empty Chatbot Responses:** Verify that the ingestion process (`/ingest` endpoint) was run successfully and that your Qdrant collection contains data. Check backend logs for errors during embedding or Qdrant operations.
-   **"This question is not answered in the book content."**: This indicates the RAG system did not find sufficiently relevant information in the indexed documents. Try re-running ingestion or adjusting chunking/embedding strategies if many relevant documents are expected.
