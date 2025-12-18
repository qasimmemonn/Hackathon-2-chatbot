import os
import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI, Depends, HTTPException, Request, status
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from dotenv import load_dotenv

from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from models import ChatRequest, ChatResponse
from db import create_tables, get_db, ChatSession, ChatMessage
from rag import get_rag_response
from ingest_book import ingest_book_content

load_dotenv()

# -------------------------------------------------
# Lifespan
# -------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    create_tables()
    yield

# -------------------------------------------------
# App Initialization
# -------------------------------------------------
app = FastAPI(lifespan=lifespan)

# -------------------------------------------------
# Rate Limiter
# -------------------------------------------------
limiter = Limiter(key_func=get_remote_address, default_limits=["50/minute"])
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# -------------------------------------------------
# CORS
# -------------------------------------------------

# Vercel frontend URL + optional localhost
origins = [
    "https://physical-ai-book-swart.vercel.app/",  # Vercel frontend URL
    "http://localhost:3000",  # for local testing
]
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------------------------
# Routes
# -------------------------------------------------
@app.post("/ingest")
async def ingest_data():
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, ingest_book_content)
        return {"message": "Ingestion process completed successfully."}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=str(e),
        )

@app.post("/chat", response_model=ChatResponse)
@limiter.limit("5/minute")
async def chat(request: Request, payload: ChatRequest, db: Session = Depends(get_db)):
    """
    Chat endpoint with proper SlowAPI support.
    """
    try:
        chat_session_id = payload.chat_session_id

        # Create or validate session
        if not chat_session_id:
            chat_session = ChatSession()
            db.add(chat_session)
            db.commit()
            db.refresh(chat_session)
            chat_session_id = chat_session.id
        else:
            chat_session = db.query(ChatSession).filter(ChatSession.id == chat_session_id).first()
            if not chat_session:
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chat session not found.")

        # Store user message
        db.add(ChatMessage(chat_session_id=chat_session_id, role="user", content=payload.query))
        db.commit()

        # RAG response
        rag_result = get_rag_response(payload.query)
        answer = rag_result.get("answer", "No answer found in book content.")
        documents = rag_result.get("documents", [])
        
        # If documents were returned under 'sources' key, map them
        if not documents and "sources" in rag_result:
             # Map simple sources to Document format if possible, or just ignore for now if structure is different
             # But ChatResponse expects Document(id, title, url, text). 
             # rag.py sources only have url, title. 
             # So we cannot easily map them without faking id/text.
             pass

        # Store assistant message
        db.add(ChatMessage(chat_session_id=chat_session_id, role="assistant", content=answer))
        db.commit()

        return ChatResponse(answer=answer, chat_session_id=chat_session_id, documents=documents)
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/")
def root():
    return {"message": "Physical AI Humanoid Robotics Chatbot Backend is running."}
