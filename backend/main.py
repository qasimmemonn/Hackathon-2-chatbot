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
# CORS (FIXED)
# -------------------------------------------------
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://physical-ai-book-swart.vercel.app",
        "http://localhost:3000",
    ],
    allow_credentials=False,   # IMPORTANT
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
async def chat(
    request: Request,                 # MUST BE FIRST
    payload: ChatRequest,
    db: Session = Depends(get_db)
):
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
                raise HTTPException(status_code=404, detail="Chat session not found.")

        # Store user message
        db.add(ChatMessage(
            chat_session_id=chat_session_id,
            role="user",
            content=payload.query
        ))
        db.commit()

        # RAG response
        rag_result = get_rag_response(payload.query)
        answer = rag_result.get("answer", "No answer found in book content.")
        documents = rag_result.get("documents", [])

        # Store assistant message
        db.add(ChatMessage(
            chat_session_id=chat_session_id,
            role="assistant",
            content=answer
        ))
        db.commit()

        return ChatResponse(
            answer=answer,
            chat_session_id=chat_session_id,
            documents=documents
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/")
def root():
    return {"message": "Physical AI Humanoid Robotics Chatbot Backend is running."}
