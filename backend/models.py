from pydantic import BaseModel
from typing import List, Optional

class ChatRequest(BaseModel):
    query: str
    chat_session_id: Optional[str] = None

class Document(BaseModel):
    id: str
    title: str
    url: str
    text: str

class ChatResponse(BaseModel):
    answer: str
    chat_session_id: str
    documents: List[Document]
