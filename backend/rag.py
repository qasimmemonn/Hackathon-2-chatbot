from vector_store import search_vectors
from cohere_client import get_embeddings, generate_answer

SYSTEM_PROMPT = (
    "You are an expert AI assistant for the 'Physical AI & Humanoid Robotics' textbook. "
    "Your goal is to help users understand the book's content and answer their questions meaningfully. "
    "Check the 'Context' provided below, which contains excerpts from the book. "
    "If the 'Context' is relevant, use it to answer the 'Question' accurately and cite the book. "
    "If the 'Context' is empty or irrelevant, or if the user asks a general question (e.g., greetings, general definitions not in the book), "
    "answer using your general knowledge about Physical AI and Robotics. "
    "Do NOT simply say 'not answered in the book' unless strictly necessary. Be helpful and conversational."
)


def get_rag_response(query: str) -> dict:
    # 1️⃣ Embed query
    query_embedding = get_embeddings(
        texts=[query],
        input_type="search_query"
    )[0]

    # 2️⃣ Vector search
    results = search_vectors(query_embedding)

    # 3️⃣ Build context
    context = ""
    if results:
        context = "\n\n".join(
            hit["payload"].get("text", "")
            for hit in results
            if hit.get("payload") and "text" in hit["payload"]
        )

    prompt = f"""
{SYSTEM_PROMPT}

Context:
{context}

Question:
{query}
"""

    # 4️⃣ Generate grounded answer
    answer = generate_answer(prompt)

    return {
        "answer": answer,
        "sources": [
            {
                "url": hit["payload"].get("url", ""),
                "title": hit["payload"].get("title", "")
            }
            for hit in results
        ]
    }
