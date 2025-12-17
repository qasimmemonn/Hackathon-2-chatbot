from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, Filter, FieldCondition, MatchValue
from dotenv import load_dotenv
import os

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = os.getenv("COLLECTION_NAME")


def create_collection():
    collections = [c.name for c in client.get_collections().collections]
    if COLLECTION_NAME not in collections:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,
                distance=Distance.COSINE
            )
        )


def upsert_points(points):
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )


def search_vectors(query_embedding, top_k=5, filters: dict = None):
    """
    Search the Qdrant collection using query embedding.
    Optional filters can be passed as a dictionary.
    Returns a list of hits with id, payload, and score.
    """
    qdrant_filter = None
    if filters:
        must_conditions = []
        for key, value in filters.items():
            must_conditions.append(FieldCondition(key=key, match=MatchValue(value=value)))
        qdrant_filter = Filter(must=must_conditions)

    # Updated: use `search` instead of `search_points`
    # Updated: use `query_points` instead of `search` since `search` is missing
    response = client.query_points(
        collection_name=COLLECTION_NAME,
        query=query_embedding,
        limit=top_k,
        query_filter=qdrant_filter
    )
    hits = response.points

    return [{"id": hit.id, "payload": hit.payload, "score": hit.score} for hit in hits]
