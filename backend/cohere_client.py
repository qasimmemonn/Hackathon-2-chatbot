import os
import cohere
from dotenv import load_dotenv
from typing import List


load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
co = cohere.Client(COHERE_API_KEY)

def get_embeddings(texts: List[str], model="embed-english-v3.0", input_type="search_document"):
    response = co.embed(
        texts=texts,
        model=model,
        input_type=input_type
    )
    return response.embeddings

def generate_answer(prompt: str):
    response = co.chat(
        model="command-r-plus-08-2024",
        message=prompt,
        max_tokens=500,
        temperature=0.1,
    )
    
    answer = response.text.strip()
    return answer
