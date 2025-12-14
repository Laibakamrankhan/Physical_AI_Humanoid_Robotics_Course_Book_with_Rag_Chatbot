import os
import asyncio
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the existing RAG agent functionality
from agent import RAGAgent

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG Agent with document retrieval and question answering",
    version="1.0.0"
)

# Add CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class QueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    document_context: Optional[Dict] = None

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    document_context: Optional[Dict] = None

class MatchedChunk(BaseModel):
    content: str
    url: str
    position: int
    similarity_score: float

class QueryResponse(BaseModel):
    answer: str
    sources: List[str]
    matched_chunks: List[MatchedChunk]
    error: Optional[str] = None
    status: str  # "success", "error", "empty"
    query_time_ms: Optional[float] = None
    confidence: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[Dict]  # Changed to match frontend expectations
    session_id: Optional[str] = None
    error: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    message: str

# Global RAG agent instance
rag_agent = None

@app.on_event("startup")
async def startup_event():
    """Initialize the RAG agent on startup"""
    global rag_agent
    logger.info("Initializing RAG Agent...")
    try:
        rag_agent = RAGAgent()
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise

@app.post("/ask", response_model=QueryResponse)
async def ask_rag(request: QueryRequest):
    """
    Process a user query through the RAG agent and return the response
    """
    logger.info(f"Processing query: {request.query[:50]}...")

    try:
        # Validate input
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(request.query) > 2000:
            raise HTTPException(status_code=400, detail="Query too long, maximum 2000 characters")

        # Process query through RAG agent
        response = rag_agent.query_agent(request.query)

        # Format response
        formatted_response = QueryResponse(
            answer=response.get("answer", ""),
            sources=response.get("sources", []),
            matched_chunks=[
                MatchedChunk(
                    content=chunk.get("content", ""),
                    url=chunk.get("url", ""),
                    position=chunk.get("position", 0),
                    similarity_score=chunk.get("similarity_score", 0.0)
                )
                for chunk in response.get("matched_chunks", [])
            ],
            error=response.get("error"),
            status="error" if response.get("error") else "success",
            query_time_ms=response.get("query_time_ms"),
            confidence=response.get("confidence")
        )

        logger.info(f"Query processed successfully in {response.get('query_time_ms', 0):.2f}ms")
        return formatted_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return QueryResponse(
            answer="",
            sources=[],
            matched_chunks=[],
            error=f"Internal server error: {str(e)}",
            status="error"
        )


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that matches the frontend API expectations
    """
    logger.info(f"Processing chat message: {request.message[:50]}...")

    try:
        # Validate input
        if not request.message or len(request.message.strip()) == 0:
            raise HTTPException(status_code=400, detail="Message cannot be empty")

        if len(request.message) > 2000:
            raise HTTPException(status_code=400, detail="Message too long, maximum 2000 characters")

        # Process query through RAG agent
        response = rag_agent.query_agent(request.message)

        # Format sources to match frontend expectations
        formatted_sources = []
        if response.get("matched_chunks"):
            for chunk in response["matched_chunks"]:
                formatted_sources.append({
                    "url": chunk.get("url", ""),
                    "title": chunk.get("url", "Source"),  # Use URL as title if no title available
                    "snippet": chunk.get("content", "")[:200] + "..." if len(chunk.get("content", "")) > 200 else chunk.get("content", "")
                })

        # Create chat response
        chat_response = ChatResponse(
            response=response.get("answer", "Sorry, I couldn't generate a response."),
            sources=formatted_sources,
            session_id=request.session_id,  # In a real implementation, you'd manage sessions
            error=response.get("error")
        )

        logger.info(f"Chat message processed successfully")
        return chat_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat message: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return ChatResponse(
            response="Sorry, I encountered an error processing your request.",
            sources=[],
            error=f"Internal server error: {str(e)}"
        )

class IngestRequest(BaseModel):
    force_reindex: bool = False
    document_path: str = "1-physical-ai-robotics/"

class IngestResponse(BaseModel):
    status: str
    message: str
    documents_processed: int = 0


@app.post("/ingest", response_model=IngestResponse)
async def ingest_documents(request: IngestRequest):
    """
    Ingest documents endpoint (placeholder - would connect to actual ingestion pipeline)
    """
    logger.info(f"Starting document ingestion with path: {request.document_path}, force_reindex: {request.force_reindex}")

    try:
        # In a real implementation, this would run the actual ingestion pipeline
        # For now, we'll simulate the process
        import time
        time.sleep(1)  # Simulate some processing time

        # Placeholder response - in real implementation this would call the ingestion pipeline
        return IngestResponse(
            status="success",
            message=f"Successfully processed documents from {request.document_path}",
            documents_processed=5  # Placeholder number
        )
    except Exception as e:
        logger.error(f"Error during document ingestion: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return IngestResponse(
            status="error",
            message=f"Error during ingestion: {str(e)}",
            documents_processed=0
        )


@app.post("/chat/stream")
async def chat_stream_endpoint(request: ChatRequest):
    """
    Streaming chat endpoint (placeholder - would implement actual streaming)
    """
    from fastapi.responses import StreamingResponse
    import json

    async def event_generator():
        try:
            # Process query through RAG agent
            response = rag_agent.query_agent(request.message)

            # Format sources to match frontend expectations
            formatted_sources = []
            if response.get("matched_chunks"):
                for chunk in response["matched_chunks"]:
                    formatted_sources.append({
                        "url": chunk.get("url", ""),
                        "title": chunk.get("url", "Source"),  # Use URL as title if no title available
                        "snippet": chunk.get("content", "")[:200] + "..." if len(chunk.get("content", "")) > 200 else chunk.get("content", "")
                    })

            # Simulate streaming by sending tokens one by one
            answer = response.get("answer", "Sorry, I couldn't generate a response.")
            tokens = answer.split()

            for i, token in enumerate(tokens):
                yield f"data: {json.dumps({'type': 'token', 'token': token + (' ' if i < len(tokens) - 1 else '')})}\n\n"

            # Send sources at the end
            if formatted_sources:
                yield f"data: {json.dumps({'type': 'sources', 'data': formatted_sources})}\n\n"

            # End the stream
            yield f"data: [DONE]\n\n"

        except Exception as e:
            import traceback
            logger.error(f"Error in streaming response: {e}")
            logger.error(f"Full traceback: {traceback.format_exc()}")
            yield f"data: {json.dumps({'type': 'error', 'message': str(e)})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/plain")


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    return HealthResponse(
        status="healthy",
        message="RAG Agent API is running"
    )

# For running with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)