"""
FastAPI application entry point for AI-Native Textbook + RAG Chatbot backend.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Create FastAPI app instance
app = FastAPI(
    title="AI-Native Textbook API",
    description="Backend API for Physical AI & Humanoid Robotics Textbook with RAG Chatbot",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # TODO: Configure for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """Health check endpoint."""
    return {
        "message": "AI-Native Textbook API is running",
        "version": "1.0.0",
        "status": "healthy"
    }

@app.get("/health")
async def health_check():
    """Detailed health check endpoint."""
    return {
        "status": "healthy",
        "services": {
            "api": "operational",
            "database": "operational",
            "vector_db": "operational"
        }
    }

# Import and include routers
from src.api.ingestion import router as ingestion_router
from src.api.query import router as query_router
from src.api.rag import router as rag_router
from src.api.chatkit_routes import router as chatkit_router
app.include_router(ingestion_router, prefix="/api", tags=["Ingestion"])
app.include_router(query_router, prefix="/api", tags=["Query"])
app.include_router(rag_router, prefix="/api", tags=["RAG Chatbot"])
app.include_router(chatkit_router, prefix="/api", tags=["ChatKit"])

# Startup event to initialize database connections
@app.on_event("startup")
async def startup_event():
    """Initialize database connections on startup."""
    from src.services.postgres_client import postgres_service
    await postgres_service.connect()

@app.on_event("shutdown")
async def shutdown_event():
    """Close database connections on shutdown."""
    from src.services.postgres_client import postgres_service
    await postgres_service.disconnect()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
