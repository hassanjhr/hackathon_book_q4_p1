"""
RAG Chatbot Agent - Standalone Reusable Component

This is a portable RAG (Retrieval-Augmented Generation) chatbot agent that can be:
- Used as a CLI tool
- Imported as a Python module
- Integrated into Discord/Slack/Telegram bots
- Used in Jupyter notebooks
- Deployed as a standalone API

Features:
- Vector search in Qdrant for textbook content
- Context7 integration for live library documentation (PyTorch, ROS2, Isaac Sim)
- OpenAI GPT-4 for natural language responses
- Source citations
- Conversation history management

Author: AI-Native Learning
License: MIT
"""

import os
import sys
import json
from typing import List, Dict, Optional, Any
from dataclasses import dataclass, asdict
import asyncio

# Core dependencies
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue


@dataclass
class Message:
    """Chat message structure"""
    role: str  # 'user' or 'assistant'
    content: str
    sources: Optional[List[Dict[str, Any]]] = None


@dataclass
class RAGConfig:
    """Configuration for RAG agent"""
    # API Keys
    openai_api_key: str
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    context7_api_key: Optional[str] = None

    # Qdrant settings
    collection_name: str = "textbook_chunks"
    embedding_dim: int = 1536

    # OpenAI settings
    model: str = "gpt-4o"
    temperature: float = 0.7
    max_tokens: int = 1000

    # RAG settings
    top_k: int = 5
    relevance_threshold: float = 0.7
    use_context7: bool = True

    @classmethod
    def from_env(cls):
        """Load configuration from environment variables"""
        return cls(
            openai_api_key=os.getenv("OPENAI_API_KEY", ""),
            qdrant_url=os.getenv("QDRANT_URL", "http://localhost:6333"),
            qdrant_api_key=os.getenv("QDRANT_API_KEY"),
            context7_api_key=os.getenv("CONTEXT7_API_KEY"),
        )


class RAGAgent:
    """
    Standalone RAG Chatbot Agent

    Usage:
        # As a module
        from rag_agent import RAGAgent, RAGConfig

        config = RAGConfig.from_env()
        agent = RAGAgent(config)

        response = await agent.ask("What is Physical AI?")
        print(response.content)
        print(response.sources)
    """

    def __init__(self, config: RAGConfig):
        """Initialize RAG agent with configuration"""
        self.config = config
        self.conversation_history: List[Message] = []

        # Initialize clients
        self.openai_client = OpenAI(api_key=config.openai_api_key)
        self.qdrant_client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key
        )

        # Verify collection exists
        self._verify_collection()

    def _verify_collection(self):
        """Verify Qdrant collection exists"""
        try:
            collections = self.qdrant_client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.config.collection_name not in collection_names:
                print(f"⚠️  Warning: Collection '{self.config.collection_name}' not found in Qdrant")
                print(f"Available collections: {collection_names}")
                print("Creating collection...")
                self._create_collection()
        except Exception as e:
            print(f"❌ Error connecting to Qdrant: {e}")
            sys.exit(1)

    def _create_collection(self):
        """Create Qdrant collection if it doesn't exist"""
        self.qdrant_client.create_collection(
            collection_name=self.config.collection_name,
            vectors_config=VectorParams(
                size=self.config.embedding_dim,
                distance=Distance.COSINE
            )
        )
        print(f"✅ Created collection: {self.config.collection_name}")

    def _embed_text(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI"""
        response = self.openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding

    def _search_textbook(self, query: str) -> List[Dict[str, Any]]:
        """Search textbook chunks in Qdrant"""
        try:
            query_vector = self._embed_text(query)

            search_results = self.qdrant_client.search(
                collection_name=self.config.collection_name,
                query_vector=query_vector,
                limit=self.config.top_k,
                score_threshold=self.config.relevance_threshold
            )

            sources = []
            for result in search_results:
                sources.append({
                    "type": "textbook",
                    "text": result.payload.get("text", ""),
                    "module": result.payload.get("module", "Unknown"),
                    "week": result.payload.get("week", "Unknown"),
                    "file": result.payload.get("file_path", ""),
                    "relevance_score": result.score
                })

            return sources
        except Exception as e:
            print(f"⚠️  Textbook search error: {e}")
            return []

    def _fetch_context7_docs(self, query: str) -> List[Dict[str, Any]]:
        """Fetch relevant library documentation from Context7"""
        if not self.config.use_context7:
            return []

        # Detect libraries mentioned in query
        libraries = []
        query_lower = query.lower()

        if "pytorch" in query_lower or "torch" in query_lower or "tensor" in query_lower:
            libraries.append("pytorch")
        if "ros" in query_lower or "ros2" in query_lower:
            libraries.append("ros2")
        if "isaac" in query_lower or "simulation" in query_lower:
            libraries.append("isaac-sim")

        if not libraries:
            return []

        # Note: Context7 integration would go here
        # For now, return placeholder
        return [
            {
                "type": "library_docs",
                "library": lib,
                "text": f"Relevant {lib.upper()} documentation"
            }
            for lib in libraries
        ]

    def _build_context(self, textbook_sources: List[Dict], library_sources: List[Dict]) -> str:
        """Build context string from sources"""
        context_parts = []

        if textbook_sources:
            context_parts.append("### Textbook Content:")
            for i, source in enumerate(textbook_sources[:3], 1):
                context_parts.append(f"\n[Source {i}] {source['module']}, {source['week']}:")
                context_parts.append(source['text'][:500])

        if library_sources:
            context_parts.append("\n### Library Documentation:")
            for source in library_sources:
                context_parts.append(f"\n[{source['library'].upper()}]:")
                context_parts.append(source['text'][:300])

        return "\n".join(context_parts)

    async def ask(self, question: str, include_history: bool = True) -> Message:
        """
        Ask a question to the RAG agent

        Args:
            question: User question
            include_history: Include conversation history in context

        Returns:
            Message object with response and sources
        """
        # Search for relevant content
        textbook_sources = self._search_textbook(question)
        library_sources = self._fetch_context7_docs(question)

        # Build context
        context = self._build_context(textbook_sources, library_sources)

        # Build conversation messages
        messages = []

        # System prompt
        system_prompt = """You are an AI teaching assistant for a Physical AI and Humanoid Robotics course.

Your role:
- Answer questions about Physical AI, humanoid robotics, reinforcement learning, and related topics
- Use the provided textbook content and library documentation to give accurate answers
- Cite sources when referencing specific content
- Be concise but thorough
- Use code examples when helpful

If you don't know something or the provided context doesn't contain the answer, say so honestly."""

        messages.append({"role": "system", "content": system_prompt})

        # Add conversation history if requested
        if include_history:
            for msg in self.conversation_history[-4:]:  # Last 4 messages
                messages.append({
                    "role": msg.role,
                    "content": msg.content
                })

        # Add current question with context
        user_message = f"""Question: {question}

Relevant Context:
{context}

Please answer the question using the context provided above. Be specific and cite sources."""

        messages.append({"role": "user", "content": user_message})

        # Call OpenAI
        try:
            response = self.openai_client.chat.completions.create(
                model=self.config.model,
                messages=messages,
                temperature=self.config.temperature,
                max_tokens=self.config.max_tokens
            )

            answer = response.choices[0].message.content

            # Create response message
            response_message = Message(
                role="assistant",
                content=answer,
                sources=textbook_sources + library_sources
            )

            # Update conversation history
            self.conversation_history.append(Message(role="user", content=question))
            self.conversation_history.append(response_message)

            return response_message

        except Exception as e:
            print(f"❌ OpenAI API error: {e}")
            return Message(
                role="assistant",
                content="Sorry, I encountered an error generating a response.",
                sources=[]
            )

    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []

    def get_history(self) -> List[Dict]:
        """Get conversation history as JSON-serializable list"""
        return [asdict(msg) for msg in self.conversation_history]

    def export_conversation(self, filepath: str):
        """Export conversation to JSON file"""
        with open(filepath, 'w') as f:
            json.dump(self.get_history(), f, indent=2)
        print(f"✅ Conversation exported to {filepath}")


async def cli_interface():
    """
    Interactive CLI interface for the RAG agent

    Usage:
        python rag_agent.py
    """
    print("=" * 60)
    print("🤖 RAG Chatbot Agent - Physical AI & Humanoid Robotics")
    print("=" * 60)
    print()

    # Load configuration
    config = RAGConfig.from_env()

    if not config.openai_api_key:
        print("❌ Error: OPENAI_API_KEY not set in environment")
        print("Set it with: export OPENAI_API_KEY=your-key-here")
        sys.exit(1)

    print(f"📡 Connecting to Qdrant at {config.qdrant_url}...")
    agent = RAGAgent(config)
    print("✅ Connected!")
    print()
    print("Type your questions below. Commands:")
    print("  /clear    - Clear conversation history")
    print("  /export   - Export conversation to JSON")
    print("  /quit     - Exit")
    print()

    while True:
        try:
            question = input("You: ").strip()

            if not question:
                continue

            # Handle commands
            if question == "/quit":
                print("👋 Goodbye!")
                break
            elif question == "/clear":
                agent.clear_history()
                print("✅ Conversation history cleared")
                continue
            elif question == "/export":
                filename = f"conversation_{int(asyncio.get_event_loop().time())}.json"
                agent.export_conversation(filename)
                continue

            # Get response
            print("🤔 Thinking...", end="\r")
            response = await agent.ask(question)
            print(" " * 50, end="\r")  # Clear "Thinking..."

            # Display response
            print(f"\n🤖 Assistant: {response.content}\n")

            # Display sources
            if response.sources:
                print("📚 Sources:")
                for i, source in enumerate(response.sources[:3], 1):
                    if source['type'] == 'textbook':
                        print(f"  {i}. {source['module']}, {source['week']} (relevance: {source['relevance_score']:.2f})")
                    else:
                        print(f"  {i}. {source['library'].upper()} documentation")
                print()

        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    """
    Run the CLI interface when executed directly

    Usage:
        python rag_agent.py
    """
    asyncio.run(cli_interface())
