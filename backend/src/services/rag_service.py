"""
Retrieval-Augmented Generation (RAG) service for chatbot.

Handles query embedding, vector search, context retrieval, and answer generation.
Enhanced with Context7 library documentation support.
"""
import asyncio
import json
from typing import List, Dict, Any, Optional
from openai import OpenAI
from tenacity import retry, stop_after_attempt, wait_exponential

from src.services.embedder import embedder_service
from src.services.qdrant_client import qdrant_service
from src.services.postgres_client import postgres_service
from src.services.context7_service import context7_service
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("rag_service")


class RAGService:
    """Service for RAG-based question answering."""

    def __init__(self):
        """Initialize RAG service."""
        self.openai_client = OpenAI(api_key=settings.openai_api_key)
        self.chat_model = settings.openai_chat_model
        self.top_k = settings.rag_top_k
        self.final_k = settings.rag_final_k

    async def answer_question(
        self,
        question: str,
        selected_text: Optional[str] = None,
        top_k: Optional[int] = None,
        include_sources: bool = True,
        use_library_docs: bool = True
    ) -> Dict[str, Any]:
        """
        Answer a question using RAG, optionally enhanced with Context7 library docs.

        Args:
            question: User's question
            selected_text: Optional text selected by user (for targeted search)
            top_k: Number of chunks to retrieve (defaults to config)
            include_sources: Whether to include source citations
            use_library_docs: Whether to enhance with Context7 library documentation

        Returns:
            Dictionary with 'answer' and optionally 'sources'
        """
        logger.info(f"Processing question: {question}")

        # Use selected text if provided
        if selected_text:
            logger.info("Using selected text for targeted search")
            return await self._answer_from_selected_text(question, selected_text)

        # Route to hybrid or standard RAG
        if use_library_docs:
            return await self._answer_from_corpus_with_libraries(
                question, top_k, include_sources
            )
        else:
            return await self._answer_from_corpus(question, top_k, include_sources)

    async def _answer_from_selected_text(
        self,
        question: str,
        selected_text: str
    ) -> Dict[str, Any]:
        """
        Answer question based only on selected text.

        Args:
            question: User's question
            selected_text: Text selected by user

        Returns:
            Answer dictionary
        """
        logger.info("Answering from selected text")

        # Construct prompt with selected text only
        system_prompt = """You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook.
Answer the user's question based ONLY on the provided text excerpt.
If the answer cannot be found in the excerpt, say "I cannot answer this question based on the selected text."
Be concise and accurate."""

        user_prompt = f"""Text excerpt:
{selected_text}

Question: {question}

Answer:"""

        try:
            response = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            answer = response.choices[0].message.content.strip()

            return {
                "answer": answer,
                "sources": [{
                    "type": "selected_text",
                    "text_preview": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text
                }]
            }

        except Exception as e:
            logger.error(f"Error generating answer from selected text: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your question.",
                "sources": [],
                "error": str(e)
            }

    async def _answer_from_corpus(
        self,
        question: str,
        top_k: Optional[int] = None,
        include_sources: bool = True
    ) -> Dict[str, Any]:
        """
        Answer question by searching the full corpus.

        Args:
            question: User's question
            top_k: Number of chunks to retrieve
            include_sources: Whether to include sources

        Returns:
            Answer dictionary with sources
        """
        k = top_k or self.top_k

        # Step 1: Generate embedding for the question
        logger.info("Generating query embedding...")
        try:
            query_embedding = embedder_service.embed_text(question)
        except Exception as e:
            logger.error(f"Error generating query embedding: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your question.",
                "sources": [],
                "error": str(e)
            }

        # Step 2: Search Qdrant for relevant chunks
        logger.info(f"Searching for top {k} relevant chunks...")
        search_results = qdrant_service.search_vectors(
            query_vector=query_embedding,
            limit=k,
            score_threshold=0.3  # Minimum similarity score
        )

        if not search_results:
            logger.warning("No relevant chunks found")
            return {
                "answer": "I couldn't find relevant information in the textbook to answer your question.",
                "sources": []
            }

        logger.info(f"Found {len(search_results)} relevant chunks")

        # Step 3: Get full chunk data from Postgres
        chunk_contexts = []
        sources = []

        for result in search_results[:self.final_k]:
            payload = result['payload']

            # Get full text from Postgres if needed
            chunk_id = payload.get('chunk_id')
            chunk_data = await postgres_service.get_chunk(chunk_id)

            if chunk_data:
                chunk_contexts.append({
                    'text': chunk_data['text'],
                    'module': payload.get('module', 'Unknown'),
                    'week': payload.get('week', 'Unknown'),
                    'file': payload.get('file_path', '').split('/')[-1],
                    'score': result['score']
                })

                if include_sources:
                    sources.append({
                        'module': payload.get('module', 'Unknown'),
                        'week': payload.get('week', 'Unknown'),
                        'file': payload.get('file_path', '').split('/')[-1],
                        'relevance_score': round(result['score'], 3)
                    })

        if not chunk_contexts:
            logger.warning("No chunk data retrieved from Postgres")
            return {
                "answer": "I couldn't retrieve the relevant information to answer your question.",
                "sources": []
            }

        # Step 4: Construct prompt with retrieved context
        context_text = "\n\n---\n\n".join([
            f"[Source: {ctx['module']}, {ctx['week']}]\n{ctx['text']}"
            for ctx in chunk_contexts
        ])

        system_prompt = """You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook.
Answer the user's question based ONLY on the provided context from the textbook.
If the answer cannot be found in the context, clearly state: "I cannot find this information in the textbook."
Be accurate, concise, and cite which module/week the information comes from when relevant.
Do not make up information or use knowledge outside the provided context."""

        user_prompt = f"""Context from textbook:

{context_text}

Question: {question}

Answer (based only on the context above):"""

        # Step 5: Generate answer using OpenAI
        logger.info("Generating answer with LLM...")
        try:
            response = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=800
            )

            answer = response.choices[0].message.content.strip()

            logger.info("Answer generated successfully")

            return {
                "answer": answer,
                "sources": sources if include_sources else []
            }

        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            return {
                "answer": "Sorry, I encountered an error generating the answer.",
                "sources": sources if include_sources else [],
                "error": str(e)
            }

    async def _detect_library_intent(self, question: str) -> Dict[str, Any]:
        """
        Detect if question involves a library using GPT-3.5-turbo.

        Args:
            question: User's question

        Returns:
            {
                "is_library_question": bool,
                "library_name": str | None,
                "mode": "code" | "info",
                "confidence": float
            }
        """
        try:
            prompt = f"""Analyze if this question requires library documentation.

Question: {question}

Supported libraries: pytorch, isaac-sim, ros2, tensorflow, opencv

Respond in JSON:
{{
  "is_library_question": true/false,
  "library_name": "pytorch" | null,
  "mode": "code" | "info",
  "confidence": 0.0-1.0
}}

Use "code" mode for API/implementation questions, "info" mode for concepts."""

            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.0,
                max_tokens=100,
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)
            logger.info(f"Library detection: {result}")
            return result

        except Exception as e:
            logger.error(f"Error in library detection: {e}")
            return {
                "is_library_question": False,
                "library_name": None,
                "mode": "code",
                "confidence": 0.0
            }

    async def _answer_from_corpus_with_libraries(
        self,
        question: str,
        top_k: Optional[int],
        include_sources: bool
    ) -> Dict[str, Any]:
        """
        Hybrid RAG: textbook + Context7 library docs.

        Args:
            question: User's question
            top_k: Number of chunks to retrieve
            include_sources: Whether to include sources

        Returns:
            Answer dictionary with sources from both textbook and libraries
        """
        k = top_k or self.top_k

        # Step 1: Parallel detection + embedding
        logger.info("Starting hybrid RAG with Context7...")

        detection_task = self._detect_library_intent(question)
        embedding_task = asyncio.to_thread(embedder_service.embed_text, question)

        try:
            library_intent, query_embedding = await asyncio.gather(
                detection_task,
                embedding_task,
                return_exceptions=True
            )

            # Handle exceptions in parallel tasks
            if isinstance(library_intent, Exception):
                logger.error(f"Library detection failed: {library_intent}")
                library_intent = {"is_library_question": False, "confidence": 0.0}

            if isinstance(query_embedding, Exception):
                logger.error(f"Embedding generation failed: {query_embedding}")
                return {
                    "answer": "Sorry, I encountered an error processing your question.",
                    "sources": [],
                    "error": str(query_embedding)
                }

        except Exception as e:
            logger.error(f"Error in parallel tasks: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your question.",
                "sources": [],
                "error": str(e)
            }

        # Step 2: Qdrant search (always run)
        logger.info(f"Searching for top {k} relevant chunks...")
        search_results = qdrant_service.search_vectors(
            query_vector=query_embedding,
            limit=k,
            score_threshold=0.3
        )

        # Get textbook contexts (existing logic)
        textbook_contexts = []
        for result in search_results[:self.final_k]:
            payload = result['payload']
            chunk_id = payload.get('chunk_id')
            chunk_data = await postgres_service.get_chunk(chunk_id)

            if chunk_data:
                textbook_contexts.append({
                    'text': chunk_data['text'],
                    'module': payload.get('module', 'Unknown'),
                    'week': payload.get('week', 'Unknown'),
                    'file': payload.get('file_path', '').split('/')[-1],
                    'score': result['score']
                })

        # Step 3: Context7 search (if library detected)
        library_contexts = []
        library_sources = []

        if (library_intent
            and library_intent.get("is_library_question")
            and library_intent.get("confidence", 0) >= 0.7):

            logger.info(f"Library detected: {library_intent['library_name']}, fetching docs...")

            try:
                library_result = await asyncio.wait_for(
                    context7_service.search_library_docs(
                        library_name=library_intent["library_name"],
                        query=question,
                        mode=library_intent["mode"]
                    ),
                    timeout=3.0
                )

                if library_result and library_result.get("content"):
                    library_contexts.append({
                        "text": library_result["content"],
                        "library": library_intent["library_name"],
                        "type": "library_docs"
                    })

                    library_sources.append({
                        "type": "library_docs",
                        "library": library_intent["library_name"],
                        "urls": library_result.get("sources", [])
                    })
                    logger.info(f"Successfully fetched library docs for {library_intent['library_name']}")

            except asyncio.TimeoutError:
                logger.warning("Context7 timeout, using textbook-only")
            except Exception as e:
                logger.error(f"Context7 error: {e}")

        # Step 4: Merge and generate answer
        merged_context = self._merge_contexts(textbook_contexts, library_contexts)
        answer = await self._generate_multisource_answer(
            question=question,
            context=merged_context,
            has_library_docs=bool(library_contexts)
        )

        # Step 5: Build unified sources
        all_sources = []
        if include_sources:
            # Limit textbook sources if library docs present
            textbook_limit = 2 if library_contexts else 5
            all_sources.extend([{
                "type": "textbook",
                "module": ctx["module"],
                "week": ctx["week"],
                "file": ctx["file"],
                "relevance_score": ctx["score"]
            } for ctx in textbook_contexts[:textbook_limit]])

            all_sources.extend(library_sources)

        return {
            "answer": answer,
            "sources": all_sources
        }

    def _merge_contexts(
        self,
        textbook_contexts: List[Dict],
        library_contexts: List[Dict]
    ) -> str:
        """
        Merge textbook + library contexts with clear separation.

        Args:
            textbook_contexts: List of textbook chunk contexts
            library_contexts: List of library documentation contexts

        Returns:
            Merged context string
        """
        sections = []

        if textbook_contexts:
            limit = 2 if library_contexts else 5
            textbook_text = "\n\n---\n\n".join([
                f"[Source: {ctx['module']}, {ctx['week']}]\n{ctx['text']}"
                for ctx in textbook_contexts[:limit]
            ])
            sections.append(f"[TEXTBOOK CONTENT]\n\n{textbook_text}")

        if library_contexts:
            library_text = "\n\n---\n\n".join([
                f"[Library: {ctx['library']}]\n{ctx['text']}"
                for ctx in library_contexts
            ])
            sections.append(f"[LIBRARY DOCUMENTATION]\n\n{library_text}")

        return "\n\n===\n\n".join(sections)

    async def _generate_multisource_answer(
        self,
        question: str,
        context: str,
        has_library_docs: bool
    ) -> str:
        """
        Generate answer with multi-source prompt.

        Args:
            question: User's question
            context: Merged context from textbook and/or library docs
            has_library_docs: Whether library documentation is included

        Returns:
            Generated answer
        """
        if has_library_docs:
            system_prompt = """You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook with access to library documentation.

Answer using:
1. TEXTBOOK CONTENT - Course concepts and explanations
2. LIBRARY DOCUMENTATION - Up-to-date API references and code examples

Instructions:
- Integrate information from both sources seamlessly
- Prioritize textbook for concepts, library docs for implementation details
- Cite sources clearly: "According to the textbook..." or "The {library} API shows..."
- If information conflicts, prefer library docs for API specifics
- Be accurate, concise, and practical
- Do NOT make up information outside provided context"""
        else:
            system_prompt = """You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook.
Answer based ONLY on the provided textbook context.
If answer not found, state: "I cannot find this information in the textbook."
Be accurate, concise, cite module/week when relevant."""

        user_prompt = f"""Context:

{context}

Question: {question}

Answer (based on the context above):"""

        try:
            response = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=800
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating multisource answer: {e}")
            return "Sorry, I encountered an error generating the answer."

    async def health_check(self) -> Dict[str, str]:
        """
        Check if RAG service is operational.

        Returns:
            Status dictionary
        """
        try:
            # Test embedding generation
            test_embedding = embedder_service.embed_text("test")

            # Test Qdrant connection
            test_results = qdrant_service.search_vectors(
                query_vector=test_embedding,
                limit=1
            )

            return {
                "status": "healthy",
                "embedder": "operational",
                "vector_db": "operational",
                "database": "operational"
            }
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }


# Global RAG service instance
rag_service = RAGService()
