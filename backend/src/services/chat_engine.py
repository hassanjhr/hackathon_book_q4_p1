"""
Chat engine service using OpenAI Agents SDK with tool calling for RAG.
"""
import json
from typing import List, Dict, Any, Optional
from openai import OpenAI
from src.config import settings
from src.tools.search_chunks import search_chunks, SEARCH_CHUNKS_TOOL
from src.tools.get_metadata import get_metadata, GET_METADATA_TOOL
from src.models.query import Citation
from src.utils.logger import get_logger

logger = get_logger("chat_engine")


class ChatEngine:
    """
    Chat engine that uses OpenAI Agents with tool calling for RAG queries.
    """

    def __init__(self):
        """Initialize OpenAI client and register tools."""
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_chat_model

        # Available tools for the agent
        self.tools = [
            SEARCH_CHUNKS_TOOL,
            GET_METADATA_TOOL
        ]

        # System prompt for the RAG agent
        self.system_prompt = """You are a helpful AI assistant that answers questions about the Physical AI & Humanoid Robotics textbook.

Your task is to:
1. Use the search_chunks tool to find relevant information from the book
2. Use the get_metadata tool to get detailed information and citations
3. Provide accurate, well-cited answers based on the book content
4. Always include citations with chapter and page references
5. If information is not in the book, clearly state that

Important guidelines:
- Only use information from the book chunks retrieved via tools
- Always cite your sources with chunk IDs and location information
- Do not make up information that isn't in the provided chunks
- If the answer requires information from multiple chunks, synthesize them coherently
- Format citations as [Chapter X, Page Y]"""

    async def query(
        self,
        query_text: str,
        selected_chunk_ids: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """
        Process a user query using the OpenAI Agent with tool calling.

        Args:
            query_text: User's question
            selected_chunk_ids: Optional list of chunk IDs for selected-text mode

        Returns:
            Dictionary with answer_text and citations
        """
        try:
            logger.info(f"Processing query: '{query_text[:100]}...'")

            # Prepare messages
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": query_text}
            ]

            # Make initial API call with tools
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=self.tools,
                tool_choice="auto"
            )

            # Process tool calls and responses
            citations_data = []
            max_iterations = 5
            iteration = 0

            while iteration < max_iterations:
                message = response.choices[0].message

                # If no tool calls, we have the final answer
                if not message.tool_calls:
                    break

                # Add assistant message to conversation
                messages.append(message)

                # Execute each tool call
                for tool_call in message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    logger.info(f"Executing tool: {function_name}")

                    # Execute the appropriate tool
                    if function_name == "search_chunks":
                        # Add selected_chunk_ids if in selected-text mode
                        if selected_chunk_ids:
                            function_args["selected_chunk_ids"] = selected_chunk_ids

                        result = await search_chunks(**function_args)

                        # Store citation information
                        for chunk in result:
                            citations_data.append({
                                "chunk_id": chunk["chunk_id"],
                                "text": chunk["text"],
                                "score": chunk["relevance_score"]
                            })

                    elif function_name == "get_metadata":
                        result = await get_metadata(**function_args)

                        # Update citation with metadata
                        if result:
                            for cit in citations_data:
                                if cit["chunk_id"] == function_args["chunk_id"]:
                                    cit["location"] = result.get("location")
                                    cit["full_text"] = result.get("full_text")

                    else:
                        result = {"error": f"Unknown function: {function_name}"}

                    # Add tool response to messages
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": json.dumps(result)
                    })

                # Get next response from the model
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    tools=self.tools,
                    tool_choice="auto"
                )

                iteration += 1

            # Extract final answer
            answer_text = response.choices[0].message.content

            # Build citations list
            citations = []
            for cit in citations_data[:5]:  # Top 5 citations
                citations.append(Citation(
                    chunk_id=cit["chunk_id"],
                    excerpt=cit["text"][:200],  # First 200 chars
                    chapter_page_ref=cit.get("location", "Unknown"),
                    relevance_score=cit["score"]
                ))

            logger.info(f"Query completed with {len(citations)} citations")

            return {
                "answer_text": answer_text,
                "citations": citations,
                "mode": "selected-text" if selected_chunk_ids else "full-book"
            }

        except Exception as e:
            logger.error(f"Error in chat engine: {e}")
            raise


# Global chat engine instance
chat_engine = ChatEngine()
