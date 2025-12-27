# RAG Agent - Standalone Chatbot Usage Guide

The `rag_agent.py` file is a **portable, reusable RAG chatbot** that you can use anywhere!

## 📦 What You Get

- ✅ **Standalone Python module** - No web framework needed
- ✅ **CLI interface** - Interactive chat in your terminal
- ✅ **Importable class** - Use in any Python project
- ✅ **Discord/Slack/Telegram ready** - Easy bot integration
- ✅ **Jupyter notebook compatible** - Use in data science workflows
- ✅ **Context7 integration** - Live library documentation (PyTorch, ROS2, Isaac Sim)
- ✅ **Conversation history** - Memory across messages
- ✅ **Source citations** - Know where answers come from

---

## 🚀 Quick Start

### 1. Set Environment Variables

```bash
export OPENAI_API_KEY=sk-your-key-here
export QDRANT_URL=http://localhost:6333  # or your Qdrant Cloud URL
export QDRANT_API_KEY=your-qdrant-key    # optional, if using Qdrant Cloud
```

### 2. Run as CLI Tool

```bash
cd backend
python rag_agent.py
```

You'll see:
```
🤖 RAG Chatbot Agent - Physical AI & Humanoid Robotics
===========================================================

📡 Connecting to Qdrant at http://localhost:6333...
✅ Connected!

Type your questions below. Commands:
  /clear    - Clear conversation history
  /export   - Export conversation to JSON
  /quit     - Exit

You: What is Physical AI?
🤔 Thinking...

🤖 Assistant: Physical AI refers to...

📚 Sources:
  1. Module 1, Week 1 (relevance: 0.89)
  2. Module 1, Week 2 (relevance: 0.82)
```

---

## 💻 Use as Python Module

### Basic Usage

```python
import asyncio
from rag_agent import RAGAgent, RAGConfig

async def main():
    # Load config from environment
    config = RAGConfig.from_env()

    # Or create custom config
    config = RAGConfig(
        openai_api_key="sk-...",
        qdrant_url="http://localhost:6333",
        model="gpt-4o",
        temperature=0.7,
        top_k=5
    )

    # Create agent
    agent = RAGAgent(config)

    # Ask questions
    response = await agent.ask("What is Physical AI?")

    print(response.content)  # Answer
    print(response.sources)  # List of sources with citations

asyncio.run(main())
```

### With Conversation History

```python
agent = RAGAgent(config)

# First question
response1 = await agent.ask("What is reinforcement learning?")
print(response1.content)

# Follow-up (agent remembers context)
response2 = await agent.ask("How is it used in robotics?")
print(response2.content)

# Clear history
agent.clear_history()
```

### Export Conversation

```python
# Get history as JSON
history = agent.get_history()
print(history)

# Export to file
agent.export_conversation("my_conversation.json")
```

---

## 🤖 Discord Bot Integration

```python
import discord
from rag_agent import RAGAgent, RAGConfig

client = discord.Client()
rag_config = RAGConfig.from_env()
agent = RAGAgent(rag_config)

@client.event
async def on_message(message):
    if message.author == client.user:
        return

    if message.content.startswith("!ask"):
        question = message.content[5:].strip()

        if not question:
            await message.channel.send("Please ask a question!")
            return

        # Get RAG response
        response = await agent.ask(question)

        # Format response
        embed = discord.Embed(
            title="🤖 AI Assistant",
            description=response.content,
            color=0x00ff00
        )

        # Add sources
        if response.sources:
            sources_text = "\n".join([
                f"• {s['module']}, {s['week']}"
                for s in response.sources[:3]
            ])
            embed.add_field(name="📚 Sources", value=sources_text)

        await message.channel.send(embed=embed)

client.run("YOUR_DISCORD_BOT_TOKEN")
```

---

## 💬 Slack Bot Integration

```python
from slack_bolt.async_app import AsyncApp
from rag_agent import RAGAgent, RAGConfig

app = AsyncApp(token="xoxb-your-token")
agent = RAGAgent(RAGConfig.from_env())

@app.message("ask")
async def handle_ask(message, say):
    question = message['text'].replace("ask", "").strip()

    response = await agent.ask(question)

    # Format sources
    sources_text = "\n".join([
        f"• {s['module']}, {s['week']} (relevance: {s['relevance_score']:.2f})"
        for s in response.sources[:3]
    ])

    await say({
        "blocks": [
            {
                "type": "section",
                "text": {"type": "mrkdwn", "text": f"*Answer:*\n{response.content}"}
            },
            {
                "type": "section",
                "text": {"type": "mrkdwn", "text": f"*Sources:*\n{sources_text}"}
            }
        ]
    })

if __name__ == "__main__":
    app.start(3000)
```

---

## 📓 Jupyter Notebook Usage

```python
# In your notebook
from rag_agent import RAGAgent, RAGConfig

# Create agent
config = RAGConfig(
    openai_api_key="sk-...",
    qdrant_url="http://localhost:6333"
)
agent = RAGAgent(config)

# Ask questions
response = await agent.ask("Explain PyTorch tensors")
display(Markdown(response.content))

# View sources
import pandas as pd
sources_df = pd.DataFrame(response.sources)
sources_df[['module', 'week', 'relevance_score']]
```

---

## 🌐 Add to Your Own Web API

### FastAPI Example

```python
from fastapi import FastAPI
from pydantic import BaseModel
from rag_agent import RAGAgent, RAGConfig

app = FastAPI()
agent = RAGAgent(RAGConfig.from_env())

class Question(BaseModel):
    question: str

@app.post("/ask")
async def ask_question(q: Question):
    response = await agent.ask(q.question)
    return {
        "answer": response.content,
        "sources": response.sources
    }
```

### Flask Example

```python
from flask import Flask, request, jsonify
from rag_agent import RAGAgent, RAGConfig
import asyncio

app = Flask(__name__)
agent = RAGAgent(RAGConfig.from_env())

@app.route("/ask", methods=["POST"])
def ask():
    question = request.json.get("question")

    # Run async in sync context
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    response = loop.run_until_complete(agent.ask(question))
    loop.close()

    return jsonify({
        "answer": response.content,
        "sources": response.sources
    })
```

---

## ⚙️ Configuration Options

### RAGConfig Parameters

```python
config = RAGConfig(
    # Required
    openai_api_key="sk-...",          # Your OpenAI API key
    qdrant_url="http://localhost:6333",  # Qdrant server URL

    # Optional
    qdrant_api_key=None,               # Qdrant Cloud API key
    context7_api_key=None,             # Context7 API key
    collection_name="textbook_chunks", # Qdrant collection name
    embedding_dim=1536,                # Embedding dimension
    model="gpt-4o",                    # OpenAI model
    temperature=0.7,                   # Response creativity (0-1)
    max_tokens=1000,                   # Max response length
    top_k=5,                          # Number of chunks to retrieve
    relevance_threshold=0.7,           # Minimum relevance score
    use_context7=True                  # Enable Context7 integration
)
```

---

## 📊 Response Format

```python
@dataclass
class Message:
    role: str                    # 'user' or 'assistant'
    content: str                 # The answer text
    sources: List[Dict] = None   # List of source citations

# Source dictionary format:
{
    "type": "textbook",          # or "library_docs"
    "text": "...",              # Chunk text
    "module": "Module 1",        # Textbook module
    "week": "Week 1",            # Week number
    "file": "path/to/file.md",   # Source file
    "relevance_score": 0.89      # Similarity score (0-1)
}
```

---

## 🔄 Migration from Existing Backend

If you're already using the FastAPI backend (`backend/src/services/rag_service.py`), you can switch to this agent:

### Before (FastAPI backend):
```python
from src.services.rag_service import rag_service

result = await rag_service.answer_question(
    question="What is Physical AI?",
    use_library_docs=True
)
```

### After (RAG Agent):
```python
from rag_agent import RAGAgent, RAGConfig

agent = RAGAgent(RAGConfig.from_env())
response = await agent.ask("What is Physical AI?")
```

---

## 🎯 Real-World Use Cases

### 1. **Study Assistant CLI Tool**
Students run `python rag_agent.py` to ask questions while studying.

### 2. **Discord Study Server**
Create a Discord bot that answers questions 24/7 in your study server.

### 3. **Jupyter Notebook Research**
Researchers use the agent in notebooks to query textbook content while writing papers.

### 4. **Slack Workspace Bot**
Team collaboration with AI assistant integrated into Slack.

### 5. **WhatsApp Bot**
Use Twilio + RAG Agent to create a WhatsApp study assistant.

### 6. **Telegram Bot**
Deploy on Heroku/Railway with python-telegram-bot library.

---

## 🐛 Troubleshooting

### "Collection not found" Error

```bash
# Make sure you've indexed your content first
cd backend
python scripts/index_content.py
```

### "OpenAI API Key not set"

```bash
export OPENAI_API_KEY=sk-your-key-here
```

### Qdrant Connection Error

```bash
# If using local Qdrant
docker run -p 6333:6333 qdrant/qdrant

# Or set URL to Qdrant Cloud
export QDRANT_URL=https://your-cluster.qdrant.io
export QDRANT_API_KEY=your-key
```

---

## 📚 Dependencies

Already included in `backend/requirements.txt`:
- `openai` - OpenAI API
- `qdrant-client` - Vector database
- `python-dotenv` - Environment variables

---

## 🚀 Next Steps

1. **Copy `rag_agent.py` to your new project**
2. **Install dependencies**: `pip install openai qdrant-client python-dotenv`
3. **Set environment variables**
4. **Import and use!**

```python
from rag_agent import RAGAgent, RAGConfig

agent = RAGAgent(RAGConfig.from_env())
response = await agent.ask("Your question here")
print(response.content)
```

---

## 📄 License

MIT License - Use this agent anywhere you want!

---

## 💡 Tips

- Use `temperature=0.3` for factual answers, `0.7-0.9` for creative answers
- Increase `top_k` to 10 for more comprehensive answers (slower)
- Set `relevance_threshold=0.8` for stricter source filtering
- Export conversations with `/export` command to analyze later
- Clear history with `/clear` if switching topics

---

Enjoy your portable RAG chatbot! 🎉
