#!/bin/bash

# Quick Start Script for RAG Chatbot
# Run this to start both frontend and backend servers

echo "🚀 Starting RAG Chatbot Servers..."
echo ""

# Check if we're in the right directory
if [ ! -d "backend" ] || [ ! -d "my-website" ]; then
    echo "❌ Error: Please run this script from the project root directory"
    exit 1
fi

# Function to start backend
start_backend() {
    echo "📦 Starting Backend Server..."
    cd backend

    # Create venv if doesn't exist
    if [ ! -d "venv" ]; then
        echo "   Creating virtual environment..."
        python3 -m venv venv
    fi

    # Activate and install
    source venv/bin/activate
    echo "   Installing dependencies..."
    pip install -q fastapi uvicorn[standard] qdrant-client openai python-dotenv asyncpg pydantic-settings

    # Start server
    echo "   Starting uvicorn..."
    uvicorn src.main:app --reload --host 0.0.0.0 --port 8000 &
    BACKEND_PID=$!

    cd ..
    echo "✅ Backend started (PID: $BACKEND_PID)"
}

# Function to start frontend
start_frontend() {
    echo "🌐 Starting Frontend Server..."
    cd my-website

    # Check if node_modules exists
    if [ ! -d "node_modules" ]; then
        echo "   Installing npm dependencies (this may take a while)..."
        npm install
    fi

    # Start server
    echo "   Starting npm..."
    npm start &
    FRONTEND_PID=$!

    cd ..
    echo "✅ Frontend started (PID: $FRONTEND_PID)"
}

# Start both servers
start_backend
sleep 2
start_frontend

echo ""
echo "=" * 60
echo "🎉 Both servers are starting!"
echo "=" * 60
echo ""
echo "📍 URLs:"
echo "   Backend:  http://localhost:8000"
echo "   Frontend: http://localhost:3000"
echo ""
echo "🧪 To test:"
echo "   1. Open http://localhost:3000 in your browser"
echo "   2. Click the 💬 chat button (bottom-right corner)"
echo "   3. Ask: 'What is Physical AI?'"
echo ""
echo "⏹️  To stop servers:"
echo "   Press Ctrl+C or run: killall -9 uvicorn node"
echo ""
echo "📊 Monitor logs:"
echo "   Backend:  tail -f backend/logs/*.log (if logging enabled)"
echo "   Frontend: Check terminal output"
echo ""

# Wait for user interrupt
wait
