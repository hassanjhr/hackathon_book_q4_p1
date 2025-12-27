#!/bin/bash
# RAG Chatbot Testing Script
# Usage: ./test_rag.sh

BASE_URL="http://localhost:8000/api"

echo "================================================"
echo "RAG Chatbot API Testing"
echo "================================================"
echo ""

echo "1. Testing API Health Check..."
curl -s -X GET "${BASE_URL}/rag/health" | python3 -m json.tool
echo ""
echo ""

echo "2. Testing General Question..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is Physical AI?",
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "3. Testing Question with Selected Text..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Summarize this text",
    "selected_text": "Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world through embodied agents like robots.",
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "4. Testing Quick Test Endpoint..."
curl -s -X POST "${BASE_URL}/rag/test?question=What%20is%20a%20humanoid%20robot?" | python3 -m json.tool
echo ""
echo ""

echo "================================================"
echo "Testing Complete!"
echo "================================================"
