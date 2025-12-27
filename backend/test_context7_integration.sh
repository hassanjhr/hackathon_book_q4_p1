#!/bin/bash
# Test script for Context7 RAG integration
# Usage: ./test_context7_integration.sh

BASE_URL="http://localhost:8000/api"

echo "================================================"
echo "Context7 RAG Integration Testing"
echo "================================================"
echo ""

echo "1. Testing Library Question (PyTorch) with Context7..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How do I create a PyTorch tensor?",
    "use_library_docs": true,
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "2. Testing Library Question WITHOUT Context7..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How do I create a PyTorch tensor?",
    "use_library_docs": false,
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "3. Testing Non-Library Question (should use textbook only)..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is reinforcement learning?",
    "use_library_docs": true,
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "4. Testing Library Question - ROS2..."
curl -s -X POST "${BASE_URL}/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How do I create a ROS2 node?",
    "use_library_docs": true,
    "include_sources": true
  }' | python3 -m json.tool
echo ""
echo ""

echo "5. Testing Health Check..."
curl -s -X GET "${BASE_URL}/rag/health" | python3 -m json.tool
echo ""
echo ""

echo "================================================"
echo "Testing Complete!"
echo "================================================"
echo ""
echo "Expected Results:"
echo "- Test 1: Should include both textbook AND library_docs sources"
echo "- Test 2: Should only include textbook sources"
echo "- Test 3: Should only include textbook sources (no library detected)"
echo "- Test 4: Should attempt Context7, may fall back to textbook if ROS2 not indexed"
echo "- Test 5: Should show all services healthy"
