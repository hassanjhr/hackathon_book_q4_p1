#!/bin/bash

# Quick check if servers are running
echo "🔍 Checking Server Status..."
echo ""

# Check Backend (port 8000)
if curl -s http://localhost:8000/health > /dev/null 2>&1; then
    echo "✅ Backend is RUNNING at http://localhost:8000"
else
    echo "❌ Backend is NOT running yet"
fi

# Check Frontend (port 3000)
if curl -s http://localhost:3000 > /dev/null 2>&1; then
    echo "✅ Frontend is RUNNING at http://localhost:3000"
else
    echo "❌ Frontend is NOT running yet"
fi

echo ""
echo "💡 If both are running, open: http://localhost:3000"
