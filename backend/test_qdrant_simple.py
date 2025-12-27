#!/usr/bin/env python3
"""
Simple standalone Qdrant connection test (no dependencies on backend code).

Usage:
    python backend/test_qdrant_simple.py
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Try importing qdrant-client
try:
    from qdrant_client import QdrantClient
    print("✅ qdrant-client is installed")
except ImportError:
    print("❌ qdrant-client not installed!")
    print("   Run: pip install qdrant-client")
    exit(1)

print()
print("=" * 60)
print("Qdrant Cloud Connection Test (Standalone)")
print("=" * 60)
print()

# Get configuration from environment
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content")

print("📋 Configuration:")
print(f"   URL: {QDRANT_URL}")
print(f"   API Key: {QDRANT_API_KEY[:20]}..." if QDRANT_API_KEY else "   API Key: NOT SET")
print(f"   Collection: {QDRANT_COLLECTION}")
print()

# Validate configuration
if not QDRANT_URL:
    print("❌ QDRANT_URL not set in .env file!")
    exit(1)

if not QDRANT_API_KEY:
    print("❌ QDRANT_API_KEY not set in .env file!")
    exit(1)

# Check URL format
warnings = []
if QDRANT_URL.startswith("http://") or QDRANT_URL.startswith("https://"):
    warnings.append("⚠️  URL should NOT include http:// or https://")
    warnings.append(f"   Current: {QDRANT_URL}")
    warnings.append("   Fix: Remove 'https://' prefix in .env")

if ":6333" not in QDRANT_URL and ":6334" not in QDRANT_URL:
    warnings.append("⚠️  URL should include port :6333 or :6334")
    warnings.append(f"   Current: {QDRANT_URL}")
    warnings.append("   Fix: Add ':6333' suffix in .env")

if warnings:
    print("WARNINGS:")
    for warning in warnings:
        print(warning)
    print()
    print("Expected format: host.qdrant.io:6333")
    print("Example: bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333")
    print()

# Test connection
print("🔄 Testing connection...")
print()

try:
    # Initialize client
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=False,
        https=True,
        timeout=30
    )

    print("✅ Client initialized successfully")
    print()

    # Test: Get collections
    print("📦 Fetching collections...")
    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]

    print(f"✅ Found {len(collection_names)} collection(s):")
    for name in collection_names:
        print(f"   - {name}")
    print()

    # Check target collection
    if QDRANT_COLLECTION in collection_names:
        collection_info = client.get_collection(QDRANT_COLLECTION)
        print(f"✅ Collection '{QDRANT_COLLECTION}' exists!")
        print(f"   Vectors count: {collection_info.vectors_count}")
        print(f"   Points count: {collection_info.points_count}")
    else:
        print(f"ℹ️  Collection '{QDRANT_COLLECTION}' does not exist yet")
        print("   (It will be created automatically on first use)")
    print()

    print("=" * 60)
    print("🎉 SUCCESS! Qdrant connection works!")
    print("=" * 60)

except Exception as e:
    print("=" * 60)
    print("❌ CONNECTION FAILED")
    print("=" * 60)
    print()
    print(f"Error: {type(e).__name__}")
    print(f"Message: {str(e)}")
    print()

    # Debugging hints
    error_str = str(e)

    if "Name or service not known" in error_str or "getaddrinfo failed" in error_str:
        print("💡 DNS Resolution Error - Possible fixes:")
        print("   1. Check internet connection")
        print("   2. Verify URL has no typos")
        print("   3. Test DNS: nslookup <your-cluster>.qdrant.io")
        print("   4. If on WSL, check /etc/resolv.conf")

    elif "404" in error_str:
        print("💡 404 Not Found - Fix:")
        print("   1. Remove 'https://' from QDRANT_URL in .env")
        print("   2. Add port ':6333' to QDRANT_URL")
        print(f"   Current: {QDRANT_URL}")
        print("   Should be: host.qdrant.io:6333")

    elif "401" in error_str or "403" in error_str or "Unauthorized" in error_str:
        print("💡 Authentication Error - Fix:")
        print("   1. Check API key in Qdrant Cloud dashboard")
        print("   2. Regenerate API key if needed")
        print("   3. Check IP whitelist (if configured)")

    elif "timeout" in error_str.lower():
        print("💡 Timeout Error - Fix:")
        print("   1. Check Qdrant cluster is running (not paused)")
        print("   2. Check internet connection")
        print("   3. Try again in a few moments")

    else:
        print("💡 Unknown Error - Check:")
        print("   1. Qdrant Cloud dashboard: https://cloud.qdrant.io/")
        print("   2. Cluster status (running vs paused)")
        print("   3. Full error trace above")

    print()
    exit(1)
