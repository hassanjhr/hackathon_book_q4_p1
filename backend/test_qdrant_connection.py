#!/usr/bin/env python3
"""
Test script to verify Qdrant Cloud connection.

Usage:
    python backend/test_qdrant_connection.py
"""
import sys
from pathlib import Path

# Add backend to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from src.config import settings
from qdrant_client import QdrantClient
import traceback


def test_qdrant_connection():
    """Test Qdrant connection with detailed debugging."""

    print("=" * 60)
    print("Qdrant Cloud Connection Test")
    print("=" * 60)
    print()

    # Display configuration
    print("📋 Configuration:")
    print(f"   URL: {settings.qdrant_url}")
    print(f"   API Key: {settings.qdrant_api_key[:20]}..." if settings.qdrant_api_key else "   API Key: NOT SET")
    print(f"   Collection: {settings.qdrant_collection_name}")
    print()

    # Check URL format
    if settings.qdrant_url.startswith("http://") or settings.qdrant_url.startswith("https://"):
        print("⚠️  WARNING: URL should NOT include http:// or https://")
        print("   Current:", settings.qdrant_url)
        print("   Should be: bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333")
        print()

    if ":6333" not in settings.qdrant_url and ":6334" not in settings.qdrant_url:
        print("⚠️  WARNING: URL should include port :6333 or :6334")
        print("   Current:", settings.qdrant_url)
        print()

    # Test connection
    print("🔄 Testing connection...")
    print()

    try:
        # Initialize client
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,
            https=True,
            timeout=30
        )

        # Test 1: Get collections
        print("✅ Test 1: Get Collections")
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]
        print(f"   Found {len(collection_names)} collection(s): {collection_names}")
        print()

        # Test 2: Check if target collection exists
        print(f"✅ Test 2: Check Collection '{settings.qdrant_collection_name}'")
        if settings.qdrant_collection_name in collection_names:
            collection_info = client.get_collection(settings.qdrant_collection_name)
            print(f"   Collection exists!")
            print(f"   Vectors count: {collection_info.vectors_count}")
            print(f"   Points count: {collection_info.points_count}")
        else:
            print(f"   Collection '{settings.qdrant_collection_name}' does NOT exist yet")
            print(f"   It will be created automatically on first use")
        print()

        # Test 3: Check cluster info
        print("✅ Test 3: Cluster Info")
        try:
            cluster_info = client.get_cluster_info()
            print(f"   Cluster status: {cluster_info.status}")
        except Exception as e:
            print(f"   Cluster info not available (normal for cloud): {e}")
        print()

        print("=" * 60)
        print("🎉 ALL TESTS PASSED!")
        print("=" * 60)
        print()
        print("Your Qdrant Cloud connection is working correctly!")
        print()
        return True

    except Exception as e:
        print("=" * 60)
        print("❌ CONNECTION FAILED")
        print("=" * 60)
        print()
        print(f"Error type: {type(e).__name__}")
        print(f"Error message: {str(e)}")
        print()
        print("Full traceback:")
        traceback.print_exc()
        print()

        # Debugging hints
        print("💡 Debugging Hints:")
        print()

        if "Name or service not known" in str(e):
            print("❌ DNS Resolution Error")
            print("   - Check your internet connection")
            print("   - Verify the URL is correct (no typos)")
            print("   - If on WSL, check DNS settings:")
            print("     nslookup bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io")
            print()

        if "404" in str(e):
            print("❌ 404 Not Found")
            print("   - Remove https:// from QDRANT_URL in .env")
            print("   - Add port :6333 to QDRANT_URL")
            print("   - Current URL:", settings.qdrant_url)
            print("   - Should be: host:6333 (without https://)")
            print()

        if "401" in str(e) or "403" in str(e):
            print("❌ Authentication Error")
            print("   - Check your API key is correct")
            print("   - Verify the API key in Qdrant Cloud dashboard")
            print("   - Check if IP whitelist allows your IP")
            print()

        if "timeout" in str(e).lower():
            print("❌ Timeout Error")
            print("   - Check your internet connection")
            print("   - Verify Qdrant cluster is running (not paused)")
            print("   - Try increasing timeout in code")
            print()

        print("📚 Documentation:")
        print("   Qdrant Cloud: https://cloud.qdrant.io/")
        print("   Qdrant Client: https://github.com/qdrant/qdrant-client")
        print()

        return False


if __name__ == "__main__":
    success = test_qdrant_connection()
    sys.exit(0 if success else 1)
