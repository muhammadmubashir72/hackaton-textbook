"""
Test script to directly test the backend API
"""
import requests
import json

# Test if the backend is running
try:
    # Test the root endpoint
    response = requests.get("http://localhost:8000/")
    print("Root endpoint response:", response.json())
    print("Status code:", response.status_code)
except Exception as e:
    print(f"Error connecting to backend: {e}")
    print("Make sure the backend is running on http://localhost:8000")

# Test the health endpoints if available
try:
    response = requests.get("http://localhost:8000/health")
    print("Health endpoint response:", response.json())
    print("Health status code:", response.status_code)
except Exception as e:
    print(f"Error connecting to health endpoint: {e}")

# Test the qdrant health endpoint if available
try:
    response = requests.get("http://localhost:8000/health/qdrant")
    print("Qdrant health endpoint response:", response.json())
    print("Qdrant health status code:", response.status_code)
except Exception as e:
    print(f"Error connecting to qdrant health endpoint: {e}")

# Try a test query (this may fail if services are not properly configured)
try:
    test_payload = {
        "query": "What is a humanoid robot?",
        "top_k": 5
    }
    response = requests.post("http://localhost:8000/query", 
                           json=test_payload, 
                           headers={'Content-Type': 'application/json'})
    print("Query endpoint response:", response.json())
    print("Query status code:", response.status_code)
except Exception as e:
    print(f"Error connecting to query endpoint: {e}")
    print("This is likely due to missing API keys or services not being configured properly")