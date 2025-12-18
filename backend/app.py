from app.api_main import app

# This file serves as the entry point for Hugging Face Spaces
# Hugging Face Spaces looks for an 'app' variable that represents the FastAPI app
# The 'app' variable is imported from app.api_main and is a FastAPI instance

if __name__ == "__main__":
    import os
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8000)))
