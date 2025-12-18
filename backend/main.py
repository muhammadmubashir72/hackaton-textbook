import os
import uvicorn
from app.api_main import app

if __name__ == "__main__":
    uvicorn.run(
        "app.api_main:app",
        host="0.0.0.0",
        port=int(os.environ.get("PORT", 7860)),  # Default to Hugging Face standard port
        reload=False,  # Disable reload for production
        log_level="info"
    )