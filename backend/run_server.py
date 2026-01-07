import uvicorn
import os
from app.api_main import app
from app.database import create_all_tables

def main():
    # Create all database tables on startup
    create_all_tables()
    print("Database tables created successfully!")

    # Run the application
    uvicorn.run(
        "app.api_main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8001)),
        reload=True  # Enable auto-reload for development
    )

if __name__ == "__main__":
    main()