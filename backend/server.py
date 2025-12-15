"""
Server entry point for Railway deployment
This file ensures the app can be started in multiple ways
"""
import os
import uvicorn
from api import app

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8000))
    print(f"Starting Physical AI Humanoid Robotics Backend on port {port}")
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )