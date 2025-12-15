FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY backend/requirement.txt .
RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir -r requirement.txt
RUN pip install --no-cache-dir "fastapi[all]" "uvicorn[standard]"

# Copy the application code
COPY backend/. .

# Verify files are copied correctly
RUN ls -la /app

# Expose port
EXPOSE 8000

# Create a startup script with proper error handling and Python path
RUN printf '%s\n' \
'#!/usr/bin/env python3' \
'import os' \
'import sys' \
'import importlib.util' \
'' \
'# Add current directory to Python path' \
'sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))' \
'' \
'print("Starting Physical AI Humanoid Robotics Backend")' \
'port = os.environ.get("PORT", 8000)' \
'print(f"Available on port: {port}")' \
'cohere_set = "yes" if os.environ.get("COHERE_API_KEY") else "no"' \
'qdrant_set = "yes" if os.environ.get("QDRANT_URL") else "no"' \
'print(f"COHERE_API_KEY set: {cohere_set}")' \
'print(f"QDRANT_URL set: {qdrant_set}")' \
'' \
'# Import and run the application' \
'try:' \
'    import uvicorn' \
'    from api import app' \
'' \
'    port = int(os.environ.get("PORT", 8000))' \
'    print(f"Starting server on port {port}")' \
'    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")' \
'except ImportError as e:' \
'    print(f"Import error: {e}")' \
'    sys.exit(1)' \
'except Exception as e:' \
'    print(f"Error starting application: {e}")' \
'    import traceback' \
'    traceback.print_exc()' \
'    sys.exit(1)' > /app/start_server.py

RUN chmod +x /app/start_server.py

# Run the application using Python directly
CMD ["python", "/app/start_server.py"]
