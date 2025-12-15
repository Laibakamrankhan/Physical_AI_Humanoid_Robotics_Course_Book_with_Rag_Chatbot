FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirement.txt .
RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir -r requirement.txt
RUN pip install --no-cache-dir "fastapi[all]" "uvicorn[standard]"

# Copy the application code
COPY . .

# Expose port
EXPOSE 8000

# Create a startup script with proper error handling
RUN echo '#!/bin/sh\n\
set -e\n\
echo "Starting Physical AI Humanoid Robotics Backend"\n\
echo "Available on port: ${PORT:-8000}"\n\
echo "Environment variables check:"\n\
echo "  COHERE_API_KEY set: $(if [ -n "$COHERE_API_KEY" ]; then echo "yes"; else echo "no"; fi)"\n\
echo "  QDRANT_URL set: $(if [ -n "$QDRANT_URL" ]; then echo "yes"; else echo "no"; fi)"\n\
\n\
# Run the application\n\
exec python -c "\n\
import os, uvicorn, sys\n\
from api import app\n\
\n\
port = int(os.environ.get(\"PORT\", 8000))\n\
print(f\"Starting server on port {port}\")\n\
uvicorn.run(app, host=\"0.0.0.0\", port=port, log_level=\"info\")\n\
"' > /start_server.sh
RUN chmod +x /start_server.sh

# Run the application using the startup script
CMD ["/start_server.sh"]
