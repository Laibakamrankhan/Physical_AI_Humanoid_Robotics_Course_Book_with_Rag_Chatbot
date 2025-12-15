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

# Verify installations
RUN python -c "import fastapi; print(f'FastAPI version: {fastapi.__version__}')"
RUN python -c "import uvicorn; print(f'Uvicorn version: {uvicorn.__version__}')"
RUN python -c "import api; print('API module imported successfully')"

# Expose port
EXPOSE 8000

# Create a startup script that Railway can execute directly
RUN echo '#!/usr/bin/env python3\n\
import os\n\
import uvicorn\n\
from api import app\n\n\
if __name__ == "__main__":\n\
    port = int(os.environ.get("PORT", 8000))\n\
    print(f"Starting server on port {port}")\n\
    uvicorn.run(\n\
        "api:app",\n\
        host="0.0.0.0",\n\
        port=port,\n\
        log_level="info"\n\
    )' > /app/start_server.py

# Run the application using Python directly
CMD ["python", "/app/start_server.py"]
