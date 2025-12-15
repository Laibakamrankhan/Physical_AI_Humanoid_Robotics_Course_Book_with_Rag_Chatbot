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
RUN pip install "fastapi[all]" "uvicorn[standard]"

# Copy the application code
COPY . .

# Expose port
EXPOSE 8000

# Create a startup script to properly handle environment variables
RUN echo '#!/bin/bash\n\
set -e\n\
echo "Starting application on port ${PORT:-8000}"\n\
python -c "import uvicorn; uvicorn.run(\"api:app\", host=\"0.0.0.0\", port=int(\"${PORT:-8000}\"), log_level=\"info\")"' > /start.sh
RUN chmod +x /start.sh

# Run the application
CMD ["/start.sh"]
