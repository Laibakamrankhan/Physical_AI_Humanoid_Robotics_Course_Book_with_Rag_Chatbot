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

# Create a startup script to ensure proper execution
RUN echo '#!/bin/bash\n\
set -e\n\
echo "Starting application..."\n\
python -c "import uvicorn; print(uvicorn.__version__)"\n\
exec uvicorn api:app --host 0.0.0.0 --port ${PORT:-8000}' > /start.sh
RUN chmod +x /start.sh

# Run the application
CMD ["/start.sh"]
