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

# Ensure uvicorn is available in PATH
RUN pip install --no-cache-dir --force-reinstall "uvicorn[standard]"

# Copy the application code
COPY . .

# Verify installations
RUN python -c "import fastapi; print(f'FastAPI version: {fastapi.__version__}')"
RUN python -c "import uvicorn; print(f'Uvicorn version: {uvicorn.__version__}')"
RUN python -c "import api; print('API module imported successfully')"

# Verify uvicorn command is available
RUN which uvicorn || echo "uvicorn not in PATH"
RUN python -m pip show uvicorn

# Expose port
EXPOSE 8000

# Run the application using uvicorn directly (now it should be available)
CMD ["sh", "-c", "uvicorn api:app --host 0.0.0.0 --port ${PORT:-8000}"]
