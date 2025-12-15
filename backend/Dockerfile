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

# Create a startup script
RUN echo '#!/bin/sh\n\
echo "Starting application on port ${PORT:-8000}"\n\
echo "Available Python packages:"\n\
pip list | grep -i uvicorn\n\
python -c "import sys; print(sys.path)"\n\
exec python -c "import uvicorn, os; uvicorn.run(\"api:app\", host=\"0.0.0.0\", port=int(os.environ.get(\"PORT\", 8000)), log_level=\"info")"' > /start.sh
RUN chmod +x /start.sh

# Run the application
CMD ["/start.sh"]
