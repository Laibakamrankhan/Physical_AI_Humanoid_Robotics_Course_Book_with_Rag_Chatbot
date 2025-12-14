@echo off
echo Document Ingestion Script for Physical AI & Humanoid Robotics Book
echo =====================================================================

echo Before running this script, please ensure you have:
echo 1. Valid API keys from Cohere, Qdrant, and optionally OpenAI
echo 2. The backend server running on port 8000
echo 3. A running Qdrant vector database
echo 4. A PostgreSQL database running
echo.

REM Check if backend is running
curl -s http://localhost:8000/api/v1/health >nul 2>&1
if %errorlevel% == 0 (
    echo ✅ Backend server: RUNNING
) else (
    echo ❌ Backend server: NOT RUNNING - Please start the backend first
    echo    Run: cd backend ^&^& python -m uvicorn main:app --reload
    pause
    exit /b 1
)

REM Check for API keys (simplified check)
if "%COHERE_API_KEY%"=="" (
    echo ❌ COHERE_API_KEY: NOT SET - Please set a valid Cohere API key
    echo    Get one at: https://dashboard.cohere.com/api-keys
    pause
    exit /b 1
) else if "%COHERE_API_KEY%"=="COHERE_API_KEY" (
    echo ❌ COHERE_API_KEY: PLACEHOLDER VALUE - Please set a valid Cohere API key
    echo    Get one at: https://dashboard.cohere.com/api-keys
    pause
    exit /b 1
) else (
    echo ✅ COHERE_API_KEY: SET
)

if "%QDRANT_URL%"=="" (
    echo ❌ QDRANT_URL: NOT SET - Please set a valid Qdrant URL
    echo    You can run Qdrant locally with: docker run -p 6333:6333 qdrant/qdrant
    pause
    exit /b 1
) else (
    echo ✅ QDRANT_URL: SET
)

echo.
echo Starting document ingestion from the project root directory...
echo.

REM Run the ingestion (from project root, not backend directory)
curl -X POST http://localhost:8000/api/v1/ingest ^
     -H "Content-Type: application/json" ^
     -d "{^
       \"force_reindex\": true,^
       \"document_path\": \"1-physical-ai-robotics/\"^
     }"

echo.
echo.
echo Ingestion completed! The chatbot should now have access to the book content.
echo You can test it by asking questions about the Physical AI & Humanoid Robotics book.
pause