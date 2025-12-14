# RAG Chatbot Implementation Plan

## 1. Content Ingestion Pipeline

### 1.1 Parse Markdown Files from Docusaurus
- **Task**: Extract content from `1-physical-ai-robotics/` directory
- **Approach**:
  - Use Python to recursively read all `.md` files
  - Parse markdown content using `markdown` or `mistune` library
  - Extract document metadata (title, headings, content)
  - Preserve document structure and hierarchy
- **Files**: All markdown files in documentation directory
- **Output**: Clean text content with metadata

### 1.2 Apply Semantic Chunking Strategy
- **Task**: Split documents into semantic chunks (300-500 tokens)
- **Approach**:
  - Implement recursive text splitter that respects document structure
  - Chunk by headings, paragraphs, and semantic boundaries
  - Add 20% overlap between chunks to preserve context
  - Preserve metadata (document ID, URL, section) with each chunk
- **Tools**: Custom chunking logic or `langchain` text splitters
- **Output**: List of document chunks with metadata

### 1.3 Generate Embeddings Using Cohere
- **Task**: Convert text chunks to vector embeddings
- **Approach**:
  - Use Cohere's embedding API to generate vector representations
  - Batch process chunks for efficiency
  - Handle API rate limits and errors gracefully
  - Cache embeddings to avoid recomputation
- **API**: Cohere Embed API
- **Output**: Vector embeddings for each document chunk

### 1.4 Store Vectors and Metadata in Qdrant
- **Task**: Store embeddings and metadata in vector database
- **Approach**:
  - Create Qdrant collection for document embeddings
  - Store vectors with metadata (content, URL, title, document ID)
  - Implement upsert logic for incremental updates
  - Create vector index for efficient similarity search
- **Tools**: Qdrant Python client
- **Output**: Vector database with searchable document embeddings

## 2. Backend (FastAPI)

### 2.1 `POST /ingest` - Index and Re-index Book Content
- **Endpoint**: `POST /api/v1/ingest`
- **Request Body**:
  ```json
  {
    "force_reindex": false,
    "document_path": "1-physical-ai-robotics/"
  }
  ```
- **Implementation**:
  - Trigger document parsing and chunking
  - Generate embeddings for new/updated chunks
  - Store vectors in Qdrant with metadata
  - Return indexing statistics
- **Response**:
  ```json
  {
    "status": "success",
    "documents_processed": 10,
    "chunks_created": 150,
    "vectors_stored": 150
  }
  ```

### 2.2 `POST /chat` - RAG-based Answers Using Full Book Context
- **Endpoint**: `POST /api/v1/chat`
- **Request Body**:
  ```json
  {
    "message": "How do I implement ROS 2 nodes?",
    "session_id": "session-123",
    "document_context": "CH01-ros2-nervous-system"
  }
  ```
- **Implementation**:
  - Embed user query using Cohere
  - Search Qdrant for relevant document chunks
  - Retrieve top 5 most relevant chunks
  - Format context for OpenAI Agent
  - Generate response with citations
  - Store conversation in Neon Postgres
- **Response**:
  ```json
  {
    "response": "To implement ROS 2 nodes...",
    "sources": [
      {
        "title": "ROS 2 Nervous System",
        "url": "/CH01-ros2-nervous-system",
        "snippet": "In ROS 2, nodes are implemented using...",
        "score": 0.85
      }
    ],
    "session_id": "session-123"
  }
  ```

### 2.3 Session and Chat History Persistence
- **Database**: Neon Serverless Postgres
- **Tables**:
  - `sessions`: session_id, user_id, created_at, updated_at
  - `chat_messages`: id, session_id, role, content, timestamp, sources
- **Implementation**:
  - Create session records for new conversations
  - Store chat messages with sources
  - Retrieve conversation history for context
  - Implement data retention policies

## 3. RAG Execution Flow

### 3.1 Receive User Query
- **Input**: User's natural language question
- **Processing**: Validate and sanitize input
- **Context**: Extract current document context if available

### 3.2 Retrieve Relevant Chunks from Qdrant
- **Query Embedding**: Convert user query to vector using Cohere
- **Similarity Search**: Find top 5 most similar document chunks
- **Filtering**: Apply document context filter if specified
- **Scoring**: Return chunks with similarity scores

### 3.3 Inject Retrieved Context into OpenAI Agent
- **Prompt Engineering**: Format retrieved context for agent
- **System Prompt**: Set up agent with documentation context
- **Message History**: Include conversation history if available
- **Response Generation**: Generate contextual response

### 3.4 Generate Grounded Response with Citations
- **Response Formatting**: Structure response with clear citations
- **Source Attribution**: Link back to relevant documentation
- **Confidence Scoring**: Include confidence in retrieved sources
- **Output Validation**: Ensure response is helpful and accurate

## 4. Frontend Integration (Docusaurus)

### 4.1 Embedded Chatbot UI Component
- **Location**: Floating widget on all documentation pages
- **Design**: Match Docusaurus theme and styling
- **Components**:
  - Chat window with message history
  - Input area with send button
  - Typing indicators
  - Source citation display
- **State Management**: React hooks for local state

### 4.2 Chat Input and Response Streaming
- **Implementation**:
  - Real-time message display
  - Streaming response from backend
  - Error handling and retry logic
  - Session management
- **Features**:
  - Typing indicators
  - Message status (sent, delivered, read)
  - Scroll to bottom on new messages

### 4.3 API Integration with FastAPI Backend
- **API Client**: Axios or fetch for HTTP requests
- **Endpoints**:
  - `/api/v1/chat` for chat messages
  - `/api/v1/health` for health checks
- **Authentication**: Integrate with existing auth system
- **Error Handling**: Graceful degradation on API failures

## 5. Implementation Phases

### Phase 1: Backend Infrastructure
1. Set up FastAPI application structure
2. Implement document ingestion pipeline
3. Create Qdrant vector database setup
4. Implement Cohere embedding integration

### Phase 2: RAG Core Functionality
1. Implement Qdrant search functionality
2. Integrate OpenAI Agents for response generation
3. Create chat endpoint with full RAG flow
4. Add session management with Neon Postgres

### Phase 3: Frontend Integration
1. Create React chat component
2. Integrate with FastAPI backend
3. Add to Docusaurus theme
4. Implement streaming responses

### Phase 4: Production Features
1. Add authentication integration
2. Implement rate limiting
3. Add comprehensive error handling
4. Performance optimization and caching

## 6. Dependencies and Setup

### Backend Dependencies:
- FastAPI
- Cohere Python SDK
- Qdrant Python client
- Neon Postgres connector
- OpenAI Python SDK
- Pydantic for data validation

### Frontend Dependencies:
- React components for chat UI
- Axios for API calls
- Integration with existing Docusaurus setup

## 7. Testing Strategy

### Unit Tests:
- Document parsing and chunking functions
- Embedding generation utilities
- Database interaction methods

### Integration Tests:
- End-to-end chat functionality
- Vector search accuracy
- API endpoint validation

### Performance Tests:
- Response time under load
- Vector search performance
- Caching effectiveness