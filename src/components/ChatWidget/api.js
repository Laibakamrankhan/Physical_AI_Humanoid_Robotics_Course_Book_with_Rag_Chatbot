// API client for the RAG chatbot
// Using a global variable that can be configured in docusaurus.config.js or default local backend URL
const API_BASE_URL = typeof window !== 'undefined' && window.chatApiBaseUrl
  ? window.chatApiBaseUrl
  : 'http://localhost:8000';

class ChatAPI {
  constructor() {
    this.baseURL = API_BASE_URL;
  }

  async chat(message, sessionId = null, documentContext = null) {
    try {
      const response = await fetch(`${this.baseURL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          "Authorization": `Bearer ${process.env.REACT_APP_API_KEY}`,
        },
        body: JSON.stringify({
          message,
          session_id: sessionId,
          document_context: documentContext
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Chat API error:', error);
      throw error;
    }
  }

  // Streaming chat method (to be used when backend supports streaming)
  async chatStream(message, sessionId = null, documentContext = null, onToken) {
    try {
      const response = await fetch(`${this.baseURL}/chat/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          session_id: sessionId,
          document_context: documentContext
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop(); // Keep last incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.slice(6); // Remove 'data: ' prefix
            if (data === '[DONE]') {
              return; // Stream ended
            }

            try {
              const parsed = JSON.parse(data);
              if (parsed.type === 'token') {
                onToken(parsed.token);
              } else if (parsed.type === 'sources') {
                // Handle sources data
                onToken({ type: 'sources', data: parsed.data });
              }
            } catch (e) {
              console.error('Error parsing stream data:', e);
            }
          }
        }
      }
    } catch (error) {
      console.error('Streaming chat API error:', error);
      throw error;
    }
  }

  async ingest(forceReindex = false, documentPath = '1-physical-ai-robotics/') {
    try {
      const response = await fetch(`${this.baseURL}/ingest`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          force_reindex: forceReindex,
          document_path: documentPath
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Ingest API error:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.baseURL}/health`);

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      // Return a mock health response in case of network error
      if (error.message.includes('fetch')) {
        return { status: 'unreachable', message: 'API server is not reachable' };
      }
      throw error;
    }
  }
}

// Create a singleton instance
const chatAPI = new ChatAPI();

export default chatAPI;
