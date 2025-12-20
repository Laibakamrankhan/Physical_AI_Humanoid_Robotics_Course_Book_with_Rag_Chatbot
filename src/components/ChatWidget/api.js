// Ensure these are set globally before frontend runs (e.g., in docusaurus.config.js)
if (typeof window !== 'undefined') {
  window.chatApiBaseUrl = 'https://physicalaihumanoidroboticscoursebookwithr-production.up.railway.app';
  window.chatApiKey = 'YOUR_BACKEND_KEY_HERE'; // Replace with your actual key
}

// API client for the RAG chatbot
const API_BASE_URL = typeof window !== 'undefined' && window.chatApiBaseUrl
  ? window.chatApiBaseUrl
  : 'https://physicalaihumanoidroboticscoursebookwithr-production.up.railway.app';

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
          "Authorization": `Bearer ${typeof window !== 'undefined' ? window.chatApiKey : ''}`,
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

  async chatStream(message, sessionId = null, documentContext = null, onToken) {
    try {
      const response = await fetch(`${this.baseURL}/chat/stream`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          "Authorization": `Bearer ${typeof window !== 'undefined' ? window.chatApiKey : ''}`,
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
        buffer = lines.pop();

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.slice(6);
            if (data === '[DONE]') return;

            try {
              const parsed = JSON.parse(data);
              if (parsed.type === 'token') {
                onToken(parsed.token);
              } else if (parsed.type === 'sources') {
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
          "Authorization": `Bearer ${typeof window !== 'undefined' ? window.chatApiKey : ''}`,
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
      const response = await fetch(`${this.baseURL}/health`, {
        headers: {
          "Authorization": `Bearer ${typeof window !== 'undefined' ? window.chatApiKey : ''}`,
        },
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} - ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      if (error.message.includes('fetch')) {
        return { status: 'unreachable', message: 'API server is not reachable' };
      }
      throw error;
    }
  }
}

// Singleton
const chatAPI = new ChatAPI();
export default chatAPI;
