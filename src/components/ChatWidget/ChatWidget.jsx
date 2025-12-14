import React, { useState, useEffect, useRef } from 'react';
import chatAPI from './api';
import './ChatWidget.css';

const ChatWidget = ({ documentContext = null }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Toggle chat widget open/closed
  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  // Handle sending a message with potential streaming support
  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading || isStreaming) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Try regular chat API
      const data = await chatAPI.chat(inputValue, sessionId, documentContext);

      // Update session ID if new one was provided
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add AI response to chat
      const aiMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setIsStreaming(false);
    }
  };

  // Format sources for display
  const formatSources = (sources) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="chat-sources">
        <div className="sources-header">Sources:</div>
        {sources.map((source, index) => (
          <div key={index} className="source-item">
            <a href={source.url} target="_blank" rel="noopener noreferrer">
              {source.title}
            </a>
            <div className="source-snippet">{source.snippet}</div>
          </div>
        ))}
      </div>
    );
  };

  return (
    <div className={`chat-widget ${isOpen ? 'open' : ''}`}>
      {!isOpen ? (
        <button className="chat-toggle-btn" onClick={toggleChat}>
          <span className="chat-icon">💬</span>
        </button>
      ) : (
        <div className="chat-container">
          <div className="chat-header">
            <h3>Ask about the Book</h3>
            <button className="chat-close-btn" onClick={toggleChat}>
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome">
                <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
                <p>Ask me anything about the content, and I'll provide answers based on the documentation.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chat-message ${message.role}`}
                >
                  <div className="message-content">
                    {message.content}
                    {message.role === 'assistant' && formatSources(message.sources)}
                  </div>
                  <div className="message-timestamp">
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {(isLoading && !isStreaming) && (
              <div className="chat-message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className="chat-input-form" onSubmit={handleSendMessage}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the book..."
              disabled={isLoading || isStreaming}
              className="chat-input"
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading || isStreaming}
              className="chat-send-btn"
            >
              {isStreaming ? '...' : 'Send'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;