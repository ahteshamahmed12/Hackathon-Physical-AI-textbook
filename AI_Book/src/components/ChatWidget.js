/**
 * ChatWidget Component for Docusaurus
 *
 * A React-based chat interface that integrates with the FastAPI RAG backend
 * to provide AI-powered documentation assistance.
 *
 * USAGE IN DOCUSAURUS:
 *
 * 1. In a standard React page (e.g., src/pages/chat.js):
 *    ```jsx
 *    import ChatWidget from '@site/src/components/ChatWidget';
 *
 *    export default function ChatPage() {
 *      return <ChatWidget />;
 *    }
 *    ```
 *
 * 2. In an MDX documentation page (e.g., docs/chat.mdx):
 *    ```mdx
 *    import ChatWidget from '@site/src/components/ChatWidget';
 *
 *    # Documentation Chat Assistant
 *
 *    Ask questions about the documentation:
 *
 *    <ChatWidget />
 *    ```
 *
 * REQUIREMENTS:
 * - FastAPI backend must be running at http://localhost:8000
 * - Backend must have POST /chat endpoint accepting { query: string }
 * - Backend must return { response: string, sources_count?: number }
 *
 * FEATURES:
 * - Real-time chat interface with message history
 * - Loading states during API calls
 * - Error handling with user-friendly messages
 * - Enter key support for sending messages
 * - Auto-scroll to latest message
 * - Responsive design
 */

import React, { useState, useRef, useEffect } from 'react';

const ChatWidget = () => {
  // State management
  const [messages, setMessages] = useState([
    {
      sender: 'bot',
      text: 'Hello! I\'m your documentation assistant. Ask me anything about the documentation.',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Ref for auto-scrolling to latest message
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  /**
   * Send message to the FastAPI backend
   */
  const sendMessage = async () => {
    // Validate input
    if (!inputValue.trim()) {
      return;
    }

    const userMessage = {
      sender: 'user',
      text: inputValue.trim(),
      timestamp: new Date()
    };

    // Add user message to chat history
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Make POST request to FastAPI backend
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage.text }),
      });

      // Handle HTTP errors
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.detail ||
          `Server error (${response.status}): ${response.statusText}`
        );
      }

      // Parse response
      const data = await response.json();

      // Add bot response to chat history
      const botMessage = {
        sender: 'bot',
        text: data.response,
        sourcesCount: data.sources_count,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
      setIsLoading(false);

    } catch (err) {
      console.error('Chat error:', err);

      // Determine user-friendly error message
      let errorMessage = 'Sorry, I encountered an error. ';

      if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
        errorMessage += 'Unable to connect to the chat service. Please ensure the backend is running at http://localhost:8000.';
      } else if (err.message.includes('not found')) {
        errorMessage += 'The documentation hasn\'t been indexed yet. Please run the ingestion script first.';
      } else {
        errorMessage += err.message;
      }

      const errorBotMessage = {
        sender: 'bot',
        text: errorMessage,
        isError: true,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorBotMessage]);
      setError(err.message);
      setIsLoading(false);
    }
  };

  /**
   * Handle Enter key press in input field
   */
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div style={styles.container}>
      {/* Header */}
      <div style={styles.header}>
        <h3 style={styles.headerTitle}>Documentation Assistant</h3>
        <p style={styles.headerSubtitle}>
          Ask questions about the documentation
        </p>
      </div>

      {/* Messages Container */}
      <div style={styles.messagesContainer}>
        {messages.map((message, index) => (
          <div
            key={index}
            style={{
              ...styles.messageWrapper,
              justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start',
            }}
          >
            <div
              style={{
                ...styles.messageBubble,
                ...(message.sender === 'user' ? styles.userBubble : styles.botBubble),
                ...(message.isError ? styles.errorBubble : {}),
              }}
            >
              <div style={styles.messageText}>{message.text}</div>

              {/* Show sources count if available */}
              {message.sourcesCount && (
                <div style={styles.sourcesInfo}>
                  📚 {message.sourcesCount} source{message.sourcesCount !== 1 ? 's' : ''} used
                </div>
              )}

              {/* Timestamp */}
              <div style={styles.timestamp}>
                {message.timestamp.toLocaleTimeString([], {
                  hour: '2-digit',
                  minute: '2-digit'
                })}
              </div>
            </div>
          </div>
        ))}

        {/* Loading indicator */}
        {isLoading && (
          <div style={{ ...styles.messageWrapper, justifyContent: 'flex-start' }}>
            <div style={{ ...styles.messageBubble, ...styles.botBubble }}>
              <div style={styles.loadingDots}>
                <span>●</span>
                <span>●</span>
                <span>●</span>
              </div>
            </div>
          </div>
        )}

        {/* Auto-scroll anchor */}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Container */}
      <div style={styles.inputContainer}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Type your question..."
          disabled={isLoading}
          style={{
            ...styles.input,
            ...(isLoading ? styles.inputDisabled : {}),
          }}
        />
        <button
          onClick={sendMessage}
          disabled={isLoading || !inputValue.trim()}
          style={{
            ...styles.sendButton,
            ...((isLoading || !inputValue.trim()) ? styles.sendButtonDisabled : {}),
          }}
        >
          {isLoading ? '⏳' : '📤'}
        </button>
      </div>

      {/* Error indicator */}
      {error && (
        <div style={styles.errorIndicator}>
          ⚠️ Connection issue - check backend status
        </div>
      )}
    </div>
  );
};

// Inline styles for the component
const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    width: '100%',
    maxWidth: '800px',
    height: '600px',
    margin: '0 auto',
    border: '1px solid #e0e0e0',
    borderRadius: '12px',
    boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)',
    backgroundColor: '#ffffff',
    fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif',
    overflow: 'hidden',
  },
  header: {
    padding: '16px 20px',
    backgroundColor: '#2e7d32',
    color: 'white',
    borderBottom: '1px solid #1b5e20',
  },
  headerTitle: {
    margin: 0,
    fontSize: '18px',
    fontWeight: '600',
  },
  headerSubtitle: {
    margin: '4px 0 0 0',
    fontSize: '13px',
    opacity: 0.9,
  },
  messagesContainer: {
    flex: 1,
    overflowY: 'auto',
    padding: '16px',
    backgroundColor: '#f5f5f5',
  },
  messageWrapper: {
    display: 'flex',
    marginBottom: '12px',
  },
  messageBubble: {
    maxWidth: '75%',
    padding: '10px 14px',
    borderRadius: '12px',
    wordWrap: 'break-word',
    position: 'relative',
  },
  userBubble: {
    backgroundColor: '#1976d2',
    color: 'white',
    borderBottomRightRadius: '4px',
  },
  botBubble: {
    backgroundColor: '#ffffff',
    color: '#333333',
    border: '1px solid #e0e0e0',
    borderBottomLeftRadius: '4px',
  },
  errorBubble: {
    backgroundColor: '#ffebee',
    borderColor: '#ef5350',
    color: '#c62828',
  },
  messageText: {
    fontSize: '14px',
    lineHeight: '1.5',
    marginBottom: '4px',
  },
  sourcesInfo: {
    fontSize: '11px',
    opacity: 0.7,
    marginTop: '6px',
    fontStyle: 'italic',
  },
  timestamp: {
    fontSize: '10px',
    opacity: 0.6,
    marginTop: '4px',
    textAlign: 'right',
  },
  loadingDots: {
    display: 'flex',
    gap: '4px',
    fontSize: '18px',
    animation: 'pulse 1.5s ease-in-out infinite',
  },
  inputContainer: {
    display: 'flex',
    padding: '16px',
    backgroundColor: '#ffffff',
    borderTop: '1px solid #e0e0e0',
    gap: '8px',
  },
  input: {
    flex: 1,
    padding: '10px 14px',
    fontSize: '14px',
    border: '1px solid #d0d0d0',
    borderRadius: '8px',
    outline: 'none',
    transition: 'border-color 0.2s',
  },
  inputDisabled: {
    backgroundColor: '#f5f5f5',
    cursor: 'not-allowed',
  },
  sendButton: {
    padding: '10px 20px',
    fontSize: '18px',
    backgroundColor: '#2e7d32',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    transition: 'background-color 0.2s',
    minWidth: '60px',
  },
  sendButtonDisabled: {
    backgroundColor: '#cccccc',
    cursor: 'not-allowed',
  },
  errorIndicator: {
    padding: '8px',
    backgroundColor: '#fff3cd',
    color: '#856404',
    fontSize: '12px',
    textAlign: 'center',
    borderTop: '1px solid #ffeaa7',
  },
};

export default ChatWidget;
