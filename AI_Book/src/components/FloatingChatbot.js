import React, { useState, useEffect } from 'react';

const FloatingChatbot = () => {
  const [mounted, setMounted] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  // Prevent SSR hydration crash
  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) return null;

  const toggleChat = () => {
    setIsOpen(prev => !prev);
  };

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userText = input;
    setInput('');
    setLoading(true);

    setMessages(prev => [
      ...prev,
      { sender: 'user', text: userText }
    ]);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userText }),
      });

      if (!response.ok) {
        const text = await response.text();
        throw new Error(text);
      }

      const data = await response.json();

      setMessages(prev => [
        ...prev,
        { sender: 'bot', text: data.response }
      ]);

    } catch (err) {
      console.error('Chat error:', err);

      setMessages(prev => [
        ...prev,
        {
          sender: 'bot',
          text: 'Sorry, the chat service is unavailable right now.',
        }
      ]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={styles.container}>
      <button style={styles.floatingButton} onClick={toggleChat}>
        {isOpen ? '✖' : '💬'}
      </button>

      {isOpen && (
        <div style={styles.chatWindow}>
          <div style={styles.header}>
            Documentation Assistant
          </div>

          <div style={styles.messagesContainer}>
            {messages.map((msg, index) => (
              <div
                key={index}
                style={msg.sender === 'user'
                  ? styles.userMessage
                  : styles.botMessage
                }
              >
                {msg.text}
              </div>
            ))}

            {loading && (
              <div style={styles.botMessage}>
                Typing…
              </div>
            )}
          </div>

          <div style={styles.inputContainer}>
            <input
              style={styles.input}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && sendMessage()}
              placeholder="Ask something..."
            />
            <button style={styles.sendButton} onClick={sendMessage}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

const styles = {
  container: {
    position: 'fixed',
    bottom: '20px',
    right: '20px',
    zIndex: 9999,
  },

  floatingButton: {
    width: '56px',
    height: '56px',
    borderRadius: '50%',
    border: 'none',
    backgroundColor: '#0a2540',
    color: '#fff',
    fontSize: '22px',
    cursor: 'pointer',
  },

  chatWindow: {
    position: 'absolute',
    bottom: '70px',
    right: '0',
    width: '320px',
    height: '420px',
    backgroundColor: '#ffffff',
    borderRadius: '12px',
    boxShadow: '0 10px 30px rgba(0,0,0,0.15)',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
    fontFamily: 'Inter, sans-serif',
  },

  header: {
    padding: '12px',
    backgroundColor: '#0a2540',
    color: 'white',
    fontWeight: '600',
    textAlign: 'center',
  },

  messagesContainer: {
    flexGrow: 1,
    padding: '12px',
    overflowY: 'auto',
    backgroundColor: '#f7f7f7',
  },

  userMessage: {
    backgroundColor: '#d6ebff',
    padding: '8px 10px',
    borderRadius: '8px',
    marginBottom: '8px',
    alignSelf: 'flex-end',
    maxWidth: '85%',
  },

  botMessage: {
    backgroundColor: '#eeeeee',
    padding: '8px 10px',
    borderRadius: '8px',
    marginBottom: '8px',
    alignSelf: 'flex-start',
    maxWidth: '85%',
  },

  inputContainer: {
    display: 'flex',
    padding: '10px',
    borderTop: '1px solid #ddd',
  },

  input: {
    flexGrow: 1,
    padding: '8px',
    borderRadius: '6px',
    border: '1px solid #ccc',
    outline: 'none',
  },

  sendButton: {
    marginLeft: '8px',
    padding: '8px 14px',
    backgroundColor: '#0a2540',
    color: 'white',
    border: 'none',
    borderRadius: '6px',
    cursor: 'pointer',
  },
};

export default FloatingChatbot;
