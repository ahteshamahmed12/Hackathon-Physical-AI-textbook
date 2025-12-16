import React, { useState } from 'react';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (input.trim() === '') return;

    const userMessage = { sender: 'user', text: input };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');

    try {
      const response = await fetch('https://vercel.com/ahtesham-ahmeds-projects/hackathon-physical-ai-textbook', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: input }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage = { sender: 'bot', text: data.response || 'Error: No response from API.' };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error fetching chat response:', error);
      const errorMessage = { sender: 'bot', text: 'Sorry, something went wrong. Please try again later.' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    }
  };

  return (
    <div style={styles.container}>
      <button onClick={toggleChat} style={styles.floatingButton}>
        {isOpen ? 'X' : 'Chat'}
      </button>

      {isOpen && (
        <div style={styles.chatWindow}>
          <div style={styles.messagesContainer}>
            {messages.map((msg, index) => (
              <div key={index} style={msg.sender === 'user' ? styles.userMessage : styles.botMessage}>
                {msg.text}
              </div>
            ))}
          </div>
          <div style={styles.inputContainer}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => {
                if (e.key === 'Enter') {
                  sendMessage();
                }
              }}
              style={styles.input}
              placeholder="Type your message..."
            />
            <button onClick={sendMessage} style={styles.sendButton}>
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
    position: 'absolute',
    bottom: '20px',
    right: '20px',
    zIndex: 1000,
  },
  floatingButton: {
    backgroundColor: '#0a2540',
    color: 'white',
    border: 'none',
    borderRadius: '50%',
    width: '60px',
    height: '60px',
    fontSize: '18px',
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
  chatWindow: {
    position: 'absolute',
    bottom: '80px',
    right: '0',
    width: '300px',
    height: '400px',
    backgroundColor: 'white',
    borderRadius: '10px',
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
    fontFamily: '"Inter", "Segoe UI", sans-serif',
  messagesContainer: {
    flexGrow: 1,
    padding: '10px',
    overflowY: 'auto',
    borderBottom: '1px solid #eee',
  },
  userMessage: {
    backgroundColor: '#e6f7ff',
    padding: '8px',
    borderRadius: '5px',
    marginBottom: '5px',
    alignSelf: 'flex-end',
    textAlign: 'right',
  },
  botMessage: {
    backgroundColor: '#f0f0f0',
    padding: '8px',
    borderRadius: '5px',
    marginBottom: '5px',
    alignSelf: 'flex-start',
    textAlign: 'left',
  },
  inputContainer: {
    display: 'flex',
    padding: '10px',
    borderTop: '1px solid #eee',
  },
  input: {
    flexGrow: 1,
    border: '1px solid #ddd',
    borderRadius: '5px',
    padding: '8px',
    marginRight: '10px',
  },
  sendButton: {
    backgroundColor: '#0a2540',
    color: 'white',
    border: 'none',
    borderRadius: '5px',
    padding: '8px 15px',
    cursor: 'pointer',
  },
}};

export default FloatingChatbot;