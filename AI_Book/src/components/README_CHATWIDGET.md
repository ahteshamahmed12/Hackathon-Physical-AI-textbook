# ChatWidget Component

A React-based chat interface component for integrating RAG-powered documentation assistance into your Docusaurus website.

## Features

- 💬 **Real-time Chat Interface** - Interactive conversation with AI assistant
- 🔄 **Loading States** - Visual feedback during API calls
- ⚠️ **Error Handling** - User-friendly error messages
- ⌨️ **Keyboard Support** - Press Enter to send messages
- 📜 **Auto-scroll** - Automatically scrolls to latest message
- 📊 **Source Citations** - Shows number of documentation sources used
- 🎨 **Responsive Design** - Works on desktop and mobile devices
- ⏰ **Timestamps** - Shows when each message was sent

## Prerequisites

1. **FastAPI backend running** at `http://localhost:8000`
2. **Documentation indexed** - Run `python ingest.py` first
3. **Backend endpoint** - POST `/chat` accepting `{ query: string }`

## Usage

### In a React Page (e.g., `src/pages/chat.js`)

```jsx
import React from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function ChatPage() {
  return (
    <Layout
      title="Documentation Chat"
      description="Ask questions about the documentation"
    >
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: 'calc(100vh - 60px)',
        padding: '20px',
        backgroundColor: '#f5f5f5'
      }}>
        <ChatWidget />
      </div>
    </Layout>
  );
}
```

### In an MDX Documentation Page (e.g., `docs/chat.mdx`)

```mdx
---
title: Documentation Assistant
description: Ask questions about the documentation
---

import ChatWidget from '@site/src/components/ChatWidget';

# Documentation Chat Assistant

Have a question about the documentation? Ask our AI assistant!

<ChatWidget />
```

### In the Homepage (e.g., `src/pages/index.js`)

```jsx
import React from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

export default function Home() {
  return (
    <Layout
      title="Welcome"
      description="Documentation with AI Assistant"
    >
      <main>
        <HomepageFeatures />

        <section style={{ padding: '40px 20px', backgroundColor: '#f9f9f9' }}>
          <div style={{ maxWidth: '1200px', margin: '0 auto' }}>
            <h2 style={{ textAlign: 'center', marginBottom: '30px' }}>
              Ask Our Documentation Assistant
            </h2>
            <ChatWidget />
          </div>
        </section>
      </main>
    </Layout>
  );
}
```

## Component API

### Props

The ChatWidget component currently doesn't accept any props, but can be extended to support:

```jsx
// Potential future props:
<ChatWidget
  backendUrl="http://localhost:8000"  // Custom backend URL
  theme="dark"                         // Theme customization
  placeholder="Ask a question..."      // Custom placeholder text
  height="500px"                       // Custom height
/>
```

### State Management

The component uses React hooks for state management:

- `messages` - Array of chat messages (user + bot)
- `inputValue` - Current text input value
- `isLoading` - Boolean indicating API call in progress
- `error` - Error state for connection issues

### Message Structure

```javascript
{
  sender: 'user' | 'bot',
  text: string,
  timestamp: Date,
  sourcesCount?: number,  // Number of doc sources used (bot only)
  isError?: boolean       // Error message flag (bot only)
}
```

## Styling

The component uses inline styles for simplicity and Docusaurus compatibility. Key style properties:

```javascript
{
  container: {
    width: '100%',
    maxWidth: '800px',
    height: '600px',
    // ... more styles
  }
}
```

### Customizing Styles

To customize the appearance, modify the `styles` object at the bottom of `ChatWidget.js`:

```javascript
const styles = {
  container: {
    // Change widget dimensions
    maxWidth: '1000px',  // Make wider
    height: '700px',     // Make taller
  },
  header: {
    // Change header colors
    backgroundColor: '#1976d2',  // Blue theme
  },
  userBubble: {
    // Change user message color
    backgroundColor: '#4caf50',  // Green
  },
  // ... more customizations
};
```

## Error Handling

The component handles various error scenarios:

### Network Errors
```
Unable to connect to the chat service.
Please ensure the backend is running at http://localhost:8000.
```

### Collection Not Found
```
The documentation hasn't been indexed yet.
Please run the ingestion script first.
```

### Server Errors
```
Server error (500): Internal Server Error
```

### Generic Errors
```
Sorry, I encountered an error. [Error details]
```

## Backend Integration

The component expects the backend to:

1. **Accept POST requests** at `http://localhost:8000/chat`
2. **Request format:**
   ```json
   {
     "query": "What GPU do I need?"
   }
   ```
3. **Response format:**
   ```json
   {
     "response": "Based on the documentation, you need an RTX 4070 Ti...",
     "sources_count": 3
   }
   ```

## Keyboard Shortcuts

- **Enter** - Send message
- **Shift+Enter** - (Reserved for multi-line input in future)

## Accessibility

The component includes basic accessibility features:

- Semantic HTML structure
- Disabled state management
- Clear visual feedback for all interactions
- Timestamps for screen readers

## Browser Compatibility

- ✅ Chrome/Edge (latest)
- ✅ Firefox (latest)
- ✅ Safari (latest)
- ✅ Mobile browsers

## Troubleshooting

### Widget not displaying

**Issue:** Component imports but doesn't render

**Solution:**
1. Check console for errors
2. Verify React version compatibility
3. Ensure Docusaurus is running: `npm start`

### "Failed to fetch" error

**Issue:** Cannot connect to backend

**Solution:**
1. Ensure backend is running: `python app.py`
2. Check backend is on port 8000: `http://localhost:8000/health`
3. Verify CORS is configured for `http://localhost:3000`

### "Collection not found" error

**Issue:** Qdrant collection doesn't exist

**Solution:**
1. Run ingestion script: `python ingest.py`
2. Verify Qdrant credentials in `.env`
3. Check collection was created successfully

### Messages not scrolling

**Issue:** Auto-scroll not working

**Solution:**
- This is usually a React rendering issue
- Refresh the page
- Check browser console for errors

### Styling looks broken

**Issue:** Styles not applying correctly

**Solution:**
1. Check for CSS conflicts with Docusaurus theme
2. Verify no custom CSS is overriding component styles
3. Try clearing browser cache

## Development

### Testing Locally

1. **Start the backend:**
   ```bash
   python app.py
   ```

2. **Start Docusaurus:**
   ```bash
   npm start
   ```

3. **Open browser:**
   ```
   http://localhost:3000
   ```

4. **Navigate to page with ChatWidget**

### Adding Debug Logging

Add console logs to debug issues:

```javascript
const sendMessage = async () => {
  console.log('Sending message:', inputValue);

  try {
    const response = await fetch('http://localhost:8000/chat', {
      // ...
    });
    console.log('Response:', response);
    // ...
  } catch (err) {
    console.error('Error details:', err);
  }
};
```

## Future Enhancements

Potential improvements:

- [ ] Conversation history persistence (localStorage/sessionStorage)
- [ ] Multi-line message support
- [ ] File/image upload support
- [ ] Code syntax highlighting in responses
- [ ] Markdown rendering for bot responses
- [ ] Export chat history feature
- [ ] Dark mode support
- [ ] Custom themes via props
- [ ] Typing indicators
- [ ] Message reactions/feedback
- [ ] Voice input support

## Performance Optimization

For better performance:

1. **Lazy Loading:**
   ```jsx
   import React, { lazy, Suspense } from 'react';

   const ChatWidget = lazy(() => import('@site/src/components/ChatWidget'));

   export default function ChatPage() {
     return (
       <Suspense fallback={<div>Loading chat...</div>}>
         <ChatWidget />
       </Suspense>
     );
   }
   ```

2. **Memoization:**
   ```jsx
   const MessageBubble = React.memo(({ message }) => {
     // Message rendering logic
   });
   ```

## License

This component is part of the RAG Documentation Chatbot project.
