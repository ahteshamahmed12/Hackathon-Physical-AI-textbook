# Urdu Translation Feature - Implementation Guide

## Overview

The Urdu Translation feature provides on-the-fly translation of documentation pages from English to Urdu using AI-powered translation via the Claude API. Users can click the "🌐 اردو" button in the navbar to translate the current page.

---

## Architecture

### Components

1. **Backend API Endpoint** (`app.py`)
   - POST `/translate` endpoint
   - Uses Anthropic Claude 3.5 Sonnet for translation
   - Preserves markdown formatting
   - Handles technical terminology appropriately

2. **Frontend React Component** (`src/components/UrduTranslator.js`)
   - Translation button UI
   - Modal display for translated content
   - Error handling and loading states
   - Right-to-left (RTL) text support for Urdu

3. **Theme Integration** (`src/theme/Root.js`)
   - Injects translator component into navbar
   - Handles component lifecycle
   - Ensures proper mounting on route changes

4. **Styling** (`src/css/custom.css`)
   - Urdu button styles
   - Modal overlay styles
   - Noto Nastaliq Urdu font import
   - Spinner animations

---

## Installation & Setup

### Prerequisites

- FastAPI backend running at `http://localhost:8000`
- Anthropic API key configured in `.env` file
- Docusaurus site running at `http://localhost:3000`

### Environment Variables

Ensure your `.env` file includes:

```env
ANTHROPIC_API_KEY=your_anthropic_api_key_here
```

### Backend Setup

The translation endpoint is already integrated into `app.py`. No additional setup required.

### Frontend Setup

All required files are created:
- `src/components/UrduTranslator.js` - Main component
- `src/theme/Root.js` - Theme wrapper for injection
- `src/css/custom.css` - Styling updates
- `docusaurus.config.ts` - Navbar configuration

---

## Usage

### For End Users

1. **Navigate to any documentation page**
   - Open any page in the Physical AI Book

2. **Click the Urdu button**
   - Look for "🌐 اردو" button in the top-right navbar
   - Click to start translation

3. **View translated content**
   - Translation appears in a modal overlay
   - Content is displayed in right-to-left (RTL) format
   - Original formatting is preserved

4. **Close or retranslate**
   - Click "Close | بند کریں" to return to original
   - Click "🔄 Retranslate" to translate again
   - Click outside modal to close

### Translation Features

- ✅ **Preserves markdown formatting** (headings, lists, code blocks, links)
- ✅ **Does NOT translate code** (code snippets remain in English)
- ✅ **Technical terms** handled appropriately (kept in English with Urdu explanations)
- ✅ **Right-to-left display** for proper Urdu reading
- ✅ **Beautiful Noto Nastaliq Urdu font** for authentic typography

---

## API Reference

### POST /translate

Translates markdown documentation content to Urdu.

**Endpoint:** `http://localhost:8000/translate`

**Request Body:**
```json
{
  "content": "string (1-50000 chars)",
  "target_language": "Urdu" (optional, defaults to "Urdu")
}
```

**Response:**
```json
{
  "translated_content": "string",
  "original_length": 1234,
  "translated_length": 2345
}
```

**Error Responses:**

- `400 Bad Request` - Invalid content or empty input
- `500 Internal Server Error` - Translation generation failed
- `503 Service Unavailable` - Claude API unavailable

**Example Usage:**

```javascript
const response = await fetch('http://localhost:8000/translate', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    content: '# Introduction to ROS2\n\nROS2 is a robotics middleware...',
    target_language: 'Urdu'
  }),
});

const data = await response.json();
console.log(data.translated_content);
```

---

## Translation Quality Guidelines

The Claude API is instructed to:

1. **Translate ALL text content** including headings, paragraphs, lists, tables
2. **PRESERVE markdown formatting** exactly as it appears
3. **NOT translate:**
   - Code snippets in backticks or code blocks
   - URLs and hyperlinks
   - Technical commands and syntax
   - File paths and system commands
4. **Maintain document structure** completely
5. **Use technical Urdu** appropriate for engineers and students
6. **Keep technical terms in English** when no good Urdu equivalent exists
7. **Provide natural, professional translations**

---

## Troubleshooting

### Common Issues

#### 1. Translation Button Not Appearing

**Problem:** The "🌐 اردو" button is not visible in the navbar.

**Solutions:**
- Ensure `src/theme/Root.js` exists
- Verify `docusaurus.config.ts` has the `urdu-translator-root` div
- Clear Docusaurus cache: `yarn clear`
- Restart development server: `yarn start`

#### 2. "Unable to connect to translation service"

**Problem:** Error when clicking translate button.

**Solutions:**
- Ensure FastAPI backend is running: `python app.py`
- Check backend is accessible at `http://localhost:8000`
- Verify `ANTHROPIC_API_KEY` is set in `.env` file
- Test backend health: `curl http://localhost:8000/health`

#### 3. Translation Takes Too Long

**Problem:** Translation appears to hang or take excessive time.

**Causes & Solutions:**
- **Large page content:** Claude has 8192 token limit; very long pages may fail
- **API rate limiting:** Check Anthropic API usage limits
- **Network latency:** Check internet connection

**Workaround:** Translate shorter sections at a time

#### 4. Poor Translation Quality

**Problem:** Translation doesn't make sense or has errors.

**Solutions:**
- Check if source content is clear and well-structured
- Technical jargon may need manual review
- Consider providing feedback to improve prompts
- Retranslate using the "🔄 Retranslate" button

#### 5. Modal Not Displaying Properly

**Problem:** Translation modal layout is broken or content is cut off.

**Solutions:**
- Check browser console for JavaScript errors
- Ensure CSS is loaded: inspect `custom.css`
- Try different browser (Chrome, Firefox, Edge)
- Clear browser cache and reload

#### 6. Urdu Font Not Rendering

**Problem:** Urdu text appears as boxes or incorrect characters.

**Solutions:**
- Verify internet connection (font loads from Google Fonts)
- Check browser supports Urdu font rendering
- Install "Noto Nastaliq Urdu" font locally as fallback
- Check browser console for font loading errors

---

## Testing

### Manual Testing Steps

1. **Start backend:**
   ```bash
   cd AI_Book
   python app.py
   ```

2. **Start frontend:**
   ```bash
   cd AI_Book
   yarn start
   ```

3. **Test translation:**
   - Navigate to `http://localhost:3000`
   - Click any documentation page
   - Click "🌐 اردو" button
   - Verify translation appears in modal
   - Check Urdu text is RTL and uses proper font
   - Test close and retranslate buttons

4. **Test error handling:**
   - Stop backend
   - Try translating - should show error
   - Restart backend
   - Try again - should work

### API Testing

Test the translation endpoint directly:

```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Hello World\n\nThis is a test.",
    "target_language": "Urdu"
  }'
```

Expected response:
```json
{
  "translated_content": "# ہیلو ورلڈ\n\nیہ ایک ٹیسٹ ہے۔",
  "original_length": 32,
  "translated_length": 45
}
```

---

## Performance Considerations

### Response Times

- **Short pages (< 1000 chars):** 2-5 seconds
- **Medium pages (1000-5000 chars):** 5-10 seconds
- **Long pages (5000-10000 chars):** 10-20 seconds
- **Very long pages (> 10000 chars):** May fail or timeout

### Optimization Tips

1. **Caching:** Consider caching translations for frequently accessed pages
2. **Pre-generation:** Pre-translate popular pages and serve statically
3. **Rate Limiting:** Implement rate limiting to prevent API quota exhaustion
4. **Chunking:** For very long documents, split into chunks

---

## Future Enhancements

### Planned Features

1. **Translation Caching**
   - Store translations in browser localStorage
   - Reduce API calls for repeated translations
   - Cache expiration strategy

2. **Pre-generated Translations**
   - Build-time translation of static pages
   - Serve Urdu versions as static files
   - Docusaurus i18n integration

3. **Multiple Languages**
   - Add support for other languages (Arabic, Farsi, Hindi)
   - Language selector dropdown
   - Configurable language options

4. **Improved UX**
   - Side-by-side view (English and Urdu)
   - Inline translation (without modal)
   - Translation progress indicator
   - Keyboard shortcuts (ESC to close)

5. **Quality Improvements**
   - User feedback mechanism
   - Translation rating system
   - Manual corrections and overrides
   - Glossary for technical terms

---

## Code Structure

```
AI_Book/
├── app.py                           # FastAPI backend
│   └── POST /translate              # Translation endpoint
├── src/
│   ├── components/
│   │   └── UrduTranslator.js       # Main translation component
│   ├── theme/
│   │   └── Root.js                 # Theme wrapper for injection
│   └── css/
│       └── custom.css              # Urdu button and modal styles
└── docusaurus.config.ts            # Navbar configuration
```

---

## Credits

- **Translation Engine:** Anthropic Claude 3.5 Sonnet
- **Urdu Font:** Noto Nastaliq Urdu (Google Fonts)
- **Framework:** Docusaurus + React
- **Backend:** FastAPI + Python

---

## Support

For issues or questions:

1. Check this guide first
2. Review backend logs: `python app.py`
3. Check browser console for frontend errors
4. Test API endpoint directly with curl
5. Verify environment variables are set

---

## License

This feature is part of the Physical AI Book project.

---

**Happy Translating! | خوش ترجمہ کریں!** 🌐
