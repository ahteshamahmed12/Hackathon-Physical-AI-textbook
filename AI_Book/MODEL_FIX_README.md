# Claude Model Name Fix

## Issue

Translation and chat endpoints were failing with 404 error due to incorrect Claude model name.

**Error Message:**
```
Error code: 404 - {'type': 'error', 'error': {'type': 'not_found_error', 'message': 'model: claude-3-5-sonnet-20241022'}}
```

## Root Cause

The model name `claude-3-5-sonnet-20241022` doesn't exist or isn't available in your Anthropic account.

## Solution

Updated to use the **stable Claude 3.5 Sonnet model**: `claude-3-5-sonnet-20240620`

## Changes Made

### File: `app.py`

**Line 250 (Chat Endpoint):**
```python
# Before:
model="claude-3-5-sonnet-20241022",

# After:
model="claude-3-5-sonnet-20240620",  # Stable version
```

**Line 376 (Translation Endpoint):**
```python
# Before:
model="claude-3-5-sonnet-20241022",

# After:
model="claude-3-5-sonnet-20240620",  # Stable version
```

## Available Claude Models

### Claude 3.5 Sonnet (Recommended)
- **Model Name:** `claude-3-5-sonnet-20240620`
- **Best for:** Complex tasks, translation, technical content
- **Context:** 200K tokens
- **Max Output:** 8K tokens

### Claude 3 Opus (Most Capable)
- **Model Name:** `claude-3-opus-20240229`
- **Best for:** Highest quality, most difficult tasks
- **Context:** 200K tokens
- **Max Output:** 4K tokens

### Claude 3 Sonnet (Balanced)
- **Model Name:** `claude-3-sonnet-20240229`
- **Best for:** Balance of speed and capability
- **Context:** 200K tokens
- **Max Output:** 4K tokens

### Claude 3 Haiku (Fast)
- **Model Name:** `claude-3-haiku-20240307`
- **Best for:** Speed, simple tasks
- **Context:** 200K tokens
- **Max Output:** 4K tokens

## Testing

### 1. Restart Backend
```bash
# Stop current process (Ctrl+C)
python app.py
```

### 2. Test Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

### 3. Test Translation Endpoint
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"content": "# Hello World\n\nThis is a test."}'
```

### 4. Test Urdu Button
- Click "🌐 اردو" button in navbar
- Should now work without 404 errors
- Check browser console for success

## Expected Response

### Success (200 OK):
```json
{
  "translated_content": "# ہیلو ورلڈ\n\nیہ ایک ٹیسٹ ہے۔",
  "original_length": 32,
  "translated_length": 45
}
```

### Previous Error (404 Not Found):
```json
{
  "type": "error",
  "error": {
    "type": "not_found_error",
    "message": "model: claude-3-5-sonnet-20241022"
  }
}
```

## Model Selection Guide

### For Translation (Current Use Case):
✅ **Claude 3.5 Sonnet (20240620)** - Best choice
- High quality translations
- Good with technical content
- Balanced speed/quality
- 8K max output for long documents

### Alternative Options:

**If you need higher quality:**
```python
model="claude-3-opus-20240229"
```

**If you need faster responses:**
```python
model="claude-3-haiku-20240307"
```

**If model still not found:**
```python
model="claude-3-sonnet-20240229"  # Older stable version
```

## Checking Available Models

To see which models are available in your account:

```python
import anthropic

client = anthropic.Anthropic(api_key="your-api-key")

# List available models (check Anthropic documentation)
# Or test with a simple API call
response = client.messages.create(
    model="claude-3-5-sonnet-20240620",
    max_tokens=10,
    messages=[{"role": "user", "content": "Hi"}]
)
print(response)
```

## Troubleshooting

### Still getting 404 error?

1. **Check API Key:**
   - Verify `ANTHROPIC_API_KEY` is set correctly in `.env`
   - Ensure API key is active and valid

2. **Try older model:**
   ```python
   model="claude-3-sonnet-20240229"
   ```

3. **Check Anthropic Account:**
   - Visit https://console.anthropic.com/
   - Verify account is active
   - Check usage limits
   - Confirm API access

4. **Model Availability:**
   - Some models may not be available in all regions
   - Check Anthropic's model availability docs
   - Contact Anthropic support if issues persist

### Rate Limiting?

If you get rate limit errors:
- Add delays between requests
- Implement exponential backoff
- Check your usage tier

## API Key Verification

Test your API key works:

```bash
curl https://api.anthropic.com/v1/messages \
  -H "x-api-key: $ANTHROPIC_API_KEY" \
  -H "anthropic-version: 2023-06-01" \
  -H "content-type: application/json" \
  -d '{
    "model": "claude-3-5-sonnet-20240620",
    "max_tokens": 10,
    "messages": [{"role": "user", "content": "Hi"}]
  }'
```

Expected response:
```json
{
  "id": "msg_...",
  "type": "message",
  "role": "assistant",
  "content": [{"type": "text", "text": "Hello!"}],
  "model": "claude-3-5-sonnet-20240620",
  "stop_reason": "end_turn",
  "usage": {"input_tokens": 8, "output_tokens": 5}
}
```

## Environment Variables

Ensure your `.env` file has:

```env
ANTHROPIC_API_KEY=sk-ant-api03-...
```

## Summary

✅ Fixed model name from `claude-3-5-sonnet-20241022` to `claude-3-5-sonnet-20240620`
✅ Updated both chat and translation endpoints
✅ Using stable, widely-available model version
✅ Translation and chat should now work properly

**Restart backend and test!**
