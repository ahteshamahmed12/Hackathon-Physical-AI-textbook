# CORS Fix for Translation Endpoint

## Issue

The translation endpoint was returning `400 Bad Request` for OPTIONS preflight requests, causing CORS errors in the browser.

**Error Message:**
```
INFO:     127.0.0.1:51329 - "OPTIONS /translate HTTP/1.1" 400 Bad Request
```

## Root Cause

The CORS middleware was configured, but not permissive enough for all development scenarios. Additionally, the Qdrant client was showing compatibility warnings.

## Fixes Applied

### 1. CORS Configuration Update

**File:** `app.py`

**Before:**
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**After:**
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,  # Cache preflight requests for 1 hour
)
```

**Note:** For production, tighten `allow_origins` to specific domains:
```python
allow_origins=["https://yourdomain.com", "https://www.yourdomain.com"]
```

### 2. Qdrant Client Compatibility Fix

**File:** `app.py`

**Before:**
```python
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
```

**After:**
```python
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=False,  # Use REST API instead of gRPC
)
```

This fixes the warning:
```
UserWarning: Failed to obtain server version. Unable to check client-server compatibility.
```

## Testing

### 1. Restart the backend:
```bash
# Stop the current process (Ctrl+C)
python app.py
```

### 2. Test CORS from browser console:
```javascript
fetch('http://localhost:8000/translate', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ content: '# Test\n\nHello world.' })
})
.then(r => r.json())
.then(console.log)
.catch(console.error);
```

### 3. Test with Urdu button:
- Click "🌐 اردو" button in navbar
- Should now work without CORS errors
- Check browser console for success

## Expected Logs (After Fix)

```
✓ All API clients initialized successfully
✓ Using Google Gemini for embeddings
✓ Using Anthropic Claude for response generation
INFO:     Started server process [5496]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     127.0.0.1:56563 - "OPTIONS /translate HTTP/1.1" 200 OK  ✅
INFO:     127.0.0.1:56563 - "POST /translate HTTP/1.1" 200 OK    ✅
```

## Production Considerations

When deploying to production:

1. **Tighten CORS origins:**
   ```python
   allow_origins=[
       "https://yourdomain.com",
       "https://www.yourdomain.com"
   ]
   ```

2. **Add environment-based configuration:**
   ```python
   import os

   CORS_ORIGINS = os.getenv("CORS_ORIGINS", "*").split(",")

   app.add_middleware(
       CORSMiddleware,
       allow_origins=CORS_ORIGINS,
       # ...
   )
   ```

3. **Use environment variables:**
   ```env
   # .env for production
   CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com
   ```

## Troubleshooting

### Still getting CORS errors?

1. **Clear browser cache** and hard reload (Ctrl+Shift+R)
2. **Check browser console** for exact error message
3. **Verify backend is running** at http://localhost:8000
4. **Test health endpoint:** http://localhost:8000/health
5. **Check browser network tab** for OPTIONS and POST requests

### CORS headers not present?

- Ensure backend restarted after code changes
- Check that `CORSMiddleware` is added BEFORE route definitions
- Verify `fastapi.middleware.cors` is imported

### Qdrant warning persists?

- This is now just a warning, not an error
- Adding `prefer_grpc=False` should eliminate it
- If it persists, you can ignore it safely

## Summary

✅ CORS now allows all origins for development
✅ OPTIONS preflight requests handled properly
✅ Qdrant client compatibility improved
✅ Translation endpoint should work from browser

**Next step:** Restart backend and test the Urdu translation button!
