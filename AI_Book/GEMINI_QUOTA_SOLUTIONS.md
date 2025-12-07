# Gemini API Quota Solutions

## Understanding the Error

```
429 You exceeded your current quota
Quota exceeded for metric: generativelanguage.googleapis.com/embed_content_free_tier_requests, limit: 0
```

This error occurs when you've exceeded Google Gemini's free tier limits.

## Gemini Free Tier Limits

- **Daily Requests**: 1,500 requests per day
- **Rate Limit**: 60 requests per minute
- **Quota Reset**: Every 24 hours (Pacific Time)

## Immediate Solutions

### Solution 1: Wait for Quota Reset

The quota resets every 24 hours. Check your usage:
- Visit: https://ai.dev/usage?tab=rate-limit
- See when your quota resets
- Wait and try again after reset

### Solution 2: Create a New API Key

If you've exhausted one key:

1. Visit: https://makersuite.google.com/app/apikey
2. Create a **new API key**
3. Update your `.env` file:
   ```env
   GEMINI_API_KEY=your_new_api_key_here
   ```
4. Restart the backend

### Solution 3: Enable Billing (Paid Tier)

For production use or higher limits:

1. Visit: https://console.cloud.google.com/
2. Select your project
3. Enable billing
4. Gemini API quotas will increase significantly

### Solution 4: Switch to Alternative Embeddings

#### Option A: Use HuggingFace Embeddings (FREE, Unlimited)

**Advantages:**
- ✅ Completely free
- ✅ No API key required
- ✅ Runs locally
- ✅ No quota limits

**Disadvantages:**
- ⚠️ Slower (runs on your CPU)
- ⚠️ Slightly lower quality than Gemini/OpenAI

**Implementation:**
See `ALTERNATIVE_EMBEDDINGS.md` for code

#### Option B: Use Cohere Embeddings (FREE Tier)

**Advantages:**
- ✅ 10,000 requests/month free
- ✅ High quality
- ✅ Fast API

**Disadvantages:**
- ⚠️ Requires API key
- ⚠️ Monthly limit

#### Option C: Return to OpenAI (Paid)

**Advantages:**
- ✅ Very high quality
- ✅ Reliable
- ✅ High rate limits

**Disadvantages:**
- ❌ Costs money ($0.02/1M tokens)

## Best Practices to Avoid Quota Issues

### 1. Implement Rate Limiting

Add delays between requests to avoid hitting rate limits:

```python
import time

# In ingest.py, add delay between chunks:
for i, chunk in enumerate(chunks):
    if i > 0 and i % 60 == 0:  # Every 60 requests
        print("  ⏰ Pausing to respect rate limit...")
        time.sleep(60)  # Wait 1 minute

    # Generate embedding...
```

### 2. Cache Embeddings Locally

Save embeddings to avoid regenerating:

```python
import pickle

# Save embeddings
with open('embeddings_cache.pkl', 'wb') as f:
    pickle.dump(embeddings, f)

# Load cached embeddings
with open('embeddings_cache.pkl', 'rb') as f:
    embeddings = pickle.load(f)
```

### 3. Batch Processing

Process documents in smaller batches:

```python
# Process 100 docs at a time
batch_size = 100
for i in range(0, len(documents), batch_size):
    batch = documents[i:i+batch_size]
    process_batch(batch)
    time.sleep(60)  # Wait between batches
```

### 4. Monitor Usage

Check your quota regularly:
```bash
# Visit this URL to see your usage:
https://ai.dev/usage?tab=rate-limit
```

## Error Handling in Code

### Enhanced app.py with Retry Logic

```python
import time
from functools import wraps

def retry_on_quota_error(max_retries=3, delay=60):
    """Retry decorator for quota errors"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if "429" in str(e) or "quota" in str(e).lower():
                        if attempt < max_retries - 1:
                            print(f"⚠️  Quota exceeded, waiting {delay}s before retry...")
                            time.sleep(delay)
                            continue
                    raise
            return None
        return wrapper
    return decorator

@retry_on_quota_error(max_retries=3, delay=60)
def generate_embedding(text):
    """Generate embedding with retry logic"""
    result = genai.embed_content(
        model="models/embedding-001",
        content=text,
        task_type="retrieval_query"
    )
    return result['embedding']
```

## Checking Your Quota Status

### Via API

```python
import google.generativeai as genai

genai.configure(api_key="your_key")

# This will show quota errors if exceeded
try:
    result = genai.embed_content(
        model="models/embedding-001",
        content="test",
        task_type="retrieval_query"
    )
    print("✓ Quota available")
except Exception as e:
    if "429" in str(e):
        print("❌ Quota exceeded")
        print(f"Error: {e}")
```

### Via Dashboard

1. Visit: https://console.cloud.google.com/
2. Navigate to: APIs & Services → Dashboard
3. Check: Gemini API usage metrics

## Temporary Workaround: Mock Embeddings

For development/testing when quota is exceeded:

```python
import numpy as np

def mock_embedding(text, dim=768):
    """Generate mock embedding for testing"""
    # Use hash for consistency
    seed = hash(text) % (2**32)
    np.random.seed(seed)
    return np.random.randn(dim).tolist()
```

**⚠️ Warning**: Only use for testing, not production!

## Long-Term Solutions

### 1. Upgrade to Paid Tier

**Costs** (as of 2024):
- Gemini embeddings: Very competitive, check current pricing
- Typically: Fraction of a cent per 1000 requests

**How to Enable**:
1. Go to: https://console.cloud.google.com/billing
2. Link a billing account
3. Quotas automatically increase

### 2. Use Multiple API Keys

Rotate between multiple free-tier keys:

```python
GEMINI_API_KEYS = [
    os.getenv("GEMINI_API_KEY_1"),
    os.getenv("GEMINI_API_KEY_2"),
    os.getenv("GEMINI_API_KEY_3"),
]

current_key_index = 0

def get_next_api_key():
    global current_key_index
    key = GEMINI_API_KEYS[current_key_index]
    current_key_index = (current_key_index + 1) % len(GEMINI_API_KEYS)
    return key
```

### 3. Hybrid Approach

Use Gemini for queries, HuggingFace for ingestion:

```python
# In ingest.py - use free HuggingFace
from sentence_transformers import SentenceTransformer
model = SentenceTransformer('all-MiniLM-L6-v2')

# In app.py - use Gemini for queries (lower volume)
# Gemini API as before
```

## Comparison of Embedding Options

| Provider | Free Tier | Quality | Speed | Quota |
|----------|-----------|---------|-------|-------|
| **Google Gemini** | 1,500/day | ⭐⭐⭐⭐⭐ | Fast | Daily |
| **HuggingFace (Local)** | Unlimited | ⭐⭐⭐⭐ | Slow | None |
| **Cohere** | 10,000/month | ⭐⭐⭐⭐⭐ | Fast | Monthly |
| **OpenAI** | $5 credit | ⭐⭐⭐⭐⭐ | Fast | Paid |

## Recommended Action Plan

**Immediate (Next 5 minutes):**
1. Check your quota status at https://ai.dev/usage?tab=rate-limit
2. If exceeded today, either:
   - Wait for reset (check reset time)
   - Create new API key
   - Switch to HuggingFace temporarily

**Short-term (This week):**
1. Implement rate limiting in code
2. Add retry logic with exponential backoff
3. Monitor usage daily

**Long-term (For production):**
1. Enable billing on Google Cloud
2. Or switch to more generous free tier (Cohere: 10K/month)
3. Or use local embeddings (HuggingFace)

## Need Help?

1. **Check Quota**: https://ai.dev/usage?tab=rate-limit
2. **Gemini Docs**: https://ai.google.dev/gemini-api/docs/rate-limits
3. **Google Cloud Console**: https://console.cloud.google.com/
4. **Alternative Solutions**: See `ALTERNATIVE_EMBEDDINGS.md`

---

**TL;DR**: Your free tier quota is exhausted. Either wait 24 hours for reset, create a new API key, enable billing, or switch to HuggingFace embeddings (unlimited, free).
