import React, { useState, useRef, useEffect } from 'react';
import './SearchBar.css';

const SearchBar = () => {
  const [searchQuery, setSearchQuery] = useState('');
  const [searchResults, setSearchResults] = useState([]);
  const [isSearching, setIsSearching] = useState(false);
  const [showResults, setShowResults] = useState(false);
  const [error, setError] = useState('');
  const searchRef = useRef(null);

  // Close results dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (searchRef.current && !searchRef.current.contains(event.target)) {
        setShowResults(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  // Debounced search function
  useEffect(() => {
    if (searchQuery.trim().length < 2) {
      setSearchResults([]);
      setShowResults(false);
      return;
    }

    const timeoutId = setTimeout(() => {
      performSearch(searchQuery);
    }, 500); // Wait 500ms after user stops typing

    return () => clearTimeout(timeoutId);
  }, [searchQuery]);

  const performSearch = async (query) => {
    setIsSearching(true);
    setError('');

    try {
      const response = await fetch('http://localhost:8000/search', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query }),
      });

      if (!response.ok) {
        throw new Error(`Search failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      setSearchResults(data.results || []);
      setShowResults(true);
    } catch (err) {
      console.error('Search error:', err);
      setError('Search is currently unavailable. Please try again later.');
      setSearchResults([]);
    } finally {
      setIsSearching(false);
    }
  };

  const handleSearchChange = (e) => {
    setSearchQuery(e.target.value);
  };

  const handleSearchSubmit = (e) => {
    e.preventDefault();
    if (searchQuery.trim().length >= 2) {
      performSearch(searchQuery);
    }
  };

  const handleResultClick = (result) => {
    // Navigate to the document
    if (result.source) {
      window.location.href = `/${result.source}`;
    }
    setShowResults(false);
    setSearchQuery('');
  };

  const highlightText = (text, query) => {
    if (!query || !text) return text;

    const parts = text.split(new RegExp(`(${query})`, 'gi'));
    return parts.map((part, index) =>
      part.toLowerCase() === query.toLowerCase() ? (
        <mark key={index}>{part}</mark>
      ) : (
        part
      )
    );
  };

  return (
    <div className="navbar-search" ref={searchRef}>
      <form onSubmit={handleSearchSubmit} className="search-form">
        <div className="search-input-wrapper">
          <input
            type="text"
            className="search-input"
            placeholder="Search documentation..."
            value={searchQuery}
            onChange={handleSearchChange}
            aria-label="Search documentation"
          />
          {isSearching && (
            <div className="search-spinner">
              <div className="spinner"></div>
            </div>
          )}
          <button
            type="submit"
            className="search-button"
            aria-label="Submit search"
            disabled={searchQuery.trim().length < 2}
          >
            <svg
              width="20"
              height="20"
              viewBox="0 0 20 20"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M9 17A8 8 0 1 0 9 1a8 8 0 0 0 0 16zM19 19l-4.35-4.35"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
        </div>
      </form>

      {showResults && (
        <div className="search-results-dropdown">
          {error && (
            <div className="search-error">
              <svg
                width="16"
                height="16"
                viewBox="0 0 16 16"
                fill="currentColor"
              >
                <path d="M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z" />
                <path d="M7.002 11a1 1 0 1 1 2 0 1 1 0 0 1-2 0zM7.1 4.995a.905.905 0 1 1 1.8 0l-.35 3.507a.552.552 0 0 1-1.1 0L7.1 4.995z" />
              </svg>
              {error}
            </div>
          )}

          {!error && searchResults.length === 0 && !isSearching && (
            <div className="search-no-results">
              <p>No results found for "{searchQuery}"</p>
              <small>Try different keywords or check your spelling</small>
            </div>
          )}

          {!error && searchResults.length > 0 && (
            <div className="search-results-list">
              <div className="search-results-header">
                Found {searchResults.length} result{searchResults.length !== 1 ? 's' : ''}
              </div>
              {searchResults.map((result, index) => (
                <div
                  key={index}
                  className="search-result-item"
                  onClick={() => handleResultClick(result)}
                  role="button"
                  tabIndex={0}
                  onKeyPress={(e) => {
                    if (e.key === 'Enter') handleResultClick(result);
                  }}
                >
                  <div className="result-title">
                    {highlightText(result.title || 'Untitled', searchQuery)}
                  </div>
                  <div className="result-content">
                    {highlightText(
                      result.content?.substring(0, 150) || '',
                      searchQuery
                    )}
                    {result.content?.length > 150 && '...'}
                  </div>
                  <div className="result-metadata">
                    <span className="result-source">{result.source || 'Unknown source'}</span>
                    {result.score && (
                      <span className="result-score">
                        Relevance: {(result.score * 100).toFixed(0)}%
                      </span>
                    )}
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default SearchBar;
