/**
 * UrduTranslator Component for Docusaurus
 *
 * Provides on-the-fly translation of markdown documentation pages to Urdu
 * using the FastAPI backend translation endpoint.
 *
 * FEATURES:
 * - Translates current page content to Urdu
 * - Displays translated content in a modal overlay
 * - Preserves markdown formatting
 * - Loading states and error handling
 * - Toggle between original and translated view
 *
 * USAGE:
 * Import and use in Docusaurus theme or as a standalone component
 */

import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const UrduTranslator = () => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const [error, setError] = useState(null);

  /**
   * Extract markdown content from the current page
   */
  const getCurrentPageContent = () => {
    if (!ExecutionEnvironment.canUseDOM) {
      return null;
    }

    // Get the main documentation content
    const articleElement = document.querySelector('article') ||
                          document.querySelector('.markdown') ||
                          document.querySelector('main');

    if (!articleElement) {
      throw new Error('Could not find page content to translate');
    }

    // Extract text content while preserving structure
    return articleElement.innerText || articleElement.textContent;
  };

  /**
   * Translate current page to Urdu
   */
  const translateToUrdu = async () => {
    setIsTranslating(true);
    setError(null);

    try {
      // Get current page content
      const content = getCurrentPageContent();

      if (!content || content.trim().length === 0) {
        throw new Error('No content found on this page');
      }

      // Call translation API
      const response = await fetch('http://localhost:8000/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: content,
          target_language: 'Urdu'
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.detail ||
          `Translation failed (${response.status}): ${response.statusText}`
        );
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setShowTranslation(true);
      setIsTranslating(false);

    } catch (err) {
      console.error('Translation error:', err);

      let errorMessage = 'Translation failed. ';
      if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
        errorMessage += 'Unable to connect to translation service. Please ensure the backend is running at http://localhost:8000.';
      } else {
        errorMessage += err.message;
      }

      setError(errorMessage);
      setIsTranslating(false);
    }
  };

  /**
   * Close translation modal
   */
  const closeTranslation = () => {
    setShowTranslation(false);
  };

  /**
   * Reset state and retranslate
   */
  const retranslate = () => {
    setTranslatedContent(null);
    setShowTranslation(false);
    translateToUrdu();
  };

  return (
    <>
      {/* Urdu Translation Button */}
      <button
        onClick={translatedContent ? retranslate : translateToUrdu}
        disabled={isTranslating}
        style={styles.urduButton}
        title="Translate page to Urdu"
        className="urdu-translate-button"
      >
        {isTranslating ? (
          <>
            <span style={styles.spinner}>⏳</span> Translating...
          </>
        ) : (
          <>
            🌐 اردو
          </>
        )}
      </button>

      {/* Translation Modal */}
      {showTranslation && translatedContent && (
        <div style={styles.modalOverlay} onClick={closeTranslation}>
          <div style={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            {/* Modal Header */}
            <div style={styles.modalHeader}>
              <h2 style={styles.modalTitle}>
                <span>🌐</span> Urdu Translation | اردو ترجمہ
              </h2>
              <button
                onClick={closeTranslation}
                style={styles.closeButton}
                title="Close"
              >
                ✕
              </button>
            </div>

            {/* Translated Content */}
            <div style={styles.translatedText}>
              <pre style={styles.preformatted}>{translatedContent}</pre>
            </div>

            {/* Modal Footer */}
            <div style={styles.modalFooter}>
              <button onClick={retranslate} style={styles.retranslateButton}>
                🔄 Retranslate
              </button>
              <button onClick={closeTranslation} style={styles.closeButtonBottom}>
                Close | بند کریں
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Error Display */}
      {error && (
        <div style={styles.errorBanner}>
          <span style={styles.errorIcon}>⚠️</span>
          <span style={styles.errorText}>{error}</span>
          <button
            onClick={() => setError(null)}
            style={styles.errorClose}
          >
            ✕
          </button>
        </div>
      )}
    </>
  );
};

// Inline styles
const styles = {
  urduButton: {
    padding: '8px 16px',
    fontSize: '14px',
    fontWeight: '600',
    backgroundColor: '#2e7d32',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    transition: 'all 0.3s ease',
    display: 'flex',
    alignItems: 'center',
    gap: '6px',
    boxShadow: '0 2px 8px rgba(46, 125, 50, 0.3)',
  },
  spinner: {
    animation: 'spin 1s linear infinite',
  },
  modalOverlay: {
    position: 'fixed',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: 'rgba(0, 0, 0, 0.7)',
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    zIndex: 9999,
    padding: '20px',
  },
  modalContent: {
    backgroundColor: 'white',
    borderRadius: '16px',
    maxWidth: '900px',
    width: '100%',
    maxHeight: '85vh',
    display: 'flex',
    flexDirection: 'column',
    boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3)',
    overflow: 'hidden',
  },
  modalHeader: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '20px 24px',
    borderBottom: '1px solid #e0e0e0',
    backgroundColor: '#f8f9fa',
  },
  modalTitle: {
    margin: 0,
    fontSize: '22px',
    fontWeight: '700',
    color: '#2e7d32',
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
  },
  closeButton: {
    background: 'none',
    border: 'none',
    fontSize: '28px',
    cursor: 'pointer',
    color: '#666',
    padding: '0',
    width: '32px',
    height: '32px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    borderRadius: '50%',
    transition: 'all 0.2s',
  },
  translatedText: {
    flex: 1,
    overflow: 'auto',
    padding: '24px',
    backgroundColor: 'white',
    direction: 'rtl',  // Right-to-left for Urdu
    textAlign: 'right',
    fontFamily: '"Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif',
    fontSize: '18px',
    lineHeight: '2',
  },
  preformatted: {
    whiteSpace: 'pre-wrap',
    wordWrap: 'break-word',
    margin: 0,
    fontFamily: 'inherit',
  },
  modalFooter: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '16px 24px',
    borderTop: '1px solid #e0e0e0',
    backgroundColor: '#f8f9fa',
  },
  retranslateButton: {
    padding: '10px 20px',
    fontSize: '14px',
    backgroundColor: '#1976d2',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: '600',
    transition: 'all 0.3s',
  },
  closeButtonBottom: {
    padding: '10px 20px',
    fontSize: '14px',
    backgroundColor: '#666',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: '600',
    transition: 'all 0.3s',
  },
  errorBanner: {
    position: 'fixed',
    top: '20px',
    right: '20px',
    backgroundColor: '#fff3cd',
    color: '#856404',
    padding: '16px 20px',
    borderRadius: '8px',
    boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    maxWidth: '400px',
    zIndex: 10000,
    border: '1px solid #ffeaa7',
  },
  errorIcon: {
    fontSize: '20px',
  },
  errorText: {
    flex: 1,
    fontSize: '14px',
    lineHeight: '1.5',
  },
  errorClose: {
    background: 'none',
    border: 'none',
    fontSize: '20px',
    cursor: 'pointer',
    color: '#856404',
    padding: '0',
  },
};

export default UrduTranslator;
