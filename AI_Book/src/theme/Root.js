/**
 * Root Component Wrapper for Docusaurus
 *
 * This component wraps the entire Docusaurus application and allows
 * for global components and initialization logic.
 *
 * Used to inject the UrduTranslator and authentication buttons into the navbar.
 */

import React, { useEffect } from 'react';
import { createRoot } from 'react-dom/client';
import UrduTranslator from '../components/UrduTranslator';
import FloatingChatbot from '../components/FloatingChatbot'; // Import FloatingChatbot
import useIsBrowser from '@docusaurus/useIsBrowser';
import SearchBar from '../components/SearchBar';

function AuthButtons() {
  const isBrowser = useIsBrowser();
  const [isLoggedIn, setIsLoggedIn] = React.useState(false);

  useEffect(() => {
    if (isBrowser) {
      setIsLoggedIn(!!localStorage.getItem('access_token'));
    }
  }, [isBrowser]);

  const handleLogout = () => {
    localStorage.removeItem('access_token');
    window.location.href = '/signin';
  };

  if (!isBrowser) {
    return null;
  }

  return (
    <>
      {isLoggedIn ? (
        <button
          className="button button--secondary navbar-log-out"
          onClick={handleLogout}
        >
          Logout
        </button>
      ) : (
        <>
          <a className="button button--secondary navbar-sign-in" href="/signin">
            Sign In
          </a>
          <a className="button button--primary navbar-sign-up" href="/signup">
            Sign Up
          </a>
        </>
      )}
    </>
  );
}

export default function Root({ children }) {
  useEffect(() => {
    console.log("Root.js useEffect called");

    const initializeTranslator = () => {
      const translatorRoot = document.getElementById('urdu-translator-root');
      if (translatorRoot && !translatorRoot.hasChildNodes()) {
        const root = createRoot(translatorRoot);
        root.render(<UrduTranslator />);
      }
    };

    const initializeAuthButtons = () => {
      console.log("initializeAuthButtons called");
      const authButtonsRoot = document.getElementById('auth-buttons-root');
      console.log("authButtonsRoot:", authButtonsRoot);
      if (authButtonsRoot && !authButtonsRoot.hasChildNodes()) {
        const root = createRoot(authButtonsRoot);
        root.render(<AuthButtons />);
      }
    };

    const initializeSearchBar = () => {
        const searchBarRoot = document.getElementById('search-bar-root');
        if (searchBarRoot && !searchBarRoot.hasChildNodes()) {
            const root = createRoot(searchBarRoot);
            root.render(<SearchBar />);
        }
    };

    const initialize = () => {
      initializeTranslator();
      initializeAuthButtons();
      initializeSearchBar();
    };

    initialize();

    const observer = new MutationObserver(initialize);
    observer.observe(document.body, {
      childList: true,
      subtree: true,
    });

    return () => observer.disconnect();
  }, []);

  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
}
