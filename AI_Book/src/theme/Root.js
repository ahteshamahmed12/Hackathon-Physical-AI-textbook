/**
 * Root Component Wrapper for Docusaurus
 *
 * This component wraps the entire Docusaurus application and allows
 * for global components and initialization logic.
 *
 * Used to inject the UrduTranslator component into the navbar.
 */

import React, { useEffect } from 'react';
import { createRoot } from 'react-dom/client';
import UrduTranslator from '../components/UrduTranslator';

export default function Root({ children }) {
  useEffect(() => {
    // Wait for navbar to be rendered
    const initializeTranslator = () => {
      const translatorRoot = document.getElementById('urdu-translator-root');

      if (translatorRoot && !translatorRoot.hasChildNodes()) {
        // Mount the UrduTranslator component
        const root = createRoot(translatorRoot);
        root.render(<UrduTranslator />);
      }
    };

    // Initialize immediately
    initializeTranslator();

    // Also initialize on route changes
    const observer = new MutationObserver(initializeTranslator);
    observer.observe(document.body, {
      childList: true,
      subtree: true,
    });

    return () => observer.disconnect();
  }, []);

  return <>{children}</>;
}
