import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
// Replace the existing content of the 'tutorialSidebar' with your modules
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI Book',
      items: [
        'module-1-intro', // Corresponds to docs/module-1-intro.md
        'module-2-core-concepts', // Corresponds to docs/module-2-core-concepts.md
        'module-3-advanced-topics',
        'module-4-conclusion',
      ],
    },
  ],
};

export default sidebars;


