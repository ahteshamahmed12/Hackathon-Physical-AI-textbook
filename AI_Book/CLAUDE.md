# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

Commonly used commands for development in this Docusaurus project:

*   **Install Dependencies:**
    ```bash
    yarn
    ```
*   **Start Local Development Server:**
    ```bash
    yarn start
    ```
    (Starts a local server and opens a browser. Most changes reflect live.)
*   **Build Static Website:**
    ```bash
    yarn build
    ```
    (Generates static content into the `build` directory.)
*   **Run Type Checking:**
    ```bash
    yarn typecheck
    ```
*   **Deploy Website (SSH):**
    ```bash
    USE_SSH=true yarn deploy
    ```
*   **Deploy Website (Non-SSH):**
    ```bash
    GIT_USER=<Your GitHub username> yarn deploy
    ```
    (For GitHub Pages deployment.)

Other Docusaurus-specific commands available in `package.json` include `swizzle`, `clear`, `serve`, `write-translations`, and `write-heading-ids`.

## High-Level Code Architecture and Structure

This repository contains a Docusaurus-based static website primarily used for documentation.

*   **Configuration:**
    *   `docusaurus.config.ts`: Main Docusaurus configuration, controlling plugins, themes, and site settings.
    *   `sidebars.ts`: Defines the structure and order of documentation navigation.
    *   `package.json`: Manages project dependencies and defines scripts.
    *   `tsconfig.json`: TypeScript compiler configuration.

*   **Content:**
    *   `blog/`: Contains markdown files for blog posts.
    *   `docs/`: Houses the primary documentation markdown files.
    *   `src/`: Contains custom React components, pages, or styling extensions for Docusaurus.
    *   `static/`: Stores static assets like images and favicons.

*   **Build Artifacts:**
    *   `build/`: Output directory for the compiled Docusaurus site.
    *   `node_modules/`: Installed Node.js dependencies.

*   **Custom Project Specific Files:**
    *   `history/`: A directory potentially used for storing Prompt History Records (PHRs).
    *   `tasks.md`: A custom file likely used for tracking project tasks.
