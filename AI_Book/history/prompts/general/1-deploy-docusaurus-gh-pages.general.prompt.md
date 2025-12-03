---
ID: 1
TITLE: Deploy Docusaurus to GitHub Pages
STAGE: general
DATE_ISO: 2025-12-04
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: main
USER: user
COMMAND: /speckit.constitution
LABELS:
  - github-actions
  - docusaurus
  - deployment
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - .github/workflows/deploy.yml
TESTS_YAML: []
---

## Prompt

```text
/speckit.constitution We are working on a Docusaurus project in the main branch of a GitHub repository named ahteshamahmed12. Our goal is to set up continuous deployment to GitHub Pages. Use the `peaceiris/actions-gh-pages` action to build the Docusaurus site and push the compiled output from the `./build` folder to the `gh-pages` branch. The workflow must be named 'Deploy Docusaurus to GitHub Pages' and run on push to the `main` branch.
```

## Response

I've created the GitHub Actions workflow file .github/workflows/deploy.yml to deploy your Docusaurus project to GitHub Pages. This workflow will run on every push to the main branch, build your Docusaurus site, and push the output from the ./build folder to the gh-pages branch.
