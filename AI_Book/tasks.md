---
description: "Task list for debugging the 'Start Learning' button"
---

# Tasks: Debug 'Start Learning' Button

**Input**: The 'Start Learning' button on the homepage is not working as expected.
**Goal**: Identify the root cause of the issue and fix it.

## Phase 1: Investigation

**Purpose**: Gather information and diagnose the problem.

- [ ] T001 Verify Docusaurus version in `package.json` to check for known routing bugs.
- [ ] T002 [P] Open the browser's developer console and check for any errors when clicking the button.
- [ ] T003 [P] Add `console.log(props)` inside the `HomepageHeader` component in `src/pages/index.tsx` to inspect the props passed to the `Link` component.
- [ ] T004 In `src/pages/index.tsx`, try different URL formats in the `to` prop of the `Link` component, such as:
    - `to={'/module-1-intro'}`
    - `to={'./module-1-intro'}`
    - `to={'/docs/module-1-intro'}`
- [ ] T005 [P] Review `docusaurus.config.ts` for any plugins or configurations that might be conflicting with the routing.
- [ ] T006 Run `npm run build` and inspect the generated HTML in the `build` directory to see what the `href` of the button is.
- [ ] T007 Temporarily disable the `plugin-client-redirects` in `docusaurus.config.ts` to see if it's causing the issue.

## Phase 2: Implementation

**Purpose**: Apply the fix based on the findings from Phase 1.

- [ ] T008 Apply the necessary code changes to fix the button's link. This might involve editing `src/pages/index.tsx`, `docusaurus.config.ts`, or `sidebars.ts`.
- [ ] T009 [P] If the issue is related to styling, adjust the CSS in `src/css/custom.css`.
- [ ] T010 Remove any debugging `console.log` statements.

## Phase 3: Verification

**Purpose**: Ensure the fix works and has not introduced any regressions.

- [ ] T011 Run the website locally with `npm run start` and confirm that the "Start Learning" button now navigates to the correct page.
- [ ] T012 [P] Click through the main navigation links to ensure no other links were broken.
- [ ] T013 Build the project again with `npm run build` and check for any build errors.