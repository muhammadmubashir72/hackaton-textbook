---
id: 006
title: "Navbar GitHub Link Update"
stage: "general"
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "website-redesign"
branch: "master"
user: "general"
command: "update-navbar-github"
labels: ["navbar", "github", "navigation", "ui"]
links:
  spec: "../specs/website-redesign/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - "frontend/textbook-physical-ai/docusaurus.config.js"
  - "frontend/textbook-physical-ai/src/css/custom.css"
  - "history/prompts/website-redesign/006-navbar-github-update.general.prompt.md"
tests_yaml: |
  - "Navbar layout verification"
  - "GitHub link functionality test"
  - "Responsive design check"
---

# Navbar GitHub Link Update

## Original Request
Update the navbar of the "PHYSICAL AI & HUMANOID ROBOTS TextBook" site. Remove the existing "Docs" item from the title area. Replace it with a "GitHub" text link that includes the GitHub icon next to it. The link should open in a new tab and appear on the left side of the navbar. Keep the design consistent with the existing theme and spacing.

## Context
The navbar needed to be updated to remove the "TextBook" link that was previously in the title area and replace it with a GitHub link that includes both text and an icon. The GitHub link should open in a new tab and be positioned on the left side next to the main title.

## Changes Made

### 1. Updated Navbar Configuration
- Modified the HTML structure in the left navbar item to include both the main title and a GitHub link
- Added SVG GitHub icon alongside the "GitHub" text
- Set the link to open in a new tab with proper security attributes
- Removed the duplicate GitHub link that was previously on the right side

### 2. Added CSS Styling
- Created specific CSS classes for the navbar GitHub link elements
- Styled the link to match the existing theme with appropriate colors and hover effects
- Ensured proper alignment of the icon and text
- Added hover effects that match the existing theme

## Technical Details

### Files Modified
- `frontend/textbook-physical-ai/docusaurus.config.js` - Updated navbar HTML structure
- `frontend/textbook-physical-ai/src/css/custom.css` - Added GitHub link styling

### HTML Structure Added
```html
<span class="navbar__title-main">PHYSICAL AI & HUMANOID ROBOTS</span>
<span> · </span>
<a href="https://github.com/panaversity/textbook-physical-ai"
   target="_blank"
   rel="noopener noreferrer"
   class="navbar-github-link">
  <svg width="20" height="20" viewBox="0 0 24 24" class="navbar-github-icon">
    <path fill="currentColor" d="M12 2A10 10 0 0 0 2 12c0 4.42 2.87 8.17 6.84 9.5.5.08.66-.23.66-.5v-1.69c-2.77.6-3.36-1.34-3.36-1.34-.46-1.16-1.11-1.47-1.11-1.47-.91-.62.07-.6.07-.6 1 .07 1.53 1.03 1.53 1.03.87 1.52 2.34 1.07 2.91.83.09-.65.35-1.09.63-1.34-2.22-.25-4.55-1.11-4.55-4.92 0-1.11.38-2 1.03-2.71-.1-.25-.45-1.29.1-2.64 0 0 .84-.27 2.75 1.02.79-.22 1.65-.33 2.5-.33.85 0 1.71.11 2.5.33 1.91-1.29 2.75-1.02 2.75-1.02.55 1.35.2 2.39.1 2.64.65.71 1.03 1.6 1.03 2.71 0 3.82-2.34 4.66-4.57 4.91.36.31.69.92.69 1.85V21c0 .27.16.59.67.5C19.14 20.16 22 16.42 22 12A10 10 0 0 0 12 2z"/>
  </svg>
  <span class="navbar-github-text">GitHub</span>
</a>
```

### CSS Classes Added
- `.navbar-github-link` - Main link styling
- `.navbar-github-icon` - Icon alignment and sizing
- `.navbar-github-text` - Text alignment

## Outcome
- ✅ "TextBook" link removed from title area
- ✅ GitHub link with icon added to left side of navbar
- ✅ Link opens in new tab with security attributes
- ✅ Consistent styling with existing theme
- ✅ Proper spacing and alignment maintained
- ✅ Duplicate GitHub link removed from right side
- ✅ Theme toggle preserved on right side

## Verification
- GitHub link appears next to main title on left side
- GitHub icon displays correctly alongside text
- Link opens in new tab when clicked
- Hover effects match existing theme
- Responsive design maintained
- Theme toggle remains functional on right side
- All styling is consistent with the professional theme