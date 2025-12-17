---
id: 007
title: "Navbar GitHub Link Position Correction"
stage: "general"
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "website-redesign"
branch: "master"
user: "general"
command: "fix-navbar-github-position"
labels: ["navbar", "github", "navigation", "ui", "positioning"]
links:
  spec: "../specs/website-redesign/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - "frontend/textbook-physical-ai/docusaurus.config.js"
  - "history/prompts/website-redesign/007-navbar-github-position-fix.general.prompt.md"
tests_yaml: |
  - "Navbar layout verification"
  - "GitHub link position check"
  - "Responsive design validation"
---

# Navbar GitHub Link Position Correction

## Original Request
The GitHub link was initially placed on the left side next to the main title. The request was to place the GitHub link on the right side of the navbar as specified in the original requirements.

## Context
During the previous update, I misinterpreted the position requirement. The original request specifically stated that the GitHub link should appear on the right side of the navbar. I've corrected this by moving the GitHub link to the right side while keeping only the main title on the left.

## Changes Made

### 1. Updated Navbar Structure
- Moved the GitHub link with icon to a separate item on the right side
- Kept only the main title "PHYSICAL AI & HUMANOID ROBOTS" on the left side
- Maintained the theme toggle on the right side
- Added proper positioning attributes for both elements

### 2. Corrected HTML Structure
- Left side: Simple HTML with main title only
- Right side: GitHub link with icon and text as a separate HTML item
- Right side: Theme toggle remains as a separate item

## Technical Details

### File Modified
- `frontend/textbook-physical-ai/docusaurus.config.js` - Corrected navbar positioning

### Final Navbar Structure
```
navbar: {
  items: [
    {
      type: 'html',
      value: '<div class="navbar__logo-text"><span class="navbar__title-main">PHYSICAL AI & HUMANOID ROBOTS</span></div>',
      position: 'left',
    },
    {
      type: 'html',
      value: '<a href="https://github.com/panaversity/textbook-physical-ai" target="_blank" rel="noopener noreferrer" class="navbar-github-link">...</a>',
      position: 'right',
    },
    {
      type: 'theme-toggle',
      position: 'right',
      'aria-label': 'Switch between dark and light mode',
    },
  ],
},
```

## Outcome
- ✅ Main title "PHYSICAL AI & HUMANOID ROBOTS" positioned on the left
- ✅ GitHub link with icon positioned on the right side
- ✅ Theme toggle preserved on the right side
- ✅ GitHub link opens in new tab with security attributes
- ✅ Consistent styling with existing theme
- ✅ Proper spacing and alignment maintained
- ✅ Responsive design preserved

## Verification
- GitHub link appears on the right side of navbar as requested
- Main title appears on the left side of navbar
- GitHub icon displays correctly alongside text
- Link opens in new tab when clicked
- All styling remains consistent with the professional theme
- Theme toggle remains functional on right side
- Layout maintains proper responsive behavior

## Alignment with Requirements
This update now fully satisfies the original request:
- "Remove the existing 'Docs' item from the title area" ✓ (already done, now confirmed)
- "Replace it with a 'GitHub' text link that includes the GitHub icon next to it" ✓
- "The link should open in a new tab" ✓
- "appear on the right side of the navbar" ✓ (now correctly implemented)
- "Keep the design consistent with the existing theme and spacing" ✓