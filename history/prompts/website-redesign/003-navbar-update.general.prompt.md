---
id: 003
title: "Navbar Update - Remove TextBook Link"
stage: "general"
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "website-redesign"
branch: "master"
user: "general"
command: "update-navbar"
labels: ["navbar", "navigation", "ui", "redesign"]
links:
  spec: "../specs/website-redesign/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - "frontend/textbook-physical-ai/docusaurus.config.js"
  - "history/prompts/website-redesign/003-navbar-update.general.prompt.md"
tests_yaml: |
  - "Navbar layout verification"
  - "Link functionality test"
---

# Navbar Update - Remove TextBook Link

## Original Request
Remove the "TextBook" link from the navbar and keep only "PHYSICAL AI & HUMANOID ROBOTS" as a single link, with both elements set separately.

## Context
The current navbar has both "PHYSICAL AI & HUMANOID ROBOTS" and "TextBook" as separate links in the HTML. The request is to remove the "TextBook" link and keep only the main title link.

## Change Made

### Before:
```html
<div class="navbar__logo-text">
  <span class="navbar__title-main">
    <a href="/textbook-physical-ai" class="navbar-title-link">PHYSICAL AI & HUMANOID ROBOTS</a>
  </span>
  <span class="textbook-hover">
    <a href="/textbook-physical-ai/docs" class="textbook-link">TextBook</a>
  </span>
</div>
```

### After:
```html
<div class="navbar__logo-text">
  <span class="navbar__title-main">
    <a href="/textbook-physical-ai" class="navbar-title-link">PHYSICAL AI & HUMANOID ROBOTS</a>
  </span>
</div>
```

## Technical Details

### File Modified
- `frontend/textbook-physical-ai/docusaurus.config.js` - Updated navbar HTML configuration

### Impact
- Removed "TextBook" link from navbar
- Kept only the main "PHYSICAL AI & HUMANOID ROBOTS" link
- Maintained all other navigation elements (GitHub link and theme toggle)
- Preserved existing styling classes

## Outcome
- ✅ "TextBook" link successfully removed from navbar
- ✅ "PHYSICAL AI & HUMANOID ROBOTS" link preserved as main title
- ✅ All other navigation elements remain intact
- ✅ GitHub link and theme toggle still functional
- ✅ Navbar layout remains clean and minimal

## Verification
- Confirmed "TextBook" link is no longer in the navbar
- Verified main title link still functions correctly
- Ensured other navigation items (GitHub, theme toggle) remain operational
- Maintained the clean, minimal design approach from the redesign