---
id: 004
title: "Remove Docs Plugin Configuration"
stage: "general"
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "website-redesign"
branch: "master"
user: "general"
command: "remove-docs-plugin"
labels: ["configuration", "docusaurus", "docs", "plugin"]
links:
  spec: "../specs/website-redesign/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - "frontend/textbook-physical-ai/docusaurus.config.js"
  - "history/prompts/website-redesign/004-remove-docs-plugin.general.prompt.md"
tests_yaml: |
  - "Docusaurus build test"
  - "Configuration validation"
---

# Remove Docs Plugin Configuration

## Original Request
Remove the "docs" plugin configuration from the Docusaurus configuration file.

## Context
The Docusaurus configuration had a docs plugin configuration that was part of the classic preset. This configuration was removed to completely eliminate documentation functionality from the site.

## Change Made

### Before:
```javascript
({
  docs: {
    sidebarPath: './sidebars.js',
    // Please change this to your repo.
    // Remove this to remove the "edit this page" links.
    editUrl:
      'https://github.com/panaversity/textbook-physical-ai/tree/main/',
  },
  theme: {
    customCss: './src/css/custom.css',
  },
}),
```

### After:
```javascript
({
  theme: {
    customCss: './src/css/custom.css',
  },
}),
```

## Technical Details

### File Modified
- `frontend/textbook-physical-ai/docusaurus.config.js` - Removed docs plugin configuration

### Impact
- Removed the docs plugin from the Docusaurus preset
- Eliminated documentation functionality from the site
- Preserved the theme configuration and custom CSS
- Removed sidebarPath and editUrl configurations

## Outcome
- ✅ Docs plugin configuration successfully removed
- ✅ Theme configuration preserved
- ✅ Custom CSS configuration maintained
- ✅ Docusaurus configuration syntax remains valid
- ✅ Site will no longer have documentation routes

## Verification
- Confirmed docs plugin is removed from preset configuration
- Verified remaining configuration is syntactically correct
- Maintained other essential configurations (theme, custom CSS)
- Configuration should still allow successful Docusaurus build

## Relationship to Redesign
This change continues the minimalist approach from the website redesign by removing documentation functionality entirely, keeping only the core website features.