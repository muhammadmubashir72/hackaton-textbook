# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Translation button for chapter pages that enables Urdu translation with toggle functionality. The solution will use client-side translation with googletrans library to dynamically translate chapter content (headings, paragraphs, lists) while preserving code blocks, links, and formatting. Translation state will be managed per chapter using browser storage, with toggle functionality to switch between original and translated content without page reload.

## Technical Context

**Language/Version**: JavaScript/ES6+ for frontend, Python 3.11 for backend
**Primary Dependencies**: Docusaurus, React 18, googletrans, DOM manipulation libraries
**Storage**: Browser localStorage/sessionStorage for translation state management
**Testing**: Jest for unit testing, React Testing Library for component testing
**Target Platform**: Web browser (Chrome, Firefox, Safari, Edge)
**Project Type**: Web application (frontend Docusaurus site)
**Performance Goals**: Translation should complete within 3 seconds for chapters up to 5000 words
**Constraints**: Must work without page reload, preserve all formatting and interactive elements, maintain existing functionality
**Scale/Scope**: Single-page application behavior for chapter content, per-session storage

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Verification

**✅ Principle VI - Accessibility and Inclusion**:
- Implementation includes Urdu translation functionality as required
- Uses googletrans/HF model as specified in constitution (NO GPT)

**✅ Principle II - AI-Native Educational Experience**:
- Adds interactive translation feature to enhance learning experience
- Integrates with existing text selection popup functionality

**✅ Technical Standards Compliance**:
- Uses Docusaurus framework as specified in constitution
- Implements client-side translation as required (no server reload)
- Preserves code blocks, links, and formatting as per requirements

**✅ Principle V - Advanced Technology Integration**:
- Works within existing technology stack (Docusaurus + FastAPI + etc.)
- Does not conflict with existing RAG system or authentication

**No violations detected** - Implementation plan aligns with all constitutional principles.

## Project Structure

### Documentation (this feature)

```text
specs/001-urdu-translation-toggle/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── TranslationButton/
│   │   │   ├── TranslationButton.js
│   │   │   └── TranslationButton.module.css
│   │   └── ChapterContent/
│   │       └── ChapterContent.js
│   ├── hooks/
│   │   └── useTranslation.js
│   ├── services/
│   │   └── translationService.js
│   └── utils/
│       └── domUtils.js
└── tests/
    ├── components/
    │   └── TranslationButton.test.js
    └── services/
        └── translationService.test.js
```

**Structure Decision**: Web application structure selected with frontend-specific implementation. The translation functionality will be implemented as React components and services within the existing Docusaurus frontend structure, following the existing patterns in the codebase.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | | |
