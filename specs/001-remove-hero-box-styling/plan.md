# Implementation Plan: Remove Box Styling from Hero Section

**Branch**: `001-remove-hero-box-styling` | **Date**: 2025-12-17 | **Spec**: specs/001-remove-hero-box-styling/spec.md
**Input**: Feature specification from `/specs/001-remove-hero-box-styling/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Remove box or card styling from the main hero section containing "PHYSICAL AI · HUMANOID ROBOTICS", "Interactive Curriculum", "Master These Skills", and "Join the Future of Robotics Education" while preserving all text, spacing, layout, and alignment. Optimize performance by reducing unnecessary CSS in the hero section, avoiding heavy visual effects, and ensuring fonts and styles load efficiently. Maintain all existing functionality including dark mode, light mode, navigation, and other website features.

## Technical Context

**Language/Version**: CSS/SCSS, JavaScript/TypeScript, Docusaurus v3
**Primary Dependencies**: Docusaurus, React, MDX, CSS-in-JS, styled components
**Storage**: N/A (UI styling, no persistent storage)
**Testing**: Visual verification, browser compatibility testing
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web application (Docusaurus documentation site)
**Performance Goals**: <10% improvement in page load time, smooth rendering
**Constraints**: Must maintain existing layout, navigation, and functionality
**Scale/Scope**: Single documentation site with hero section optimization

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Check
- ✅ **Embodied Intelligence Focus**: N/A - This is a UI enhancement feature
- ✅ **AI-Native Educational Experience**: Compliant - Improves user experience of the AI textbook interface
- ✅ **Modular Learning Progression**: N/A - This is a UI enhancement feature
- ✅ **Practical Application Emphasis**: N/A - This is a UI enhancement feature
- ✅ **Advanced Technology Integration**: Compliant - Uses Docusaurus v3 technology stack as specified
- ✅ **Accessibility and Inclusion**: Enhanced - Better performance and cleaner UI improves accessibility
- ✅ **Authentication and Personalization**: N/A - This is a UI enhancement feature

### Technical Standards Compliance
- ✅ Uses Docusaurus (Vercel) for responsive, modern textbook presentation
- ✅ Maintains existing interactive elements and functionality
- ✅ Supports multilingual content presentation (improved performance for all languages)
- ✅ Maintains text selection → popup functionality

## Project Structure

### Documentation (this feature)

```text
specs/001-remove-hero-box-styling/
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
│   ├── pages/
│   ├── styles/
│   │   ├── custom.css
│   │   └── theme.css
│   └── theme/
│       ├── Navbar/
│       ├── Layout/
│       └── ColorMode/
├── static/
├── docusaurus.config.js
├── package.json
└── babel.config.js

docs/
├── intro.md
└── [other markdown files]

src/
├── css/
│   └── custom.css      # Docusaurus custom styles
└── theme/
    └── [custom theme components]
```

**Structure Decision**: Docusaurus-based web application with CSS customizations in the standard Docusaurus structure. The hero section fix will involve updating CSS styles to remove box/card styling while maintaining content and layout.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Post-Design Constitution Check

### Compliance Verification After Design
- ✅ **Accessibility and Inclusion**: Enhanced - Improved performance and cleaner UI
- ✅ **AI-Native Educational Experience**: Maintained - Better UI experience for textbook
- ✅ **Advanced Technology Integration**: Compliant - Proper use of Docusaurus v3 styling
- ✅ **Technical Standards**: Compliant - Uses Docusaurus CSS variables and styling system
