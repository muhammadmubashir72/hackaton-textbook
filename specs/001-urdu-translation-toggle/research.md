# Research: Urdu Translation Toggle Feature

## Decision: Translation Implementation Approach
**Rationale**: Based on the constitution and requirements, we need to implement client-side translation that works dynamically without page reload. The constitution specifies using googletrans/HF model for Urdu translation (NO GPT). This approach will allow us to translate content on the fly while preserving code blocks, links, and formatting.

**Alternatives considered**:
1. Server-side translation - Would require page reload and doesn't meet requirements
2. Pre-translated content - Would not be dynamic and would require significant storage
3. Google Translate API - Violates constitution requirement (NO GPT) and would be costly
4. Client-side googletrans library - Best fits requirements for dynamic, client-side translation

## Decision: Translation State Management
**Rationale**: To maintain chapter-specific translation states and toggle functionality, we'll use browser localStorage/sessionStorage to store the translation state per chapter. This meets the requirement to preserve state per chapter during the session while allowing other chapters to load in default language.

**Alternatives considered**:
1. URL parameters - Would change the URL, violating requirements
2. Global state management - Would affect all chapters, not just current one
3. In-memory storage - Would not persist across navigation

## Decision: Content Selection Strategy
**Rationale**: To translate only text content while preserving code blocks, links, and formatting, we'll use DOM traversal to identify and select only text nodes within chapter content, excluding pre, code, and link elements.

**Alternatives considered**:
1. Full HTML translation - Would affect code blocks and links
2. Markdown-level translation - Would require page reload
3. CSS-based approach - Not feasible for translation

## Decision: Button Placement and UI
**Rationale**: The translation button should be placed consistently at the top of each chapter page, integrated with the existing Docusaurus theme. This provides easy access while maintaining consistency with the design.

**Alternatives considered**:
1. Floating button - Might interfere with reading experience
2. Context menu - Less discoverable
3. Toolbar integration - More complex implementation