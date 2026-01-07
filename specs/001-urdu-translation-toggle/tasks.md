# Tasks: Urdu Translation Toggle

**Feature**: Urdu Translation Toggle
**Branch**: 001-urdu-translation-toggle
**Spec**: [specs/001-urdu-translation-toggle/spec.md](file:///mnt/d/Dev_Workspace/GIAIC/Spec-Kit-Plus/Hackathon AI TextBook/specs/001-urdu-translation-toggle/spec.md)
**Plan**: [specs/001-urdu-translation-toggle/plan.md](file:///mnt/d/Dev_Workspace/GIAIC/Spec-Kit-Plus/Hackathon AI TextBook/specs/001-urdu-translation-toggle/plan.md)
**Input**: Feature specification with 3 prioritized user stories (P1, P2, P3)

## Implementation Strategy

**MVP Scope**: User Story 1 (Toggle Chapter Content Translation) - Basic translation button with toggle functionality that translates text content while preserving code blocks and links.

**Delivery Approach**:
- Phase 1: Setup dependencies and basic infrastructure
- Phase 2: Foundational components (translation service, state management)
- Phase 3: User Story 1 (Core translation functionality)
- Phase 4: User Story 2 (Chapter-specific state management)
- Phase 5: User Story 3 (Dynamic behavior verification)
- Phase 6: Polish and cross-cutting concerns

## Dependencies

- User Story 2 depends on User Story 1 (needs core translation functionality)
- User Story 3 is independent but builds on core functionality

## Parallel Execution Examples

- Translation service and hook can be developed in parallel [P]
- Translation button component and CSS can be developed in parallel [P]
- Tests can be written in parallel with implementation [P]

---

## Phase 1: Setup

**Goal**: Initialize project dependencies and basic component structure

- [X] T001 Set up googletrans dependency in frontend package.json
- [X] T002 Create TranslationButton component directory structure: frontend/src/components/TranslationButton/
- [X] T003 Create translation service directory: frontend/src/services/
- [X] T004 Create hooks directory: frontend/src/hooks/
- [X] T005 Create utils directory: frontend/src/utils/

---

## Phase 2: Foundational Components

**Goal**: Create core services and utilities needed by all user stories

- [X] T006 [P] Create translation service: frontend/src/services/translationService.js
- [X] T007 [P] Create DOM utilities: frontend/src/utils/domUtils.js
- [X] T008 [P] Create useTranslation hook: frontend/src/hooks/useTranslation.js
- [X] T009 [P] Create basic TranslationButton component: frontend/src/components/TranslationButton/TranslationButton.js
- [X] T010 [P] Create TranslationButton styles: frontend/src/components/TranslationButton/TranslationButton.module.css

---

## Phase 3: User Story 1 - Toggle Chapter Content Translation (P1)

**Goal**: Implement core functionality to translate chapter content and toggle between languages

**Independent Test**: Can be fully tested by clicking the Translation button and verifying that text content is translated while code blocks and links remain unchanged. The toggle functionality should work both ways (English to Urdu and Urdu to English).

- [X] T011 [US1] Implement DOM traversal to identify text nodes in chapter content (excluding code, pre, a tags)
- [X] T012 [US1] Implement googletrans integration for Urdu translation in translationService
- [X] T013 [US1] Implement content preservation logic to maintain original code blocks, links, and formatting
- [X] T014 [US1] Implement toggle functionality in useTranslation hook to switch between original and translated content
- [X] T015 [US1] Connect TranslationButton to useTranslation hook and implement UI state
- [X] T016 [US1] Add button to chapter pages (integrate with Docusaurus theme)
- [X] T017 [US1] Test acceptance scenario 1: English content translates to Urdu while preserving code blocks and links
- [X] T018 [US1] Test acceptance scenario 2: Urdu content toggles back to English when button clicked again

---

## Phase 4: User Story 2 - Preserve Chapter-Specific Translation State (P2)

**Goal**: Implement chapter-specific translation state management that persists across navigation

**Independent Test**: Can be tested by translating one chapter, navigating to another chapter, and verifying that the second chapter loads in the default language. Then return to the first chapter and confirm the translation state is preserved.

- [X] T019 [US2] Implement chapter ID detection to identify current chapter
- [X] T020 [US2] Implement localStorage/sessionStorage management for per-chapter translation state
- [X] T021 [US2] Update useTranslation hook to store translation state per chapter
- [X] T022 [US2] Implement logic to load default language when navigating to different chapters
- [X] T023 [US2] Implement logic to restore translation state when returning to previously translated chapters
- [X] T024 [US2] Test acceptance scenario 1: translated chapter navigates to different chapter, new chapter loads in default language
- [X] T025 [US2] Test acceptance scenario 2: return to translated chapter preserves translation state

---

## Phase 5: User Story 3 - Dynamic Translation Without Page Reload (P3)

**Goal**: Ensure translation happens dynamically without page reload or URL changes

**Independent Test**: Can be tested by clicking the Translation button and verifying that the page does not reload, the URL remains the same, and all interactive elements continue to function properly.

- [X] T026 [US3] Verify translation occurs without page reload or URL change
- [X] T027 [US3] Test that all existing interactive elements continue to function after translation
- [X] T028 [US3] Verify URL remains unchanged during translation process
- [X] T029 [US3] Test performance meets requirement (translation completes under 3 seconds for chapters up to 5000 words)
- [X] T030 [US3] Test acceptance scenario 1: translation happens without page reload and URL remains unchanged

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Handle edge cases, error conditions, and finalize implementation

- [X] T031 Implement error handling for translation failures (FR-009)
- [X] T032 Add loading states during translation process
- [X] T033 Implement rate limiting to handle rapid button clicks
- [X] T034 Add error message display when translation API is unavailable
- [X] T035 Handle special characters and mixed languages in content
- [X] T036 Test with large text content to verify performance
- [X] T037 Handle session timeout scenarios
- [X] T038 Update button accessibility attributes and ARIA labels
- [X] T039 Add analytics/event tracking for translation usage
- [X] T040 Create unit tests for translationService: frontend/tests/services/translationService.test.js
- [X] T041 Create component tests for TranslationButton: frontend/tests/components/TranslationButton.test.js
- [X] T042 Run integration tests across all chapter pages
- [X] T043 Update documentation with usage instructions
- [X] T044 Perform final validation against all functional requirements (FR-001 through FR-009)
- [X] T045 Verify success criteria are met (SC-001 through SC-005)