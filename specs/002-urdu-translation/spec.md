# Feature Specification: Urdu Translation

**Feature Branch**: `002-urdu-translation`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Implement on-demand Urdu translation for selected text in a Docusaurus chapter page using the existing backend with Qdrant already connected. When the user selects text and clicks the “Urdu” button, the selected English text must be translated into Urdu and returned. If the user selects text that is already in Urdu and clicks the “Urdu” button, return the same selected text without re-translating it. The frontend should send the selected text to the backend via a fetch call. The FastAPI backend must first check Qdrant metadata for an existing Urdu translation. If found, return it immediately. If not found, call the Gemini LLM to translate the text into Urdu, store the translated result in Qdrant metadata, and return it. The translated (or preserved Urdu) text must be displayed inside the chat interface, following the same interaction pattern as the existing “Ask” feature. The solution must not change routes or reload the page."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Translate selected English text to Urdu (Priority: P1)

A user reads content on a Docusaurus chapter page and selects a portion of English text. When they click the "Urdu" button in the text selection toolbar, the selected text is sent to the backend for translation. The system returns the Urdu translation and displays it in the chat interface. This provides immediate access to Urdu translations of technical content for Urdu-speaking users.

**Why this priority**: This is the core functionality that delivers the primary value of the feature - enabling Urdu-speaking users to understand English content.

**Independent Test**: Can be fully tested by selecting English text, clicking the Urdu button, and verifying that the translated text appears in the chat interface. Delivers the core value of making English content accessible to Urdu speakers.

**Acceptance Scenarios**:

1. **Given** user has selected English text on a chapter page, **When** user clicks the "Urdu" button, **Then** the system translates the text to Urdu and displays it in the chat interface
2. **Given** user has selected English text and clicked the "Urdu" button, **When** translation completes, **Then** the Urdu translation is displayed in the chat interface with appropriate formatting

---

### User Story 2 - Avoid re-translating already translated Urdu text (Priority: P2)

A user selects text that is already in Urdu on a Docusaurus chapter page. When they click the "Urdu" button, the system recognizes the text is already in Urdu and returns the same text without attempting to translate it. This prevents unnecessary processing and maintains the original content.

**Why this priority**: This prevents wasteful processing and ensures that already-translated content is preserved, improving system efficiency.

**Independent Test**: Can be tested by selecting Urdu text, clicking the Urdu button, and verifying that the same text is returned without translation. Ensures system efficiency and proper content handling.

**Acceptance Scenarios**:

1. **Given** user has selected text that is already in Urdu, **When** user clicks the "Urdu" button, **Then** the system returns the same text without translation
2. **Given** user has selected mixed English/Urdu text with majority Urdu, **When** user clicks the "Urdu" button, **Then** the system identifies it as Urdu content and returns the text unchanged

---

### User Story 3 - Cache translations in Qdrant for performance (Priority: P3)

When a translation is requested, the system first checks Qdrant metadata for an existing translation of the same text. If found, it returns the cached translation immediately. If not found, it performs the translation, stores the result in Qdrant metadata, and returns it. This improves performance for repeated requests.

**Why this priority**: This optimizes performance and reduces API calls to translation services, improving user experience and reducing costs.

**Independent Test**: Can be tested by requesting the same translation multiple times and verifying that subsequent requests return faster due to caching. Improves system performance and efficiency.

**Acceptance Scenarios**:

1. **Given** a text translation has been previously requested, **When** the same text is requested again, **Then** the system returns the cached translation from Qdrant immediately
2. **Given** a text translation does not exist in cache, **When** the translation is requested, **Then** the system performs the translation, stores it in Qdrant, and returns the result

---

### User Story 4 - Integrate with existing chat interface (Priority: P1)

The Urdu translation result must be displayed in the existing chat interface following the same interaction pattern as the existing "Ask" feature. This maintains consistency in user experience and leverages existing UI components.

**Why this priority**: This ensures a consistent user experience and leverages existing functionality without requiring new UI development.

**Independent Test**: Can be tested by verifying that translated text appears in the chat interface with the same styling and behavior as responses from the "Ask" feature. Maintains UI consistency.

**Acceptance Scenarios**:

1. **Given** user has requested Urdu translation, **When** translation is received, **Then** it appears in the chat interface with the same styling as "Ask" responses
2. **Given** translation is displayed in chat interface, **When** user interacts with it, **Then** it behaves the same as other chat messages

---

### Edge Cases

- What happens when the selected text is empty or contains only whitespace?
- How does the system handle very long text selections that might exceed API limits?
- What happens when the Gemini LLM API is unavailable or returns an error?
- How does the system handle text that contains both English and Urdu characters?
- What happens when Qdrant is unavailable for checking cached translations?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST provide an "Urdu" button in the text selection toolbar when text is selected on a Docusaurus chapter page
- **FR-002**: System MUST detect when selected text is already in Urdu and return it unchanged without re-translating
- **FR-003**: System MUST send selected text to the backend via a fetch call when the "Urdu" button is clicked
- **FR-004**: Backend MUST check Qdrant metadata for existing Urdu translation before performing new translation
- **FR-005**: Backend MUST call the Gemini LLM to translate English text to Urdu when no cached translation exists
- **FR-006**: Backend MUST store new translations in Qdrant metadata for future retrieval
- **FR-007**: System MUST display translated text in the existing chat interface following the same pattern as the "Ask" feature
- **FR-008**: System MUST handle errors gracefully when translation services are unavailable
- **FR-009**: System MUST preserve the original text when it's already in Urdu
- **FR-010**: System MUST not change routes or reload the page during translation

### Key Entities *(include if feature involves data)*

- **Translation Cache**: Qdrant collection entry that stores English text as input and Urdu translation as output in metadata
- **Text Selection**: User-selected content on a Docusaurus page that serves as input for translation
- **Translation Result**: Urdu output that is displayed in the chat interface

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can translate selected English text to Urdu in under 3 seconds for text under 500 words
- **SC-002**: System returns cached translations in under 1 second when available in Qdrant
- **SC-003**: 95% of translation requests result in successful Urdu output when text is in English
- **SC-004**: System correctly identifies and preserves Urdu text without re-translation 98% of the time
- **SC-005**: User satisfaction rating for translation feature is above 4.0/5.0
- **SC-006**: Translation accuracy for technical AI/Robotics terminology is sufficient for user comprehension
