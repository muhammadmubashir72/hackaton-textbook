# Implementation Tasks: Urdu Translation

**Feature**: Urdu Translation for selected text in Docusaurus pages
**Branch**: `002-urdu-translation`
**Spec**: [specs/002-urdu-translation/spec.md](specs/002-urdu-translation/spec.md)

## Task List

### [FR-001] Add Urdu Button to Text Selection Component
**Priority**: P1
**Component**: frontend/src/components/TextSelectionButtons/TextSelectionButtons.js
**Effort**: Small

**Description**: Add "Urdu" button to the existing text selection toolbar that appears when users select text on Docusaurus pages.

**Implementation Steps**:
1. Add Urdu button (with globe icon) next to the existing "Ask" button
2. Implement button click handler that captures the selected text
3. Ensure button styling matches existing design language
4. Add appropriate aria-label and title attributes for accessibility

**Acceptance Criteria**:
- [ ] Urdu button appears when text is selected
- [ ] Button has appropriate icon and label ("اردو")
- [ ] Button click captures selected text
- [ ] Button follows existing styling patterns

**Dependencies**: None

### [FR-003] Implement Frontend Translation Request
**Priority**: P1
**Component**: frontend/src/components/TextSelectionButtons/TextSelectionButtons.js
**Effort**: Medium

**Description**: When the Urdu button is clicked, send the selected text to the backend translation endpoint via fetch API.

**Implementation Steps**:
1. Create fetch request to backend `/translate` endpoint
2. Send selected text in request body with target language "ur"
3. Handle loading state during translation request
4. Dispatch custom event to notify ChatKit component of translation request

**Acceptance Criteria**:
- [ ] Selected text is sent to backend via fetch call
- [ ] Loading state is shown during translation
- [ ] Custom event is dispatched to trigger chat interface update
- [ ] Error handling for failed requests

**Dependencies**: [FR-001], Backend translation endpoint

### [FR-002, FR-009] Implement Urdu Detection Logic
**Priority**: P2
**Component**: backend/app/api_main.py
**Effort**: Medium

**Description**: Add logic to detect if incoming text is already in Urdu and return it unchanged without translation.

**Implementation Steps**:
1. Create function to detect Urdu text using Unicode character range (U+0600 to U+06FF)
2. Check if majority of text characters fall within Arabic/Urdu Unicode range
3. Return original text unchanged if detected as Urdu
4. Add appropriate logging for detection decisions

**Acceptance Criteria**:
- [ ] Function accurately detects Urdu text
- [ ] Urdu text is returned unchanged
- [ ] English text is processed for translation
- [ ] Mixed text with Urdu majority is preserved

**Dependencies**: None

### [FR-004] Extend Retrieval Service for Translation Cache
**Priority**: P2
**Component**: backend/app/services/retrieval_service.py
**Effort**: Medium

**Description**: Add functionality to check Qdrant metadata for cached Urdu translations.

**Implementation Steps**:
1. Add method to search for existing translations in Qdrant metadata
2. Use text content as search key in metadata
3. Return cached translation if found
4. Return null if no cached translation exists

**Acceptance Criteria**:
- [ ] Can search Qdrant metadata for cached translations
- [ ] Returns cached translation when found
- [ ] Returns null when no cached translation exists
- [ ] Proper error handling for Qdrant connectivity issues

**Dependencies**: Qdrant integration

### [FR-005] Implement Gemini Translation Service
**Priority**: P1
**Component**: backend/app/api_main.py
**Effort**: Medium

**Description**: Call Gemini LLM to translate English text to Urdu when no cached translation exists.

**Implementation Steps**:
1. Create function to call Gemini API for translation
2. Format prompt specifically for English-to-Urdu translation
3. Handle API response and extract translated text
4. Add error handling for API failures

**Acceptance Criteria**:
- [ ] Gemini API is called for translation
- [ ] English text is properly translated to Urdu
- [ ] Translation errors are handled gracefully
- [ ] Response is properly formatted

**Dependencies**: Gemini API configuration

### [FR-006] Store Translations in Qdrant Metadata
**Priority**: P2
**Component**: backend/app/api_main.py, backend/app/services/retrieval_service.py
**Effort**: Medium

**Description**: Store newly generated Urdu translations in Qdrant metadata for future retrieval.

**Implementation Steps**:
1. Add method to store translation pairs in Qdrant metadata
2. Create metadata structure with original text and translation
3. Store both original and translated text for future lookups
4. Ensure proper indexing for efficient retrieval

**Acceptance Criteria**:
- [ ] New translations are stored in Qdrant metadata
- [ ] Both original and translated text are stored
- [ ] Translations can be retrieved in subsequent requests
- [ ] Proper error handling for storage failures

**Dependencies**: Qdrant integration, Translation service

### [FR-007] Create Translation API Endpoint
**Priority**: P1
**Component**: backend/app/api_main.py
**Effort**: Medium

**Description**: Create FastAPI endpoint that coordinates translation flow (cache check → translation → storage).

**Implementation Steps**:
1. Create `/translate` POST endpoint with TranslationRequest model
2. Implement translation workflow: check cache → detect language → translate if needed → store if new
3. Return TranslationResponse with cached flag
4. Add proper error handling and validation

**Acceptance Criteria**:
- [ ] `/translate` endpoint accepts text and target language
- [ ] Endpoint follows cache-first workflow
- [ ] Returns proper response format
- [ ] Handles errors appropriately

**Dependencies**: All backend components above

### [FR-007] Update ChatKit to Handle Translation Results
**Priority**: P1
**Component**: frontend/src/components/ChatKit/ChatKit.js
**Effort**: Medium

**Description**: Update ChatKit component to handle and display translation results in the same way as "Ask" feature responses.

**Implementation Steps**:
1. Add event listener for translation result events
2. Update chat interface to display translation results
3. Ensure consistent styling with existing "Ask" responses
4. Add proper loading and error states

**Acceptance Criteria**:
- [ ] Translation results appear in chat interface
- [ ] Results styled consistently with "Ask" responses
- [ ] Loading states work properly
- [ ] Error states are handled

**Dependencies**: Frontend translation request, Backend API endpoint

### [FR-008] Implement Error Handling
**Priority**: P2
**Component**: frontend/src/components/TextSelectionButtons/TextSelectionButtons.js, backend/app/api_main.py
**Effort**: Small

**Description**: Add comprehensive error handling for translation failures.

**Implementation Steps**:
1. Add error handling in frontend for API failures
2. Add error handling in backend for Gemini API failures
3. Provide user-friendly error messages
4. Implement fallback behavior when translation fails

**Acceptance Criteria**:
- [ ] API failures are handled gracefully in frontend
- [ ] Backend errors are properly propagated
- [ ] Users see meaningful error messages
- [ ] System doesn't crash on translation failures

**Dependencies**: All other tasks

### [FR-010] Verify No Route Changes or Page Reloads
**Priority**: P1
**Component**: All components
**Effort**: Small

**Description**: Ensure the translation feature works without changing routes or reloading the page.

**Implementation Steps**:
1. Verify all functionality works via AJAX/fetch calls
2. Confirm no page navigation occurs during translation
3. Test that existing page state is preserved
4. Ensure component states are maintained

**Acceptance Criteria**:
- [ ] No page reloads during translation
- [ ] No route changes during translation
- [ ] Existing page content remains intact
- [ ] Component states are preserved

**Dependencies**: All tasks

## Implementation Order

1. **Phase 1** (Backend foundation): FR-005, FR-002, FR-004, FR-006, FR-007
2. **Phase 2** (Frontend integration): FR-001, FR-003, FR-009
3. **Phase 3** (UI integration): FR-007 (frontend part), FR-008, FR-010

## Success Metrics

- [ ] Users can translate selected English text to Urdu in under 3 seconds
- [ ] Cached translations return in under 1 second
- [ ] 95% of translation requests succeed
- [ ] Urdu text is correctly identified and preserved 98% of the time
- [ ] No route changes or page reloads occur during translation