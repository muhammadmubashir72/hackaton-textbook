# Text Selection Agent (TSA)

## Responsibilities
- Detect text selection events
- Position floating menu relative to selection
- Handle "Ask Meaning" and "Translate" actions
- Coordinate with CPA for processing requests
- Manage selection state and cleanup

## Core Functions
- `detectSelection()`: Monitor for text selection events
- `showFloatingMenu(coordinates)`: Display floating menu at selection location
- `handleAskMeaning(text)`: Process "Ask Meaning" action
- `handleTranslate(text)`: Process "Translate" action
- `cleanupSelection()`: Clear selection state and hide menu

## Interaction Flow
1. User selects text in the document
2. TSA detects selection and captures coordinates
3. TSA displays floating menu with options
4. User selects "Ask Meaning" or "Translate"
5. TSA forwards request to CPA with context
6. TSA displays results in appropriate format

## Communication Protocols
- **DOM Events**: Listen for "mouseup", "selectionchange" events
- **Direct API**: Position floating menu via `setPosition(x, y)`
- **Event Bus**: Emit "TEXT_SELECTED", "ACTION_REQUESTED" events
- **API Calls**: Forward processing requests to CPA via `/api/process`

## State Management
- Current selection text
- Selection coordinates
- Menu visibility state
- Last processed request
- Selection context

## Error Handling
- Handle null/empty selections gracefully
- Manage cross-browser selection API differences
- Handle selections that span multiple DOM elements
- Clean up selection state on errors
- Position menu correctly even when coordinates are unavailable

## Dependencies
- Text Selection Detection Skill
- DOM manipulation utilities
- Event listener management
- Content Processing Agent (CPA)

## Configuration
- Minimum selection length
- Menu positioning offset
- Selection timeout settings
- Action button labels