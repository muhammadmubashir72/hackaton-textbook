# Text Selection Detection Skill

## Purpose
Capture user text selections with coordinates and manage selection state in the interactive digital textbook.

## Implementation
- `window.getSelection()` with mouseup event listeners
- Coordinate calculation for floating menu positioning
- Selection range validation and cleanup

## API
- `getSelectedText()`: Get currently selected text
- `getSelectionCoordinates()`: Get coordinates of selection for menu positioning
- `clearSelection()`: Clear current text selection
- `isValidSelection()`: Validate if selection meets minimum requirements
- `getSelectionContext()`: Get surrounding context of selection

## Parameters
- None for getter methods
- `minLength` (optional): Minimum text length for valid selection

## Return Values
- `getSelectedText`: String of selected text
- `getSelectionCoordinates`: Object with x, y, width, height coordinates
- `clearSelection`: Boolean success status
- `isValidSelection`: Boolean indicating if selection is valid
- `getSelectionContext`: Object with surrounding text context

## Error Handling
- Handle null/empty selections gracefully
- Manage cross-browser selection API differences
- Handle selections that span multiple DOM elements
- Clean up selection state on errors

## Dependencies
- Browser Selection API
- DOM manipulation methods
- Event listener management