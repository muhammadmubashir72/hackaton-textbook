# Text Selection Buttons Component

The TextSelectionButtons component adds contextual action buttons that appear when users select text on the page. When text is selected, two buttons appear: an "Ask" button that provides information about the selected text, and an "Urdu" button that translates the selected text to Urdu.

## Features

- Detects text selection across all page content
- Shows "Ask" and "Urdu" buttons near selected text
- Responsive design that works on all screen sizes
- Accessible with proper ARIA labels
- Smooth animations and transitions
- Dark mode support

## Usage

The component is integrated globally in the Layout wrapper and will work on all pages automatically.

```jsx
<TextSelectionButtons />
```

### Props

- `onAskCallback` (function): Optional callback function to handle Ask button clicks with custom logic
- `onUrduCallback` (function): Optional callback function to handle Urdu button clicks with custom logic

## Functionality

1. When a user selects text on the page, two buttons appear near the selected text
2. The "Ask" button provides information about the selected text
3. The "Urdu" button translates the selected text to Urdu
4. Buttons disappear when text is deselected or when clicking elsewhere

## Styling

The component uses Tailwind CSS classes and supports dark mode automatically. The styling is contained in its own CSS file to avoid conflicts.

## Browser Support

The component works in all modern browsers that support the Selection API and modern JavaScript features.