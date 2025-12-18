# ChatKit Component

A futuristic AI education-themed conversational UI component for Docusaurus websites.

## Features

- **Futuristic Design**: Modern UI with animations and transitions fitting for AI education
- **Responsive**: Works on mobile, tablet, and desktop devices
- **Theming**: Automatically adapts to Docusaurus light/dark mode
- **Context-Aware**: Supports page-based and selected-text context for more relevant responses
- **API Ready**: Clean separation between UI and backend logic for easy integration
- **Accessible**: Full keyboard navigation and screen reader support

## Usage

### Global Integration (Recommended for Docusaurus)

The ChatKit component is designed to be integrated globally using Docusaurus's layout system. Add it to your custom Layout wrapper to make it available on all pages:

```jsx
// In src/theme/Layout/index.js
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatKit from '@site/src/components/ChatKit/ChatKit';

export default function Layout(props) {
  const { children } = props;

  return (
    <>
      <OriginalLayout {...props}>
        {children}
        <ChatKit
          config={{
            title: 'AI Assistant',
            placeholder: 'Ask about AI, Robotics...'
          }}
        />
      </OriginalLayout>
    </>
  );
}
```

### Direct Usage (Alternative)

If you prefer to add the component to individual pages:

```jsx
import ChatKit from './components/ChatKit/ChatKit';

function MyPage() {
  return (
    <Layout>
      <div>Page content...</div>
      <ChatKit
        config={{
          title: 'AI Assistant',
          placeholder: 'Ask about AI, Robotics...'
        }}
      />
    </Layout>
  );
}
```

## Configuration Options

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `config.title` | string | `'AI Assistant'` | Title displayed in the chat header |
| `config.placeholder` | string | `'Ask about AI, Robotics...'` | Placeholder text in the input field |
| `config.enablePageContext` | boolean | `true` | Whether to enable page context for responses |
| `config.enableSelectedTextContext` | boolean | `true` | Whether to enable selected text context |

## API Integration

The component is designed with a clean API service layer for easy backend integration:

```jsx
// To integrate with a real backend, replace the mock service:
import { ChatAPIService } from './components/ChatKit/ChatKit';

// Example of how to connect to a real backend:
ChatAPIService.sendUserMessage = async (message, context) => {
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ message, context })
  });
  return response.json();
};
```

## Text Selection Feature

The component automatically captures text when users select text on the page:

1. User selects text on any page
2. Component captures the selected text and enters "selection mode"
3. Visual indicator appears on the chat button (orange pulsing dot)
4. When user sends a message, the selected text is included as context
5. Responses are context-aware based on the selected text
6. Visual indicators show when in selection mode:
   - Pulsing dot on the chat toggle button
   - "Selected text mode" badge in the chat header
   - Context preview showing the captured text

## Backend Integration

The component includes a single function for backend wiring that can be easily replaced:

```javascript
// The connectToBackend function is currently mocked:
const connectToBackend = async (message, context) => {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(generateMockResponse(message, context));
    }, 800 + Math.random() * 400);
  });
};

// To connect to a real backend, replace this function:
const connectToBackend = async (message, context) => {
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ message, context }) // context includes selectedText
  });
  return response.json();
};
```

## Styling

The component uses Docusaurus CSS variables for theming:
- `var(--ifm-color-primary)` - Primary color
- `var(--ifm-background-color)` - Background color
- `var(--ifm-font-color-base)` - Text color
- `var(--ifm-toc-border-color)` - Border color

These automatically adapt to the current Docusaurus theme (light/dark mode).

## Accessibility

- Full keyboard navigation support
- Proper ARIA labels
- Screen reader friendly
- Focus management
- High contrast support in dark mode