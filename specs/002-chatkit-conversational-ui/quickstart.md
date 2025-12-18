# Quickstart Guide: ChatKit Component

## Installation

The ChatKit component is already integrated into the Docusaurus project. No additional installation is required.

## Basic Usage

### 1. Import the Component
```javascript
import ChatKit from '../components/ChatKit/ChatKit';
```

### 2. Add to Your Page
```javascript
function MyPage() {
  return (
    <Layout>
      <div>Page content...</div>
      <ChatKit />
    </Layout>
  );
}
```

### 3. With Custom Configuration
```javascript
<ChatKit
  config={{
    title: 'AI Assistant',
    placeholder: 'Ask about AI, Robotics...',
    enablePageContext: true,
    enableSelectedTextContext: true
  }}
/>
```

## Key Features

### Context Capture
- The component automatically captures the current page title
- Selected text is captured when users highlight content on the page
- Context is passed to the response generation for more relevant answers

### Theme Compatibility
- Automatically adapts to Docusaurus light/dark mode
- Uses CSS variables that match the current theme
- No additional configuration needed for theming

### Responsive Design
- Works on mobile, tablet, and desktop
- Adjusts size and layout based on screen dimensions
- Touch-friendly interface for mobile users

## API Integration (Future)

To connect to a real backend, replace the mock service:

```javascript
ChatKit.ChatAPIService.sendUserMessage = async (message, context) => {
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ message, context })
  });
  return response.json();
};
```

## Customization Options

| Config Option | Type | Default | Description |
|---------------|------|---------|-------------|
| title | string | 'AI Assistant' | Title displayed in chat header |
| placeholder | string | 'Ask about AI, Robotics...' | Input field placeholder |
| enablePageContext | boolean | true | Enable page context for responses |
| enableSelectedTextContext | boolean | true | Enable selected text context |

## File Locations

- Component: `frontend/textbook-physical-ai/src/components/ChatKit/ChatKit.js`
- Styles: `frontend/textbook-physical-ai/src/components/ChatKit/ChatKit.css`
- Demo: `frontend/textbook-physical-ai/src/pages/chatkit-demo.js`
- Documentation: `frontend/textbook-physical-ai/src/components/ChatKit/README.md`