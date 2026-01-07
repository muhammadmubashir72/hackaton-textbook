# Interactive Digital Textbook System - Agent Architecture Overview

## System Components

### Core Agents
1. **Language Management Agent (LMA)** - Manages language preferences using Next.js i18n and content switching
2. **Text Selection Agent (TSA)** - Handles text selection and floating menu management
3. **Content Processing Agent (CPA)** - Processes translation and meaning lookup requests with locale awareness
4. **Interaction Tracking Agent (ITA)** - Tracks interactions and monitors usage patterns

### Core Skills
1. **Language Persistence Skill** - Manages language preference storage with Next.js i18n synchronization
2. **Text Selection Detection Skill** - Captures text selections and coordinates
3. **Translation Processing Skill** - Handles text translation between English and Urdu with locale validation
4. **Meaning Lookup Skill** - Generates contextual explanations for selected text in current locale
5. **Interaction Tracking Skill** - Logs and analyzes user interactions with rate limiting

## System Architecture

### High-Level Flow
1. User interacts with language toggle in navbar → LMA processes language change via Next.js router
2. User selects text in document → TSA detects selection and shows floating menu
3. User chooses "Ask Meaning" or "Translate" → TSA forwards to CPA for processing
4. CPA processes request using appropriate skill with locale awareness → Returns result to TSA
5. ITA tracks interaction and validates usage patterns → Updates interaction statistics

### Communication Layer
- **Event Bus**: Centralized communication for language changes and user actions
- **Direct API Calls**: Synchronous communication between agents and skills
- **Next.js Router**: Locale management and route updates via i18n
- **Database**: Persistent storage for user profiles and interaction history

### Data Flow
- Language preferences flow from LMA to all other agents via Next.js i18n
- Text selections flow from TSA to CPA for processing
- Processing results flow from CPA back to TSA for display
- Interaction events flow from TSA to ITA for tracking

## Implementation Guidelines

### Agent Development
- Each agent should have clear, single responsibility
- Agents communicate via standardized protocols
- State management should be encapsulated within each agent
- Error handling should be comprehensive and graceful

### Skill Development
- Skills should be reusable across multiple agents
- Skills should have clear API contracts
- Skills should handle their own error states
- Skills should be configurable for different environments

### Performance Considerations
- Implement caching for frequently accessed data
- Use debouncing for text selection events
- Optimize API calls to external services with locale headers
- Minimize DOM manipulations

### Security Considerations
- Sanitize all user inputs before processing
- Implement proper authentication for API calls
- Protect against injection attacks
- Validate all data before storage
- Secure Next.js i18n configuration

## Integration Points

### UI Integration
- Language toggle in navbar using Next.js i18n
- Text selection detection across all content
- Floating menu positioning
- Locale-aware content rendering

### Backend Integration
- Next.js i18n routing and locale management
- Translation API services with locale headers
- LLM for meaning lookup with locale awareness
- User profile management with locale preferences
- Interaction tracking database

### Third-Party Services
- Translation providers (Google Translate, Azure Translator) with locale support
- LLM providers (GPT, Claude) with locale-specific responses
- Analytics services for usage tracking
- Next.js framework with i18n capabilities

This architecture provides a scalable, maintainable solution for the interactive digital textbook with Next.js i18n-based language switching and contextual text processing capabilities.