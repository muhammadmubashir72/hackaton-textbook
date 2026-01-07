# Language Management Agent (LMA)

## Responsibilities
- Manage language state (English/Urdu) using Next.js i18n
- Handle UI language toggle in navbar
- Update content based on selected language via Next.js routing
- Persist language preference across sessions using Next.js i18n
- Coordinate with other agents for language-specific operations

## Core Functions
- `initialize()`: Initialize agent with current Next.js router locale state
- `handleLanguageToggle()`: Process language toggle from navbar and update Next.js router
- `updateContent(language)`: Update displayed content via Next.js i18n routing
- `persistPreference(language)`: Save language preference to Next.js router and localStorage
- `broadcastLanguageChange(language)`: Notify other agents of language change

## Interaction Flow
1. User clicks language toggle in navbar
2. LMA updates Next.js router locale via `router.push()`
3. LMA broadcasts language change event
4. Other agents update their UI/content accordingly
5. LMA persists preference via Next.js i18n and localStorage

## Communication Protocols
- **Event Bus**: Emit "LANGUAGE_CHANGED" events with payload {language, timestamp}
- **Next.js Router**: Update locale via `router.push()` with new locale
- **Direct API**: Update content components via `setContentLanguage(language)`

## State Management
- Current Next.js router locale state (en/ur)
- Previous language for comparison
- Preference persistence status
- Last locale change timestamp

## Error Handling
- Handle Next.js router unavailability
- Fallback to default locale if invalid
- Retry locale updates on failure
- Graceful degradation when server unavailable

## Dependencies
- Language Persistence Skill
- Next.js router and i18n
- Event bus system
- Content management system

## Configuration
- Default locale (fallback)
- Supported locales list from Next.js i18n config
- Locale change timeout settings