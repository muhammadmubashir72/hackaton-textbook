# Language Persistence Skill

## Purpose
Maintain language preference across sessions and chapters using Next.js i18n mechanism in the interactive digital textbook.

## Implementation
- Next.js i18n routing with locale management
- localStorage fallback for client-side preference
- Synchronization with Next.js router locale state
- Cross-device preference sync via server-side session

## API
- `setLanguage(lang)`: Set current language preference via Next.js router (en/ur)
- `getLanguage()`: Get current language from Next.js router
- `syncWithRouter()`: Synchronize with Next.js i18n router state
- `getStoredPreference()`: Retrieve stored preference from localStorage
- `updateRoute(locale)`: Update route with new locale using Next.js router

## Parameters
- `lang`: Language code ('en' for English, 'ur' for Urdu)
- `locale`: Locale code for Next.js routing

## Return Values
- `setLanguage`: Boolean success status
- `getLanguage`: Current language code from router
- `syncWithRouter`: Sync status object
- `getStoredPreference`: Object with preference sources and values
- `updateRoute`: Promise resolving to route update status

## Error Handling
- Handle Next.js router unavailability
- Fallback to localStorage if router unavailable
- Handle locale validation errors
- Graceful degradation when server unavailable

## Dependencies
- Next.js router and i18n
- Browser localStorage API
- Server-side session management