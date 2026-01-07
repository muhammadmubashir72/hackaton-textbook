# Research: Frontend Authentication Flow Enhancement

## Overview

This research document outlines the current authentication implementation and the approach for enhancing the frontend authentication flow to meet the requirements specified in the feature specification.

## Current Authentication Implementation

### 1. Frontend Authentication Components

- **`/frontend/src/services/authAPI.js`** - Axios client with interceptors for token management
- **`/frontend/src/services/tokenService.js`** - Token storage/retrieval with session/local storage
- **`/frontend/src/context/AuthContext.jsx`** - Global auth state management
- **`/frontend/src/hooks/useAuth.js`** - Auth context hook
- **`/frontend/src/theme/AuthNavbarItem.js`** - Navbar auth component
- **`/frontend/src/pages/profile.js`** - Profile management page
- **`/frontend/src/pages/auth-callback.js`** - OAuth callback handler

### 2. Backend Authentication Services

- **`/auth-backend/dist/routes/auth.routes.js`** - Main authentication routes
- **`/auth-backend/dist/services/jwt.service.js`** - JWT token handling
- **`/auth-backend/dist/services/auth-simple.service.js`** - User management
- **`/auth-backend/migrations/001_create_core_schema.sql`** - Database schema
- **`/auth-backend/src/routes/oauth.routes.ts`** - OAuth integration

### 3. Token Management Strategy

The system currently implements a secure token management strategy:
- **Access Token**: Stored in `sessionStorage` (cleared on tab close) for XSS protection
- **Refresh Token**: Stored in `localStorage` (persists across sessions)
- Token refresh handled automatically via axios interceptors
- Multi-tab synchronization via storage events

### 4. User Profile and Avatar Support

The system has comprehensive user profile support:
- Database includes `image` column in `users` table for profile pictures
- Google OAuth integration stores profile pictures from `picture` field
- Navbar displays user's first initial in a colored circle when no image exists
- Profile page supports image upload with preview functionality

### 5. Navbar Implementation

The navbar is currently implemented as:
- **`/frontend/src/theme/AuthNavbarItem.js`** - Shows login/signup buttons when unauthenticated, user dropdown when authenticated
- Configured in `/frontend/docusaurus.config.js` with `type: 'custom-auth'` at position 'right'

### 6. Protected Routes

The system uses:
- `ProtectedFeature` component for feature-level protection
- `useAuth` hook to check authentication status
- Context-based authentication state management

## Technical Approach for Enhancement

### 1. Persistent Session Management (FR-001, FR-011)

The current implementation already supports JWT token storage in browser storage. The existing token service handles both access and refresh tokens with appropriate persistence strategies. This meets the requirements for persistent sessions across page refreshes.

### 2. Dynamic Navbar Updates (FR-003, FR-004)

The existing `AuthNavbarItem.js` component already has logic to switch between authenticated and unauthenticated states. The enhancement will involve:

- Modifying the authenticated state display to show user avatar instead of generic dropdown
- Implementing avatar rendering that shows Google profile image for OAuth users or first initial for email/password users
- Adding the required dropdown menu options: Profile, Dashboard/Progress, Settings, Logout

### 3. Dropdown Menu Implementation (FR-005, FR-006)

The dropdown menu needs to include:
- Profile link (navigate to profile page)
- Dashboard/Progress link (placeholder for future implementation)
- Settings link (placeholder for future implementation)
- Logout option with proper functionality

The dropdown should close on outside clicks, which can be implemented using click-outside detection.

### 4. Logout Functionality (FR-007, FR-008)

The current logout functionality in the AuthContext already clears tokens and resets state. The enhancement will ensure:
- All auth tokens are cleared from storage
- User state is reset
- Navigation to home page after logout
- Navbar UI updates to show login/signup buttons

### 5. Route Protection (FR-009)

The existing protected route mechanism using `ProtectedFeature` component already redirects unauthenticated users to the sign-in screen. This meets the requirement for protected route navigation.

## Implementation Decisions

### 1. Token Storage Strategy

**Decision**: Continue using the existing dual-storage approach (sessionStorage for access tokens, localStorage for refresh tokens)

**Rationale**: This approach balances XSS protection (sessionStorage clears on tab close) with persistent sessions (localStorage persists across browser restarts). It follows JWT security best practices.

**Alternatives considered**:
- Cookie-based storage: More complex to implement with current architecture
- Single storage method: Would compromise either security or persistence

### 2. Avatar Rendering Strategy

**Decision**: Use Google profile image for OAuth users, first initial in colored circle for email/password users

**Rationale**: This matches the existing implementation and provides visual consistency with the current design.

**Alternatives considered**:
- Always use generic avatar: Would not leverage OAuth profile images
- Complex avatar customization: Would add unnecessary complexity

### 3. Navbar Component Architecture

**Decision**: Enhance the existing `AuthNavbarItem.js` component rather than creating a new component

**Rationale**: This maintains consistency with the current architecture and preserves existing functionality while adding the required features.

**Alternatives considered**:
- New component: Would require more extensive refactoring
- Separate dropdown component: Would add complexity without clear benefit

## Potential Challenges

1. **State Synchronization**: Ensuring the navbar updates correctly when authentication state changes across tabs
2. **Responsive Design**: Making sure the avatar dropdown works well on mobile devices
3. **Backward Compatibility**: Maintaining existing functionality while adding new features
4. **Performance**: Ensuring navbar updates happen quickly (<100ms) as required by success criteria

## Research Conclusion

The current authentication implementation provides a solid foundation for the requested enhancements. The main areas that need modification are the navbar UI and dropdown functionality, while the underlying token management and authentication logic can be leveraged as-is. The implementation should be straightforward with minimal risk to existing functionality.