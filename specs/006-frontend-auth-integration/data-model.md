# Data Model: Frontend Authentication Flow Enhancement

## Overview

This document outlines the data models relevant to the frontend authentication flow enhancement. The enhancement builds upon the existing authentication data models while adding requirements for persistent session management and dynamic UI state.

## Key Entities

### 1. User Session

**Description**: Represents the authenticated state of a user, including JWT tokens and session validity

**Attributes**:
- `accessToken`: string - JWT access token for API authentication
- `refreshToken`: string - JWT refresh token for token renewal
- `expiresAt`: Date - Expiration timestamp for the access token
- `isValid`: boolean - Flag indicating if the session is still valid
- `userId`: string - Unique identifier for the user
- `userEmail`: string - User's email address

**Relationships**:
- Belongs to: User (via userId)
- Used by: AuthContext, tokenService

**Validation Rules**:
- accessToken must be a valid JWT
- refreshToken must be a valid JWT
- expiresAt must be in the future
- userId must be non-empty

### 2. User Profile Data

**Description**: Information about the authenticated user including name, profile image URL, and authentication method used

**Attributes**:
- `id`: string - Unique identifier for the user
- `name`: string - Full name of the user
- `email`: string - Email address of the user
- `profileImage`: string - URL to user's profile image (may be from OAuth provider)
- `authMethod`: string - Authentication method used (email/password, Google OAuth, etc.)
- `firstName`: string - First name of the user (for avatar display)
- `lastName`: string - Last name of the user

**Relationships**:
- Part of: User Session
- Retrieved from: Backend API / user database

**Validation Rules**:
- name must be non-empty
- email must be a valid email format
- profileImage must be a valid URL or null
- authMethod must be one of: 'email', 'google', 'oauth'

### 3. Authentication State

**Description**: Boolean indicator of whether user is currently authenticated, used for UI rendering and route protection

**Attributes**:
- `isAuthenticated`: boolean - Whether the user is currently authenticated
- `isLoading`: boolean - Whether authentication status is being determined
- `user`: User Profile Data - Profile information of the authenticated user (null if not authenticated)
- `error`: string - Error message if authentication failed

**Relationships**:
- Contains: User Profile Data (when authenticated)
- Used by: AuthContext, useAuth hook, Protected routes

**Validation Rules**:
- When isAuthenticated is true, user must be non-null
- When isAuthenticated is false, user must be null
- isLoading and error should not both be true simultaneously

### 4. Navbar State

**Description**: State representation for the navbar UI, determining whether to show login buttons or user avatar

**Attributes**:
- `viewType`: string - Type of view to show ('login-buttons' or 'user-avatar')
- `userAvatar`: string - URL or character to display in avatar
- `userName`: string - Display name for the user in dropdown
- `dropdownOpen`: boolean - Whether the user dropdown menu is open

**Relationships**:
- Derived from: Authentication State
- Used by: AuthNavbarItem component

**Validation Rules**:
- viewType must be one of: 'login-buttons', 'user-avatar'
- When viewType is 'user-avatar', userAvatar must be non-empty
- When viewType is 'login-buttons', userAvatar and userName should be null

## State Transitions

### 1. Authentication State Transitions

```
UNAUTHENTICATED → AUTHENTICATING → AUTHENTICATED → LOGGING_OUT → UNAUTHENTICATED
     ↑                                    ↓
     └──────────────── LOGOUT ────────────┘
```

**Transitions**:
- `login()`: UNAUTHENTICATED → AUTHENTICATING → AUTHENTICATED (on success)
- `logout()`: AUTHENTICATED → LOGGING_OUT → UNAUTHENTICATED
- `tokenRefresh()`: AUTHENTICATED → AUTHENTICATED (with new tokens)
- `error()`: Any state → UNAUTHENTICATED (with error state)

### 2. Navbar State Transitions

```
SHOW_LOGIN_BUTTONS ↔ SHOW_USER_AVATAR
```

**Transitions**:
- `authenticationChanged()`: Triggers transition based on Authentication State
- `toggleDropdown()`: SHOW_USER_AVATAR (dropdown closed) ↔ SHOW_USER_AVATAR (dropdown open)
- `closeDropdown()`: SHOW_USER_AVATAR (dropdown open) → SHOW_USER_AVATAR (dropdown closed)

## API Contracts

### 1. Token Storage Interface

**Methods**:
- `storeTokens(accessToken, refreshToken)`: Store tokens in browser storage
- `getTokens()`: Retrieve tokens from browser storage
- `clearTokens()`: Remove tokens from browser storage
- `isTokenValid(token)`: Check if token is valid and not expired

**Validation**:
- All methods require valid JWT format
- Storage methods must handle browser storage limitations
- Clear methods must remove all related data

### 2. User Profile Interface

**Methods**:
- `fetchUserProfile()`: Retrieve user profile data from backend
- `updateUserProfile(profileData)`: Update user profile on backend
- `getLocalProfile()`: Retrieve cached profile data from frontend
- `clearLocalProfile()`: Remove cached profile data

**Validation**:
- Profile data must match User Profile Data schema
- Update operations require authentication
- Local cache must be invalidated when tokens are cleared

## Constraints

1. **Security**: Tokens must never be exposed to non-secure contexts
2. **Persistence**: Session state must survive page refreshes within token validity period
3. **Consistency**: Navbar UI must always reflect current authentication state
4. **Performance**: State updates must complete within 100ms for UI responsiveness
5. **Synchronization**: Multiple tabs must maintain consistent authentication state