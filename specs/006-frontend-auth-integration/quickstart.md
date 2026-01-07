# Quickstart: Frontend Authentication Flow Enhancement

## Overview

This guide provides a quick overview of the enhanced frontend authentication flow with persistent JWT tokens, dynamic navbar updates, and protected route navigation. This implementation builds upon the existing better-auth system with improved user experience and security.

## Key Components

### 1. Authentication Context
- **File**: `/frontend/src/context/AuthContext.jsx`
- **Purpose**: Global authentication state management
- **Key Features**:
  - JWT token storage and refresh
  - User profile data management
  - Authentication state synchronization

### 2. Authentication Hook
- **File**: `/frontend/src/hooks/useAuth.js`
- **Purpose**: Access authentication state in components
- **Usage**: `const { user, isAuthenticated, login, logout } = useAuth();`

### 3. Token Service
- **File**: `/frontend/src/services/tokenService.js`
- **Purpose**: Secure token storage and retrieval
- **Storage Strategy**:
  - Access tokens: sessionStorage (XSS protection)
  - Refresh tokens: localStorage (persistent sessions)

### 4. Navbar Component
- **File**: `/frontend/src/theme/AuthNavbarItem.js`
- **Purpose**: Dynamic navbar with user avatar and dropdown
- **Features**:
  - Switches between login buttons and user avatar
  - Dropdown menu with Profile, Dashboard, Settings, Logout

## Implementation Guide

### 1. Setting Up Persistent Sessions

The system automatically handles persistent sessions through the token service:

```javascript
// Tokens are stored automatically after login
await login(credentials);

// Session persists across page refreshes
// Access token is refreshed automatically when needed
```

### 2. Using the Enhanced Navbar

The navbar component automatically updates based on authentication state:

```jsx
// The navbar will show login buttons when unauthenticated
// and user avatar when authenticated
<Navbar />
// No additional setup needed - this works automatically
```

### 3. Protected Routes

Use the existing ProtectedFeature component for route protection:

```jsx
import { ProtectedFeature } from './components/ProtectedFeature';

function App() {
  return (
    <ProtectedFeature>
      <ProtectedPage />
    </ProtectedFeature>
  );
}
```

### 4. Accessing User Data

Access user profile data through the auth context:

```javascript
import { useAuth } from '../hooks/useAuth';

function UserProfile() {
  const { user, isAuthenticated } = useAuth();

  if (isAuthenticated) {
    return (
      <div>
        <img src={user.profileImage} alt="Profile" />
        <span>{user.name}</span>
      </div>
    );
  }
  return <div>Please log in</div>;
}
```

## Key Features

### 1. Dynamic Navbar Updates
- Shows "Sign In/Up" buttons when unauthenticated
- Shows user avatar when authenticated
- Avatar displays Google profile image for OAuth users
- Avatar shows first initial for email/password users

### 2. User Dropdown Menu
- Profile: Navigate to user profile page
- Dashboard/Progress: Placeholder for progress tracking
- Settings: Placeholder for user settings
- Logout: Clear tokens and redirect to home

### 3. Secure Token Management
- Access tokens in sessionStorage (cleared on tab close)
- Refresh tokens in localStorage (persist across sessions)
- Automatic token refresh when needed
- Multi-tab logout synchronization

### 4. Session Persistence
- Maintains login state across page refreshes
- Preserves session until token expiration
- Automatic logout when tokens expire

## Testing the Implementation

### 1. Session Persistence Test
1. Log in to the application
2. Refresh the page
3. Verify that you remain logged in and navbar shows avatar

### 2. Navbar UI Test
1. Verify login buttons show when unauthenticated
2. Log in and verify avatar appears
3. Click avatar and verify dropdown menu appears
4. Click outside menu and verify it closes

### 3. Logout Functionality Test
1. Log in and verify avatar is shown
2. Click logout from dropdown
3. Verify tokens are cleared and navbar shows login buttons
4. Verify redirect to home page

### 4. Protected Route Test
1. Navigate directly to a protected route while unauthenticated
2. Verify redirect to login page

## Troubleshooting

### Common Issues

**Issue**: User not staying logged in after refresh
**Solution**: Check that tokens are properly stored in tokenService and that the AuthContext is properly initialized on app load.

**Issue**: Navbar not updating after login/logout
**Solution**: Ensure AuthContext is properly wrapped around the application and components are using the useAuth hook correctly.

**Issue**: Dropdown not closing on outside click
**Solution**: Verify click-outside detection is properly implemented in the navbar component.

**Issue**: Protected routes not redirecting properly
**Solution**: Check that ProtectedFeature component is properly wrapping the protected content and the authentication state is correctly propagated.

## Security Considerations

1. **Token Storage**: Access tokens are stored in sessionStorage for XSS protection
2. **Token Refresh**: Automatic refresh prevents session expiration during use
3. **Multi-tab Sync**: Logout in one tab triggers logout in all tabs
4. **Session Validation**: Tokens are validated on each page load

## Next Steps

1. Implement Dashboard/Progress and Settings pages for the dropdown menu
2. Add additional profile customization options
3. Implement token refresh notifications for better UX
4. Add loading states for authentication operations