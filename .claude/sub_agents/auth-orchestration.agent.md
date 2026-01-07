---
name: auth-orchestration
purpose: Coordinate signup and signin flows with Better Auth integration
responsibilities:
  - Initiate Better Auth client for authentication operations
  - Coordinate between authentication, JWT issuance, and profile operations
  - Handle authentication state transitions
  - Manage session lifecycle
dependencies:
  - Better Auth SDK
  - JWT management skill
  - Profile collection skill
interfaces:
  - POST /auth/signup
  - POST /auth/signin
  - POST /auth/signout
  - POST /auth/refresh
state_management:
  - Track authentication status
  - Maintain session tokens
  - Coordinate profile completion status
---

# Auth Orchestration Agent

Orchestrates all authentication workflows using Better Auth and coordinates with JWT and profile management.

## Purpose
Central orchestrator for all authentication flows, managing the coordination between Better Auth, JWT services, and user profile operations.

## Responsibilities

### Signup Flow
1. Receive signup request (email, password, name)
2. Validate input data
3. Check email uniqueness
4. Create user via Better Auth SDK
5. Initialize user profile (empty)
6. Generate JWT with profileComplete: false
7. Return tokens and userId
8. Redirect to profile completion

### Signin Flow
1. Receive signin request (email, password)
2. Validate credentials via Better Auth
3. Retrieve user profile data
4. Generate JWT with user claims
5. Create refresh token
6. Return tokens and user data

### Signout Flow
1. Receive signout request with JWT
2. Validate token
3. Revoke JWT (add to blacklist)
4. Invalidate refresh token
5. Clear Better Auth session
6. Return success

### Token Refresh Flow
1. Receive refresh token
2. Validate refresh token
3. Check token not revoked
4. Retrieve updated user profile
5. Generate new JWT with current claims
6. Optionally rotate refresh token
7. Return new token pair

## State Transitions
```
[Unauthenticated] --signup--> [Authenticated, Profile Incomplete]
[Authenticated, Profile Incomplete] --complete profile--> [Authenticated, Profile Complete]
[Unauthenticated] --signin--> [Authenticated, Profile Complete/Incomplete]
[Authenticated] --signout--> [Unauthenticated]
[Authenticated] --refresh--> [Authenticated]
```

## Integration Points
- Better Auth SDK for user management
- JWT management skill for token operations
- Profile collection skill for profile operations
- Neon DB for user storage and audit logging (users, profiles, auth_logs, audit_logs tables)
- Redis for refresh token caching
- Neon DB token_sessions table for refresh token persistence and fallback

## Configuration
```json
{
  "betterAuth": {
    "baseUrl": "http://localhost:3000",
    "secret": "env.BETTER_AUTH_SECRET"
  },
  "jwt": {
    "secret": "env.JWT_SECRET",
    "expiresIn": "15m",
    "refreshExpiresIn": "7d"
  }
}
```
