---
name: auth-signin
description: Authenticate existing users using Better Auth and issue JWT tokens
trigger: User submits login credentials
inputs:
  - email: string (required)
  - password: string (required)
outputs:
  - userId: string
  - jwtToken: string
  - refreshToken: string
  - userProfile: object
validations:
  - Email format validation
  - Credentials verification against Better Auth
error_handling:
  - Invalid credentials: Return 401 Unauthorized
  - Account locked: Return 423 Locked
  - Server error: Return 500 Internal Server Error
---

# Auth Signin Skill

Authenticates users via Better Auth and issues JWT tokens with user context.

## Purpose
Handle user authentication and JWT token issuance with embedded user profile claims.

## Workflow
1. Validate email format
2. Verify credentials via Better Auth
3. Retrieve user profile and background data
4. Generate JWT with user claims (profile completion status, backgrounds)
5. Generate refresh token
6. Return tokens and user profile

## Integration Points
- Better Auth SDK for credential verification
- JWT service for token generation
- Profile manager for profile retrieval
- Neon DB for user data retrieval and auth logging (users, profiles, software_backgrounds, hardware_backgrounds, auth_logs, token_sessions tables)
- Redis for profile caching and refresh token storage
