---
name: jwt-management
description: Generate, validate, and refresh JWT tokens with user claims
trigger: Token generation, validation, or refresh request
inputs:
  - operation: enum (generate, validate, refresh)
  - userId: string (for generate)
  - token: string (for validate/refresh)
  - refreshToken: string (for refresh)
outputs:
  - jwtToken: string
  - refreshToken: string
  - claims: object
  - isValid: boolean
claims_structure:
  - sub: userId
  - email: user email
  - profileComplete: boolean
  - softwareBackground: array
  - hardwareBackground: array
  - iat: issued at timestamp
  - exp: expiration timestamp
validations:
  - Token signature verification
  - Token expiration check
  - Refresh token validity
---

# JWT Management Skill

Manages JWT lifecycle including generation with user profile claims, validation, and refresh operations.

## Purpose
Centralize all JWT operations including token generation with custom claims, signature validation, expiration checks, and token refresh.

## Operations

### Generate
1. Accept userId and user claims
2. Build JWT payload with standard and custom claims
3. Sign token with secret key
4. Generate refresh token
5. Store refresh token in secure store
6. Return both tokens

### Validate
1. Extract token from request
2. Verify signature using secret key
3. Check expiration timestamp
4. Validate required claims present
5. Return validation result and decoded claims

### Refresh
1. Verify refresh token validity
2. Check refresh token not revoked
3. Retrieve user data
4. Generate new JWT with updated claims
5. Optionally rotate refresh token
6. Return new token pair

## Integration Points
- JWT library (jsonwebtoken/jose)
- Secret management service
- Redis for token blacklist and refresh token storage (primary)
- Neon DB for token persistence and fallback (token_sessions, revoked_tokens tables)
- User profile service for claim data
