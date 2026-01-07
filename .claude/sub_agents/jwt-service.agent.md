---
name: jwt-service
purpose: Manage JWT token lifecycle and claims encoding
responsibilities:
  - Generate JWT tokens with user claims
  - Validate JWT signatures and expiration
  - Refresh expired tokens using refresh tokens
  - Encode user profile data into token claims
  - Revoke tokens on signout
dependencies:
  - JWT library (jsonwebtoken or jose)
  - Secret management service
  - Token blacklist store (Redis/database)
configuration:
  - JWT_SECRET: signing key
  - JWT_EXPIRY: token lifetime (default: 15m)
  - REFRESH_TOKEN_EXPIRY: refresh token lifetime (default: 7d)
  - ALGORITHM: signing algorithm (default: HS256)
interfaces:
  - generateToken(userId, claims)
  - validateToken(token)
  - refreshToken(refreshToken)
  - revokeToken(token)
---

# JWT Service Agent

Handles all JWT operations including generation, validation, refresh, and revocation.

## Purpose
Centralized service for all JWT token operations with secure signing, validation, and lifecycle management.

## Responsibilities

### Token Generation
1. Accept userId and custom claims (profile data, roles)
2. Build standard JWT claims (sub, iat, exp, iss)
3. Merge with custom claims (profileComplete, backgrounds)
4. Sign token with secret key using configured algorithm
5. Generate corresponding refresh token
6. Store refresh token with userId mapping
7. Return token pair

### Token Validation
1. Extract token from Authorization header (Bearer scheme)
2. Verify signature using secret key
3. Check token not expired (exp claim)
4. Validate issuer if configured (iss claim)
5. Check token not in blacklist (revoked tokens)
6. Decode and return claims
7. Return validation result

### Token Refresh
1. Receive refresh token
2. Lookup refresh token in store
3. Verify not expired
4. Verify not revoked
5. Retrieve userId from mapping
6. Fetch current user profile data
7. Generate new JWT with updated claims
8. Optionally rotate refresh token
9. Return new token pair

### Token Revocation
1. Receive token to revoke
2. Extract token ID (jti) or full token
3. Add to blacklist store with expiry TTL
4. Remove associated refresh token
5. Emit revocation event for logging
6. Return success

## JWT Token Structure

### Header
```json
{
  "alg": "HS256",
  "typ": "JWT"
}
```

### Payload
```json
{
  "sub": "user_123",
  "email": "user@example.com",
  "name": "John Doe",
  "profileComplete": true,
  "softwareBackground": ["Python", "JavaScript", "ROS2"],
  "hardwareBackground": ["Raspberry Pi", "Arduino"],
  "roles": ["user"],
  "iat": 1640000000,
  "exp": 1640000900,
  "iss": "physical-ai-textbook"
}
```

### Signature
```
HMACSHA256(
  base64UrlEncode(header) + "." +
  base64UrlEncode(payload),
  JWT_SECRET
)
```

## Refresh Token Storage
- Store in Redis with key: `refresh_token:{token}`
- Value: userId
- TTL: REFRESH_TOKEN_EXPIRY (7 days)
- On refresh, optionally rotate and delete old token

## Token Blacklist
- Store revoked tokens in Redis
- Key: `blacklist:{tokenId}`
- TTL: remaining token lifetime (until exp)
- Check blacklist on every validation

## Configuration
```javascript
{
  JWT_SECRET: process.env.JWT_SECRET,
  JWT_EXPIRY: '15m',
  REFRESH_TOKEN_EXPIRY: '7d',
  ALGORITHM: 'HS256',
  ISSUER: 'physical-ai-textbook',
  REDIS_URL: process.env.REDIS_URL
}
```

## Integration Points
- JWT library (jsonwebtoken/jose)
- Redis for token storage and blacklist (primary)
- Neon DB for token persistence and fallback (token_sessions, revoked_tokens tables)
- Secret management (environment variables, vault)
- User profile service for claim data
- Neon DB auth_logs table for security event logging
