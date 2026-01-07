---
name: access-control
description: Validate JWT tokens and enforce authorization rules for protected resources
trigger: Incoming request to protected endpoint
inputs:
  - jwtToken: string (required)
  - requestedResource: string (required)
  - requiredPermissions: array of strings
outputs:
  - authorized: boolean
  - userId: string
  - userClaims: object
authorization_rules:
  - Own profile access: userId in JWT matches resource userId
  - Admin access: role claim includes 'admin'
  - Content personalization: profileComplete must be true
validations:
  - Token signature valid
  - Token not expired
  - User has required permissions
error_handling:
  - Missing token: Return 401 Unauthorized
  - Invalid token: Return 401 Unauthorized
  - Insufficient permissions: Return 403 Forbidden
---

# Access Control Skill

Validates JWT tokens and enforces resource-level authorization based on user claims.

## Purpose
Act as authorization gatekeeper for protected endpoints by validating JWT tokens and enforcing policy-based access control.

## Authorization Policies

### Self Profile Access
- User can access their own profile
- userId in JWT token must match requested resource userId

### Admin Access
- Users with 'admin' role can access any resource
- Role claim must include 'admin'

### Personalized Content Access
- Requires profile completion
- profileComplete claim must be true in JWT

### Authenticated Access
- Valid, non-expired JWT token present
- Signature verification passes

## Workflow
1. Extract JWT from Authorization header
2. Validate token signature
3. Check token expiration
4. Decode user claims
5. Apply authorization policy for requested resource
6. Check required permissions
7. Return authorization decision and user context
8. Log authorization attempt

## Integration Points
- JWT service for token validation
- Policy engine for authorization rules
- Neon DB for audit logging (authz_logs table) and revoked token checks (revoked_tokens table)
- Redis for blacklist checks (blacklist:{tokenId})
- Rate limiting service
