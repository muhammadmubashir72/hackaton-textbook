---
name: authorization-guard
purpose: Enforce access control and authorization for protected resources
responsibilities:
  - Extract and validate JWT from request headers
  - Verify token signatures and expiration
  - Enforce resource-level permissions
  - Check profile completion requirements
  - Log authorization attempts
dependencies:
  - JWT service agent
  - Authorization policy engine
  - Audit logging service
authorization_policies:
  - self_profile_access: userId in token == requested userId
  - admin_access: 'admin' in token.roles
  - personalized_content_access: token.profileComplete == true
  - authenticated_access: valid token present
interfaces:
  - validateRequest(request, requiredPolicy)
  - extractUserContext(token)
  - checkPermission(userId, resource, action)
middleware_integration:
  - Express/Fastify middleware for route protection
  - Decorator/guard for framework-specific patterns
---

# Authorization Guard Agent

Acts as authorization gatekeeper for all protected endpoints and enforces access control policies.

## Purpose
Middleware-based authorization service that validates JWT tokens and enforces fine-grained access control policies for protected resources.

## Responsibilities

### Request Validation
1. Extract JWT from Authorization header
2. Validate Bearer scheme format
3. Verify token via JWT service
4. Decode user claims
5. Attach user context to request
6. Proceed to policy enforcement

### Policy Enforcement
1. Identify requested resource and action
2. Retrieve applicable authorization policies
3. Evaluate policies against user claims
4. Check required permissions
5. Return authorization decision
6. Log authorization attempt
7. Return 403 if denied, allow if granted

### Token Extraction
1. Check Authorization header present
2. Verify Bearer scheme
3. Extract token string
4. Return token or throw 401

### User Context Building
1. Decode JWT claims
2. Extract userId, email, roles
3. Extract profileComplete status
4. Extract background data
5. Build context object
6. Return user context

## Authorization Policies

### Authenticated Access
- **Rule**: Valid, non-expired JWT present
- **Check**: Token signature valid AND not expired AND not blacklisted
- **Response**: Allow if pass, 401 if fail

### Self Profile Access
- **Rule**: User accessing their own profile
- **Check**: userId in JWT == requested userId in URL/body
- **Response**: Allow if match, 403 if mismatch

### Admin Access
- **Rule**: User has admin role
- **Check**: 'admin' in token.roles array
- **Response**: Allow if admin, 403 if not

### Personalized Content Access
- **Rule**: Profile must be complete
- **Check**: token.profileComplete == true
- **Response**: Allow if complete, 403 with message to complete profile

### Resource-Based Access
- **Rule**: User has specific permission for resource
- **Check**: Check permission matrix for userId + resource + action
- **Response**: Allow if authorized, 403 if not

## Middleware Implementation

### Express Middleware
```javascript
function authGuard(requiredPolicy) {
  return async (req, res, next) => {
    try {
      // Extract token
      const token = extractToken(req);

      // Validate token
      const { isValid, claims } = await jwtService.validateToken(token);
      if (!isValid) {
        return res.status(401).json({ error: 'Invalid token' });
      }

      // Attach user context
      req.user = claims;

      // Enforce policy
      const authorized = await enforcePolicy(claims, req, requiredPolicy);
      if (!authorized) {
        return res.status(403).json({ error: 'Forbidden' });
      }

      // Log authorization
      await logAuthAttempt(claims.sub, req.path, true);

      next();
    } catch (error) {
      await logAuthAttempt(null, req.path, false, error);
      return res.status(401).json({ error: 'Unauthorized' });
    }
  };
}
```

### Usage Examples
```javascript
// Authenticated access only
app.get('/api/content', authGuard('authenticated'), getContent);

// Self profile access
app.get('/api/users/:userId', authGuard('self_profile'), getUser);

// Admin only
app.delete('/api/users/:userId', authGuard('admin'), deleteUser);

// Personalized content (requires complete profile)
app.get('/api/recommendations', authGuard('personalized_content'), getRecommendations);
```

## Policy Evaluation Logic

### Policy Evaluation Flow
```
1. Extract policy name from middleware parameter
2. Load policy rules from policy engine
3. Evaluate each rule against user claims and request context
4. Combine results (AND/OR logic based on policy)
5. Return boolean authorization decision
```

### Policy Rules Structure
```javascript
{
  authenticated: {
    rules: [
      { check: 'token.valid', operator: '==', value: true },
      { check: 'token.expired', operator: '==', value: false }
    ],
    combinator: 'AND'
  },
  self_profile: {
    rules: [
      { check: 'token.sub', operator: '==', value: 'request.params.userId' },
    ],
    combinator: 'AND',
    extends: 'authenticated'
  },
  admin: {
    rules: [
      { check: 'token.roles', operator: 'includes', value: 'admin' }
    ],
    combinator: 'AND',
    extends: 'authenticated'
  },
  personalized_content: {
    rules: [
      { check: 'token.profileComplete', operator: '==', value: true }
    ],
    combinator: 'AND',
    extends: 'authenticated'
  }
}
```

## Error Responses

### 401 Unauthorized
```json
{
  "error": "Unauthorized",
  "message": "Invalid or missing authentication token",
  "code": "AUTH_TOKEN_INVALID"
}
```

### 403 Forbidden
```json
{
  "error": "Forbidden",
  "message": "You do not have permission to access this resource",
  "code": "AUTH_INSUFFICIENT_PERMISSIONS"
}
```

### 403 Profile Incomplete
```json
{
  "error": "Forbidden",
  "message": "Please complete your profile to access personalized content",
  "code": "AUTH_PROFILE_INCOMPLETE",
  "redirectTo": "/profile/complete"
}
```

## Audit Logging
Log every authorization attempt with:
- Timestamp
- userId (if available)
- Requested resource and action
- Authorization result (granted/denied)
- Policy evaluated
- IP address
- User agent

## Integration Points
- JWT service for token validation
- Policy engine for rule evaluation
- Neon DB for audit logging (authz_logs table) and revoked token checks (revoked_tokens table)
- Redis for blacklist caching (blacklist:{tokenId})
- Cache service for policy caching
- Rate limiting service for abuse prevention
