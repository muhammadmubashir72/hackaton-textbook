# Authentication API Contracts

## Overview

This document defines the API contracts for the authentication system that supports the frontend authentication flow enhancement. These contracts ensure consistent communication between frontend and backend services.

## Authentication Endpoints

### 1. User Login
```
POST /api/auth/login
```

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securePassword123"
}
```

**Response (Success)**:
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "user_12345",
      "email": "user@example.com",
      "name": "John Doe",
      "profileImage": "https://example.com/profile.jpg",
      "authMethod": "email"
    },
    "tokens": {
      "accessToken": "eyJhbGciOiJIUzI1NiIs...",
      "refreshToken": "dGhpcyBpcyBhIHJlZnJlc2g...",
      "expiresAt": "2025-12-27T15:30:00Z"
    }
  }
}
```

**Response (Error)**:
```json
{
  "success": false,
  "error": {
    "code": "INVALID_CREDENTIALS",
    "message": "Invalid email or password"
  }
}
```

### 2. User Signup
```
POST /api/auth/signup
```

**Request**:
```json
{
  "email": "newuser@example.com",
  "password": "securePassword123",
  "name": "Jane Smith",
  "softwareBackground": "intermediate",
  "hardwareBackground": "beginner"
}
```

**Response (Success)**:
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "user_67890",
      "email": "newuser@example.com",
      "name": "Jane Smith",
      "profileImage": null,
      "authMethod": "email"
    },
    "tokens": {
      "accessToken": "eyJhbGciOiJIUzI1NiIs...",
      "refreshToken": "dGhpcyBpcyBhIHJlZnJlc2g...",
      "expiresAt": "2025-12-27T15:30:00Z"
    }
  }
}
```

### 3. Token Refresh
```
POST /api/auth/refresh
```

**Request**:
```json
{
  "refreshToken": "dGhpcyBpcyBhIHJlZnJlc2g..."
}
```

**Response (Success)**:
```json
{
  "success": true,
  "data": {
    "tokens": {
      "accessToken": "eyJhbGciOiJIUzI1NiIs...",
      "expiresAt": "2025-12-27T16:30:00Z"
    }
  }
}
```

### 4. User Profile
```
GET /api/auth/profile
```

**Headers**:
```
Authorization: Bearer {accessToken}
```

**Response (Success)**:
```json
{
  "success": true,
  "data": {
    "id": "user_12345",
    "email": "user@example.com",
    "name": "John Doe",
    "profileImage": "https://example.com/profile.jpg",
    "authMethod": "google",
    "createdAt": "2025-12-01T10:00:00Z",
    "profile": {
      "softwareBackground": "advanced",
      "hardwareBackground": "intermediate",
      "completionPercentage": 45
    }
  }
}
```

### 5. User Logout
```
POST /api/auth/logout
```

**Headers**:
```
Authorization: Bearer {accessToken}
```

**Request**:
```json
{
  "refreshToken": "dGhpcyBpcyBhIHJlZnJlc2g..."
}
```

**Response (Success)**:
```json
{
  "success": true,
  "message": "Successfully logged out"
}
```

## Frontend Service Contracts

### 1. Token Service Interface

```javascript
interface TokenService {
  storeTokens(accessToken: string, refreshToken: string): void;
  getTokens(): { accessToken: string, refreshToken: string } | null;
  clearTokens(): void;
  isTokenValid(token: string): boolean;
  setTokenUpdateListener(callback: Function): void;
}
```

### 2. Auth Service Interface

```javascript
interface AuthService {
  login(credentials: LoginCredentials): Promise<AuthResponse>;
  signup(userData: SignupData): Promise<AuthResponse>;
  logout(refreshToken: string): Promise<LogoutResponse>;
  refreshSession(refreshToken: string): Promise<RefreshResponse>;
  validateSession(accessToken: string): Promise<UserProfile>;
}
```

## Error Handling

### Standard Error Format
```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {} // Optional additional details
  }
}
```

### Common Error Codes
- `INVALID_CREDENTIALS`: Email/password combination is incorrect
- `USER_NOT_FOUND`: User account does not exist
- `TOKEN_EXPIRED`: Authentication token has expired
- `TOKEN_INVALID`: Authentication token is malformed or invalid
- `RATE_LIMIT_EXCEEDED`: Too many login attempts
- `ACCOUNT_LOCKED`: Account temporarily locked due to failed attempts
- `VALIDATION_ERROR`: Request data failed validation

## Security Requirements

1. **Token Expiration**: Access tokens must expire within 1 hour
2. **Refresh Token Storage**: Refresh tokens must be stored securely and rotated periodically
3. **Rate Limiting**: Login attempts must be limited to prevent brute force attacks
4. **CORS Policy**: API endpoints must implement appropriate CORS policies
5. **HTTPS Enforcement**: All authentication endpoints must require HTTPS

## Validation Rules

1. **Email Format**: Must be a valid email address
2. **Password Strength**: Minimum 8 characters with mixed case, numbers, and special characters
3. **Rate Limiting**: Maximum 5 login attempts per minute per IP
4. **Token Format**: Must be valid JWT with proper signature and claims
5. **Session Validation**: All protected endpoints must validate access tokens