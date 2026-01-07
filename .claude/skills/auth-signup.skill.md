---
name: auth-signup
description: Handle user registration using Better Auth with JWT token issuance
trigger: User initiates signup flow
inputs:
  - email: string (required)
  - password: string (required)
  - confirmPassword: string (required)
  - name: string (optional)
outputs:
  - userId: string
  - jwtToken: string
  - refreshToken: string
  - profileComplete: boolean
validations:
  - Email format validation
  - Password strength requirements (min 8 chars, uppercase, lowercase, number)
  - Password confirmation match
  - Email uniqueness check
error_handling:
  - Duplicate email: Return 409 Conflict
  - Invalid input: Return 400 Bad Request
  - Server error: Return 500 Internal Server Error
---

# Auth Signup Skill

Orchestrates user signup by coordinating with Better Auth, JWT generation, and user profile initialization.

## Purpose
Handle complete user registration flow including credential validation, Better Auth integration, and JWT token issuance.

## Workflow
1. Validate input data (email format, password strength, confirmation match)
2. Check email uniqueness in database
3. Create user via Better Auth
4. Generate initial JWT with profileComplete: false
5. Initialize empty user profile
6. Return authentication tokens

## Integration Points
- Better Auth SDK for user creation
- JWT service for token generation
- Profile manager for profile initialization
- Neon DB for persistent storage (users, profiles, software_backgrounds, hardware_backgrounds, audit_logs, token_sessions tables)
- Redis for refresh token caching
