# Implementation Plan: Better Auth with Neon DB Integration

**Feature**: Better Auth Authentication System with Neon DB Storage
**Branch**: `003-better-auth-neon-db`
**Created**: 2025-12-24
**Status**: Planning Complete

## Executive Summary

This plan implements a comprehensive authentication system using Better Auth with JWT-based authorization, user background data collection for personalization, and Neon DB as the primary persistent storage. The implementation is organized into 5 sequential phases focusing on infrastructure setup, data layer, authentication core, profile management, and integration/testing.

## Technical Context

### Technology Stack
- **Authentication**: Better Auth SDK with email/password provider
- **Token Management**: JWT (HS256) with 15-minute expiration, refresh tokens with 7-day expiration
- **Primary Database**: Neon DB (PostgreSQL-compatible) via @neondatabase/serverless driver
- **Cache Layer**: Redis for tokens, profiles, and personalization context
- **Validation**: Zod for schema validation
- **Encryption**: Node.js crypto (AES-256) for sensitive data

### Architecture Overview
```
┌─────────────────┐
│   Frontend      │
│  (Docusaurus)   │
└────────┬────────┘
         │
         v
┌─────────────────────────────────────────┐
│        FastAPI Backend                  │
│  ┌──────────────────────────────────┐  │
│  │   Auth Orchestration Agent       │  │
│  │  (Coordinates all auth flows)    │  │
│  └──────────┬────────────┬──────────┘  │
│             │            │              │
│    ┌────────v──────┐  ┌─v──────────┐  │
│    │  JWT Service  │  │  Profile   │  │
│    │    Agent      │  │  Manager   │  │
│    └────────┬──────┘  └─┬──────────┘  │
│             │            │              │
│    ┌────────v────────────v──────────┐  │
│    │   Authorization Guard Agent    │  │
│    └─────────────────────────────────┘  │
└─────────┬───────────────┬───────────────┘
          │               │
     ┌────v────┐     ┌───v──────┐
     │  Redis  │     │ Neon DB  │
     │  Cache  │     │ (Primary)│
     └─────────┘     └──────────┘
```

### Constitution Alignment Check

✅ **VII. Authentication and Personalization**: Implements better-auth with signup questions about software/hardware background
✅ **II. AI-Native Educational Experience**: Provides foundation for personalized learning experiences
✅ **V. Advanced Technology Integration**: Uses specified stack (better-auth + Neon Postgres)
✅ **VI. Accessibility and Inclusion**: Collects user background for content personalization

**Gate Status**: ✅ PASS - Implementation aligns with all relevant constitutional principles

## Phase 0: Research & Technical Validation

### Objectives
- Validate technology choices
- Resolve unknowns
- Document integration patterns

### Tasks

#### 0.1: Neon DB Setup & Configuration
**Research Questions**:
- Neon DB connection pooling best practices for serverless environments
- Schema migration strategy using Neon DB branching
- Connection timeout and retry configuration

**Deliverables**:
- `research/neon-db-setup.md` - Connection pooling configuration guide
- `research/neon-db-migrations.md` - Migration strategy using database branching
- Environment variable documentation (NEON_DATABASE_URL, connection pool settings)

#### 0.2: Better Auth Integration Patterns
**Research Questions**:
- Better Auth email/password provider setup
- Session management with JWT adapter
- Callback hooks for post-signup/signin

**Deliverables**:
- `research/better-auth-integration.md` - Setup guide and configuration
- Better Auth configuration template
- Callback hook patterns for profile initialization

#### 0.3: JWT Security & Token Management
**Research Questions**:
- JWT secret rotation procedures
- Refresh token rotation vs non-rotation trade-offs
- Token blacklist storage patterns (Redis + Neon DB)

**Deliverables**:
- `research/jwt-security.md` - Token management best practices
- Secret rotation procedure documentation
- Blacklist cleanup job specifications

#### 0.4: Redis-Neon DB Fallback Strategy
**Research Questions**:
- Graceful degradation when Redis unavailable
- Dual-write vs cache-aside patterns for tokens
- Performance implications of Neon DB fallback

**Deliverables**:
- `research/redis-neon-fallback.md` - Fallback pattern documentation
- Performance benchmarks and trade-offs
- Circuit breaker implementation strategy

**Output**: `research.md` consolidating all findings

**Dependencies**: None
**Estimated Effort**: 2-3 days

---

## Phase 1: Data Layer & Schema Implementation

### Objectives
- Set up Neon DB database with complete schema
- Implement migrations and seed data
- Configure connection pooling

### Tasks

#### 1.1: Neon DB Project Initialization
**Actions**:
1. Create Neon DB project via dashboard/CLI
2. Configure connection string and API key
3. Set up development and production databases
4. Configure connection pooling (min 2, max 10 connections)

**Files**:
- `.env.example` - Template with NEON_DATABASE_URL, NEON_API_KEY
- `backend/config/database.ts` - Connection pool configuration

**Acceptance Criteria**:
- [ ] Neon DB project created
- [ ] Connection string secured in environment variables
- [ ] Connection pooling configured and tested
- [ ] Can connect from backend successfully

#### 1.2: Database Schema Creation
**Actions**:
1. Create migration script for core tables
2. Define indexes on foreign keys and query columns
3. Set up row-level security (RLS) policies
4. Create database roles for application access

**Schema Tables** (from spec.md):
```sql
-- Core Tables
users (id UUID, email VARCHAR UNIQUE, name VARCHAR, password_hash VARCHAR, created_at, updated_at)
profiles (id UUID, user_id UUID FK, profile_complete BOOLEAN, created_at, updated_at)
software_backgrounds (id UUID, profile_id UUID FK, programming_languages TEXT[], frameworks TEXT[], experience_level VARCHAR, specializations TEXT[], years_of_experience INT)
hardware_backgrounds (id UUID, profile_id UUID FK, familiar_platforms TEXT[], robotics_experience VARCHAR, electronics_knowledge VARCHAR, preferred_tools TEXT[])
user_preferences (id UUID, user_id UUID FK, difficulty_setting VARCHAR, focus_areas JSONB, preferred_platforms JSONB, updated_at)

-- Token Management
token_sessions (id UUID, token VARCHAR INDEXED, user_id UUID FK, created_at, expires_at, last_used_at, is_revoked BOOLEAN)
revoked_tokens (id UUID, token_id VARCHAR UNIQUE INDEXED, user_id UUID FK, revoked_at, expires_at INDEXED, reason VARCHAR)

-- Audit Logs
auth_logs (id UUID, user_id UUID FK NULLABLE, email VARCHAR, ip_address INET, user_agent TEXT, success BOOLEAN, failure_reason VARCHAR NULLABLE, timestamp TIMESTAMP INDEXED)
authz_logs (id UUID, user_id UUID FK, resource VARCHAR, action VARCHAR, policy VARCHAR, authorized BOOLEAN, timestamp TIMESTAMP INDEXED)
audit_logs (id UUID, user_id UUID FK NULLABLE, operation VARCHAR, table_name VARCHAR, record_id UUID, old_values JSONB, new_values JSONB, timestamp TIMESTAMP INDEXED)
```

**Files**:
- `backend/migrations/001_create_core_schema.sql`
- `backend/migrations/002_create_indexes.sql`
- `backend/migrations/003_create_rls_policies.sql`

**Acceptance Criteria**:
- [ ] All 10 tables created successfully
- [ ] Indexes created on foreign keys and timestamp columns
- [ ] RLS policies prevent cross-user data access
- [ ] Migration can be rolled back cleanly

#### 1.3: Neon DB Connection Module
**Actions**:
1. Create database connection wrapper using @neondatabase/serverless
2. Implement connection pooling with retry logic
3. Add query helper functions (SELECT, INSERT, UPDATE, DELETE)
4. Implement transaction management

**Files**:
- `backend/db/connection.ts` - Connection pool manager
- `backend/db/query-builder.ts` - Helper functions with prepared statements
- `backend/db/transactions.ts` - Transaction wrapper

**Key Functions**:
```typescript
// Connection management
export async function getConnection(): Promise<PoolClient>
export async function query(sql: string, params: any[]): Promise<QueryResult>
export async function transaction<T>(callback: (client: PoolClient) => Promise<T>): Promise<T>

// Retry logic
export async function queryWithRetry(sql: string, params: any[], maxRetries = 3): Promise<QueryResult>
```

**Acceptance Criteria**:
- [ ] Connection pool initializes on startup
- [ ] Retry logic handles transient failures (max 3 retries, exponential backoff)
- [ ] Prepared statements prevent SQL injection
- [ ] Transaction rollback works correctly on errors

#### 1.4: Data Access Layer (DAL)
**Actions**:
1. Create repository pattern for each entity
2. Implement CRUD operations with validation
3. Add audit logging for all mutations
4. Implement efficient JOIN queries for profile retrieval

**Files**:
- `backend/dal/user-repository.ts`
- `backend/dal/profile-repository.ts`
- `backend/dal/token-repository.ts`
- `backend/dal/audit-repository.ts`

**Key Methods**:
```typescript
// User Repository
createUser(email: string, passwordHash: string, name?: string): Promise<User>
findUserByEmail(email: string): Promise<User | null>
findUserById(id: string): Promise<User | null>

// Profile Repository
createProfile(userId: string): Promise<Profile>
updateProfile(userId: string, data: ProfileData): Promise<Profile>
getCompleteProfile(userId: string): Promise<CompleteProfile> // JOINs all tables
markProfileComplete(userId: string): Promise<void>

// Token Repository
storeRefreshToken(token: string, userId: string, expiresAt: Date): Promise<void>
validateRefreshToken(token: string): Promise<{ valid: boolean; userId?: string }>
revokeRefreshToken(token: string): Promise<void>
addToBlacklist(tokenId: string, userId: string, expiresAt: Date, reason: string): Promise<void>
isTokenBlacklisted(tokenId: string): Promise<boolean>

// Audit Repository
logAuthAttempt(email: string, success: boolean, ipAddress: string, userAgent: string, failureReason?: string): Promise<void>
logAuthzAttempt(userId: string, resource: string, policy: string, authorized: boolean): Promise<void>
logDatabaseMutation(userId: string, operation: string, tableName: string, recordId: string, oldValues: any, newValues: any): Promise<void>
```

**Acceptance Criteria**:
- [ ] All CRUD operations tested
- [ ] Audit logs written for all mutations
- [ ] Profile JOIN query retrieves all data in single query
- [ ] Repository methods use prepared statements

**Dependencies**: Phase 0 complete
**Estimated Effort**: 4-5 days

---

## Phase 2: Authentication Core (Better Auth + JWT)

### Objectives
- Integrate Better Auth for credential management
- Implement JWT generation and validation
- Set up Redis caching layer

### Tasks

#### 2.1: Better Auth Configuration
**Actions**:
1. Install and configure Better Auth SDK
2. Set up email/password provider
3. Configure session settings
4. Implement user creation callback

**Files**:
- `backend/auth/better-auth-config.ts` - Better Auth initialization
- `backend/auth/providers/email-password.ts` - Email/password provider setup

**Configuration**:
```typescript
import { betterAuth } from "better-auth"

export const auth = betterAuth({
  database: {
    provider: "neon", // Use Neon DB for Better Auth's internal tables
    url: process.env.NEON_DATABASE_URL
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireUppercase: true,
    requireLowercase: true,
    requireNumber: true
  },
  session: {
    expiresIn: 60 * 15, // 15 minutes (JWT will be primary)
    updateAge: 60 * 5 // Update session every 5 minutes
  },
  callbacks: {
    async onSignUp(user) {
      // Initialize empty profile after signup
      await profileRepository.createProfile(user.id)
    }
  }
})
```

**Acceptance Criteria**:
- [ ] Better Auth initializes successfully
- [ ] Email/password validation enforces requirements
- [ ] User creation triggers profile initialization
- [ ] Better Auth stores data in Neon DB

#### 2.2: JWT Service Implementation
**Actions**:
1. Create JWT generation with custom claims
2. Implement JWT validation middleware
3. Add refresh token generation
4. Implement token revocation

**Files**:
- `backend/services/jwt-service.ts` - JWT operations
- `backend/middleware/jwt-validation.ts` - Express/Fastify middleware

**Key Functions**:
```typescript
// JWT Generation
export function generateAccessToken(user: User, profile: Profile): string {
  const claims = {
    sub: user.id,
    email: user.email,
    name: user.name,
    profileComplete: profile.profileComplete,
    softwareBackground: profile.softwareBackground.programmingLanguages.slice(0, 5),
    hardwareBackground: profile.hardwareBackground.familiarPlatforms.slice(0, 5),
    roles: ['user'],
    iat: Math.floor(Date.now() / 1000),
    exp: Math.floor(Date.now() / 1000) + (15 * 60), // 15 minutes
    iss: 'physical-ai-textbook'
  }
  return jwt.sign(claims, process.env.JWT_SECRET, { algorithm: 'HS256' })
}

export function generateRefreshToken(): string {
  return crypto.randomBytes(32).toString('hex')
}

// JWT Validation
export async function validateAccessToken(token: string): Promise<JWTPayload | null> {
  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET, { algorithms: ['HS256'] })

    // Check blacklist
    const isBlacklisted = await isTokenBlacklisted(decoded.jti || decoded.sub)
    if (isBlacklisted) return null

    return decoded as JWTPayload
  } catch (error) {
    return null
  }
}

// Refresh Token Operations
export async function validateRefreshToken(token: string): Promise<{ valid: boolean; userId?: string }> {
  // Check Redis first
  const userId = await redis.get(`refresh_token:${token}`)
  if (userId) {
    return { valid: true, userId }
  }

  // Fallback to Neon DB
  return await tokenRepository.validateRefreshToken(token)
}

// Token Revocation
export async function revokeTokens(accessToken: string, refreshToken: string): Promise<void> {
  const decoded = jwt.decode(accessToken) as JWTPayload

  // Add JWT to blacklist (Redis + Neon DB)
  await Promise.all([
    redis.setex(`blacklist:${decoded.jti}`, decoded.exp - Math.floor(Date.now() / 1000), '1'),
    tokenRepository.addToBlacklist(decoded.jti, decoded.sub, new Date(decoded.exp * 1000), 'signout')
  ])

  // Revoke refresh token (Redis + Neon DB)
  await Promise.all([
    redis.del(`refresh_token:${refreshToken}`),
    tokenRepository.revokeRefreshToken(refreshToken)
  ])
}
```

**Acceptance Criteria**:
- [ ] JWT contains all required claims from specification
- [ ] JWT expiration set to 15 minutes
- [ ] Refresh tokens stored in both Redis and Neon DB
- [ ] Revoked tokens rejected immediately
- [ ] Middleware validates tokens on protected routes

#### 2.3: Redis Cache Layer
**Actions**:
1. Configure Redis connection with retry logic
2. Implement cache helpers for tokens and profiles
3. Add fallback to Neon DB when Redis unavailable
4. Implement cache invalidation patterns

**Files**:
- `backend/cache/redis-client.ts` - Redis connection and helpers
- `backend/cache/cache-manager.ts` - Cache-aside pattern implementation

**Key Functions**:
```typescript
// Cache Operations
export async function cacheRefreshToken(token: string, userId: string, ttl: number): Promise<void> {
  try {
    await redis.setex(`refresh_token:${token}`, ttl, userId)
    // Fallback to Neon DB for persistence
    await tokenRepository.storeRefreshToken(token, userId, new Date(Date.now() + ttl * 1000))
  } catch (error) {
    logger.warn('Redis unavailable, using Neon DB only', error)
    await tokenRepository.storeRefreshToken(token, userId, new Date(Date.now() + ttl * 1000))
  }
}

export async function cacheProfile(userId: string, profile: CompleteProfile): Promise<void> {
  try {
    await redis.setex(`profile:${userId}`, 3600, JSON.stringify(profile))
  } catch (error) {
    logger.warn('Profile caching failed, will fetch from DB on next request', error)
  }
}

export async function getCachedProfile(userId: string): Promise<CompleteProfile | null> {
  try {
    const cached = await redis.get(`profile:${userId}`)
    return cached ? JSON.parse(cached) : null
  } catch (error) {
    logger.warn('Redis unavailable for profile cache', error)
    return null
  }
}

export async function invalidateProfileCache(userId: string): Promise<void> {
  try {
    await Promise.all([
      redis.del(`profile:${userId}`),
      redis.del(`personalization_context:${userId}`)
    ])
  } catch (error) {
    logger.warn('Cache invalidation failed', error)
  }
}
```

**Acceptance Criteria**:
- [ ] Redis connection handles disconnects gracefully
- [ ] Tokens stored in both Redis and Neon DB
- [ ] Profile cache hit rate > 80% in testing
- [ ] System functions when Redis is unavailable (degraded performance)

#### 2.4: Auth Orchestration Agent Implementation
**Actions**:
1. Implement signup flow coordination
2. Implement signin flow coordination
3. Implement token refresh flow
4. Implement signout flow

**Files**:
- `backend/agents/auth-orchestration-agent.ts` - Main orchestration logic
- `backend/routes/auth.ts` - API endpoints

**API Endpoints**:
```typescript
POST /api/auth/signup
Request: { email: string, password: string, confirmPassword: string, name?: string }
Response: { userId: string, accessToken: string, refreshToken: string, profileComplete: boolean }

POST /api/auth/signin
Request: { email: string, password: string }
Response: { userId: string, accessToken: string, refreshToken: string, profile: CompleteProfile }

POST /api/auth/refresh
Request: { refreshToken: string }
Response: { accessToken: string, refreshToken: string }

POST /api/auth/signout
Request: { refreshToken: string }
Headers: Authorization: Bearer <accessToken>
Response: { success: boolean }
```

**Signup Flow**:
```typescript
export async function handleSignup(email: string, password: string, name?: string): Promise<SignupResponse> {
  // 1. Validate input
  const validation = validateSignupInput(email, password)
  if (!validation.valid) throw new ValidationError(validation.errors)

  // 2. Check email uniqueness
  const existing = await userRepository.findUserByEmail(email)
  if (existing) throw new ConflictError('Email already registered')

  // 3. Create user via Better Auth
  const user = await auth.emailPassword.signUp({ email, password, name })

  // 4. Initialize profile (callback triggered)
  // Profile is created automatically via Better Auth callback

  // 5. Generate tokens
  const profile = await profileRepository.getCompleteProfile(user.id)
  const accessToken = generateAccessToken(user, profile)
  const refreshToken = generateRefreshToken()

  // 6. Store refresh token
  await cacheRefreshToken(refreshToken, user.id, 7 * 24 * 60 * 60)

  // 7. Log authentication event
  await auditRepository.logAuthAttempt(email, true, req.ip, req.headers['user-agent'])

  return {
    userId: user.id,
    accessToken,
    refreshToken,
    profileComplete: profile.profileComplete
  }
}
```

**Signin Flow**:
```typescript
export async function handleSignin(email: string, password: string): Promise<SigninResponse> {
  // 1. Validate credentials via Better Auth
  const user = await auth.emailPassword.signIn({ email, password })
  if (!user) {
    await auditRepository.logAuthAttempt(email, false, req.ip, req.headers['user-agent'], 'Invalid credentials')
    throw new UnauthorizedError('Invalid email or password')
  }

  // 2. Retrieve complete profile (check cache first)
  let profile = await getCachedProfile(user.id)
  if (!profile) {
    profile = await profileRepository.getCompleteProfile(user.id)
    await cacheProfile(user.id, profile)
  }

  // 3. Generate tokens with profile claims
  const accessToken = generateAccessToken(user, profile)
  const refreshToken = generateRefreshToken()

  // 4. Store refresh token
  await cacheRefreshToken(refreshToken, user.id, 7 * 24 * 60 * 60)

  // 5. Log successful authentication
  await auditRepository.logAuthAttempt(email, true, req.ip, req.headers['user-agent'])

  return {
    userId: user.id,
    accessToken,
    refreshToken,
    profile
  }
}
```

**Acceptance Criteria**:
- [ ] Signup creates user and initializes profile
- [ ] Signin retrieves profile and generates tokens
- [ ] Token refresh validates and rotates tokens
- [ ] Signout revokes all tokens
- [ ] All authentication events logged to auth_logs table

**Dependencies**: Phase 1 complete
**Estimated Effort**: 5-6 days

---

## Phase 3: Profile Management & Background Collection

### Objectives
- Implement profile data collection
- Add validation for software/hardware backgrounds
- Enable profile updates with audit logging

### Tasks

#### 3.1: Profile Collection Skill Implementation
**Actions**:
1. Create validation schemas for software/hardware backgrounds
2. Implement profile update endpoint
3. Add profile completion check logic
4. Trigger JWT refresh on profile completion

**Files**:
- `backend/schemas/profile-schemas.ts` - Zod validation schemas
- `backend/agents/profile-manager-agent.ts` - Profile management logic
- `backend/routes/profile.ts` - Profile API endpoints

**Validation Schemas**:
```typescript
import { z } from 'zod'

export const SoftwareBackgroundSchema = z.object({
  programmingLanguages: z.array(z.string()).min(1).max(20),
  frameworks: z.array(z.string()).max(20),
  experienceLevel: z.enum(['beginner', 'intermediate', 'advanced', 'expert']),
  specializations: z.array(z.string()).max(10),
  yearsOfExperience: z.number().min(0).max(50).optional()
})

export const HardwareBackgroundSchema = z.object({
  familiarPlatforms: z.array(z.string()).min(1).max(10),
  roboticsExperience: z.enum(['none', 'hobbyist', 'professional']),
  electronicsKnowledge: z.enum(['none', 'basic', 'intermediate', 'advanced']),
  preferredTools: z.array(z.string()).max(10)
})

export const ProfileUpdateSchema = z.object({
  softwareBackground: SoftwareBackgroundSchema,
  hardwareBackground: HardwareBackgroundSchema
})
```

**API Endpoints**:
```typescript
GET /api/profile/:userId
Headers: Authorization: Bearer <accessToken>
Response: CompleteProfile

PUT /api/profile/:userId
Headers: Authorization: Bearer <accessToken>
Request: { softwareBackground: SoftwareBackgroundData, hardwareBackground: HardwareBackgroundData }
Response: { profile: CompleteProfile, accessToken: string }
```

**Profile Update Flow**:
```typescript
export async function updateProfile(userId: string, data: ProfileUpdateData): Promise<ProfileUpdateResponse> {
  // 1. Validate authorization (self-profile only)
  if (req.user.sub !== userId) {
    throw new ForbiddenError('Cannot update other user profiles')
  }

  // 2. Validate input data
  const validation = ProfileUpdateSchema.safeParse(data)
  if (!validation.success) {
    throw new ValidationError(validation.error.errors)
  }

  // 3. Get current profile for audit log
  const oldProfile = await profileRepository.getCompleteProfile(userId)

  // 4. Update in transaction
  const newProfile = await transaction(async (client) => {
    // Update software background
    await client.query(
      'UPDATE software_backgrounds SET programming_languages = $1, frameworks = $2, experience_level = $3, specializations = $4, years_of_experience = $5, updated_at = NOW() WHERE profile_id = $6',
      [data.softwareBackground.programmingLanguages, data.softwareBackground.frameworks, data.softwareBackground.experienceLevel, data.softwareBackground.specializations, data.softwareBackground.yearsOfExperience, oldProfile.id]
    )

    // Update hardware background
    await client.query(
      'UPDATE hardware_backgrounds SET familiar_platforms = $1, robotics_experience = $2, electronics_knowledge = $3, preferred_tools = $4, updated_at = NOW() WHERE profile_id = $5',
      [data.hardwareBackground.familiarPlatforms, data.hardwareBackground.roboticsExperience, data.hardwareBackground.electronicsKnowledge, data.hardwareBackground.preferredTools, oldProfile.id]
    )

    // Check profile completion
    const isComplete = checkProfileCompletion(data.softwareBackground, data.hardwareBackground)
    if (isComplete && !oldProfile.profileComplete) {
      await client.query('UPDATE profiles SET profile_complete = true, updated_at = NOW() WHERE user_id = $1', [userId])
    }

    // Write audit log
    await auditRepository.logDatabaseMutation(
      userId,
      'UPDATE',
      'profiles',
      oldProfile.id,
      { softwareBackground: oldProfile.softwareBackground, hardwareBackground: oldProfile.hardwareBackground },
      data
    )

    return await profileRepository.getCompleteProfile(userId)
  })

  // 5. Invalidate caches
  await invalidateProfileCache(userId)

  // 6. Generate new JWT with updated claims
  const user = await userRepository.findUserById(userId)
  const newAccessToken = generateAccessToken(user, newProfile)

  // 7. Emit profile updated event (for personalization context)
  eventBus.emit('profile.updated', { userId, profile: newProfile })

  return {
    profile: newProfile,
    accessToken: newAccessToken
  }
}

function checkProfileCompletion(software: SoftwareBackgroundData, hardware: HardwareBackgroundData): boolean {
  return (
    software.programmingLanguages.length >= 1 &&
    software.experienceLevel !== undefined &&
    hardware.familiarPlatforms.length >= 1 &&
    hardware.roboticsExperience !== undefined &&
    hardware.electronicsKnowledge !== undefined
  )
}
```

**Acceptance Criteria**:
- [ ] Profile validation rejects invalid data
- [ ] Profile updates are atomic (transaction)
- [ ] Profile completion marked when criteria met
- [ ] New JWT issued with updated claims
- [ ] Audit log records old and new values
- [ ] Cache invalidated after update

#### 3.2: Authorization Guard Implementation
**Actions**:
1. Create middleware for JWT validation
2. Implement policy-based authorization
3. Add authz logging
4. Implement self-profile access checks

**Files**:
- `backend/agents/authorization-guard-agent.ts` - Authorization logic
- `backend/middleware/auth-guard.ts` - Express/Fastify middleware
- `backend/policies/authorization-policies.ts` - Policy definitions

**Authorization Policies**:
```typescript
export const AuthorizationPolicies = {
  authenticated: {
    check: async (req: Request): Promise<boolean> => {
      const token = extractBearerToken(req)
      if (!token) return false

      const payload = await validateAccessToken(token)
      if (!payload) return false

      req.user = payload
      return true
    }
  },

  self_profile: {
    check: async (req: Request): Promise<boolean> => {
      if (!req.user) return false
      const requestedUserId = req.params.userId
      return req.user.sub === requestedUserId
    },
    extends: 'authenticated'
  },

  personalized_content: {
    check: async (req: Request): Promise<boolean> => {
      if (!req.user) return false
      return req.user.profileComplete === true
    },
    extends: 'authenticated'
  },

  admin: {
    check: async (req: Request): Promise<boolean> => {
      if (!req.user) return false
      return req.user.roles.includes('admin')
    },
    extends: 'authenticated'
  }
}
```

**Middleware Implementation**:
```typescript
export function requireAuth(policyName: string) {
  return async (req: Request, res: Response, next: NextFunction) => {
    try {
      const policy = AuthorizationPolicies[policyName]
      if (!policy) {
        throw new Error(`Unknown policy: ${policyName}`)
      }

      // Check base policy
      const baseAuthorized = await policy.check(req)
      if (!baseAuthorized) {
        await auditRepository.logAuthzAttempt(
          req.user?.sub || null,
          req.path,
          policyName,
          false
        )

        return res.status(401).json({
          error: 'Unauthorized',
          message: 'Invalid or missing authentication token',
          code: 'AUTH_TOKEN_INVALID'
        })
      }

      // Check extended policy if applicable
      if (policy.extends) {
        const parentPolicy = AuthorizationPolicies[policy.extends]
        const parentAuthorized = await parentPolicy.check(req)
        if (!parentAuthorized) {
          await auditRepository.logAuthzAttempt(
            req.user?.sub || null,
            req.path,
            policyName,
            false
          )

          return res.status(403).json({
            error: 'Forbidden',
            message: 'You do not have permission to access this resource',
            code: 'AUTH_INSUFFICIENT_PERMISSIONS'
          })
        }
      }

      // Log successful authorization
      await auditRepository.logAuthzAttempt(
        req.user.sub,
        req.path,
        policyName,
        true
      )

      next()
    } catch (error) {
      logger.error('Authorization error', error)
      return res.status(500).json({
        error: 'Internal Server Error',
        message: 'Authorization check failed',
        code: 'AUTH_ERROR'
      })
    }
  }
}
```

**Usage Examples**:
```typescript
// Authenticated access only
router.get('/api/content', requireAuth('authenticated'), getContentHandler)

// Self profile access
router.get('/api/profile/:userId', requireAuth('self_profile'), getProfileHandler)

// Personalized content (requires complete profile)
router.get('/api/recommendations', requireAuth('personalized_content'), getRecommendationsHandler)

// Admin only
router.delete('/api/users/:userId', requireAuth('admin'), deleteUserHandler)
```

**Acceptance Criteria**:
- [ ] Middleware validates JWT on every request
- [ ] Policies enforced correctly (authenticated, self_profile, admin, personalized_content)
- [ ] Authorization decisions logged to authz_logs table
- [ ] Appropriate error responses (401 for invalid token, 403 for insufficient permissions)

**Dependencies**: Phase 2 complete
**Estimated Effort**: 3-4 days

---

## Phase 4: Personalization Context & Integration

### Objectives
- Build personalization context from profile data
- Integrate with content service
- Implement cache warming and invalidation

### Tasks

#### 4.1: Personalization Context Agent
**Actions**:
1. Create context building algorithm
2. Implement user_preferences persistence
3. Add Redis caching for context
4. Subscribe to profile update events

**Files**:
- `backend/agents/personalization-context-agent.ts` - Context building logic
- `backend/services/personalization-service.ts` - Service layer

**Context Structure** (from spec.md):
```typescript
interface PersonalizationContext {
  userId: string
  profileComplete: boolean
  expertise: {
    software: {
      languages: Record<string, 'beginner' | 'intermediate' | 'advanced' | 'expert'>
      frameworks: Record<string, 'beginner' | 'intermediate' | 'advanced'>
      overallLevel: 'beginner' | 'intermediate' | 'advanced' | 'expert'
      specializations: string[]
    }
    hardware: {
      platforms: Record<string, 'learning' | 'proficient' | 'expert'>
      roboticsExperience: 'none' | 'hobbyist' | 'professional'
      electronicsKnowledge: 'none' | 'basic' | 'intermediate' | 'advanced'
      preferredTools: string[]
    }
  }
  learningPath: {
    currentLevel: string
    suggestedNextTopics: string[]
    skillGaps: string[]
  }
  contentPreferences: {
    difficultySetting: string
    focusAreas: string[]
    preferredPlatforms: string[]
    showAdvancedTopics: boolean
    showBeginnerTopics: boolean
  }
  recommendations: {
    chapters: Array<{ id: string, title: string, reason: string }>
    projects: Array<{ id: string, title: string, reason: string }>
  }
  uiAdaptations: {
    showCodeExamples: boolean
    codeLanguage: string
    showHardwareSetup: boolean
    preferredPlatform: string
    hideBeginnerTips: boolean
    showAdvancedNotes: boolean
  }
  metadata: {
    createdAt: string
    lastUpdated: string
    cacheExpiry: string
  }
}
```

**Context Building**:
```typescript
export async function buildPersonalizationContext(userId: string): Promise<PersonalizationContext> {
  // 1. Retrieve profile
  const profile = await profileRepository.getCompleteProfile(userId)

  if (!profile.profileComplete) {
    throw new Error('Profile not complete')
  }

  // 2. Map software expertise
  const softwareExpertise = {
    languages: profile.softwareBackground.programmingLanguages.reduce((acc, lang) => {
      acc[lang] = inferLanguageProficiency(profile.softwareBackground.experienceLevel, profile.softwareBackground.specializations)
      return acc
    }, {}),
    frameworks: profile.softwareBackground.frameworks.reduce((acc, fw) => {
      acc[fw] = inferFrameworkProficiency(profile.softwareBackground.experienceLevel)
      return acc
    }, {}),
    overallLevel: profile.softwareBackground.experienceLevel,
    specializations: profile.softwareBackground.specializations
  }

  // 3. Map hardware expertise
  const hardwareExpertise = {
    platforms: profile.hardwareBackground.familiarPlatforms.reduce((acc, platform) => {
      acc[platform] = inferPlatformProficiency(profile.hardwareBackground.roboticsExperience, profile.hardwareBackground.electronicsKnowledge)
      return acc
    }, {}),
    roboticsExperience: profile.hardwareBackground.roboticsExperience,
    electronicsKnowledge: profile.hardwareBackground.electronicsKnowledge,
    preferredTools: profile.hardwareBackground.preferredTools
  }

  // 4. Generate learning path
  const learningPath = generateLearningPath(softwareExpertise, hardwareExpertise)

  // 5. Build content preferences
  const contentPreferences = {
    difficultySetting: softwareExpertise.overallLevel,
    focusAreas: softwareExpertise.specializations,
    preferredPlatforms: Object.keys(hardwareExpertise.platforms),
    showAdvancedTopics: softwareExpertise.overallLevel === 'advanced' || softwareExpertise.overallLevel === 'expert',
    showBeginnerTopics: softwareExpertise.overallLevel === 'beginner'
  }

  // 6. Generate recommendations (integrate with content service)
  const recommendations = await generateRecommendations(contentPreferences, softwareExpertise, hardwareExpertise)

  // 7. Build UI adaptations
  const uiAdaptations = {
    showCodeExamples: softwareExpertise.languages.length > 0,
    codeLanguage: profile.softwareBackground.programmingLanguages[0] || 'Python',
    showHardwareSetup: hardwareExpertise.platforms.length > 0,
    preferredPlatform: profile.hardwareBackground.familiarPlatforms[0] || 'Raspberry Pi',
    hideBeginnerTips: softwareExpertise.overallLevel === 'advanced' || softwareExpertise.overallLevel === 'expert',
    showAdvancedNotes: softwareExpertise.overallLevel === 'advanced' || softwareExpertise.overallLevel === 'expert'
  }

  const context: PersonalizationContext = {
    userId,
    profileComplete: true,
    expertise: {
      software: softwareExpertise,
      hardware: hardwareExpertise
    },
    learningPath,
    contentPreferences,
    recommendations,
    uiAdaptations,
    metadata: {
      createdAt: new Date().toISOString(),
      lastUpdated: new Date().toISOString(),
      cacheExpiry: new Date(Date.now() + 3600000).toISOString() // 1 hour
    }
  }

  // 8. Store preferences in Neon DB
  await storeUserPreferences(userId, contentPreferences)

  // 9. Cache context in Redis
  await cachePersonalizationContext(userId, context)

  return context
}

async function cachePersonalizationContext(userId: string, context: PersonalizationContext): Promise<void> {
  try {
    await redis.setex(`personalization_context:${userId}`, 3600, JSON.stringify(context))
  } catch (error) {
    logger.warn('Failed to cache personalization context', error)
  }
}

async function storeUserPreferences(userId: string, preferences: ContentPreferences): Promise<void> {
  await query(
    `INSERT INTO user_preferences (user_id, difficulty_setting, focus_areas, preferred_platforms, updated_at)
     VALUES ($1, $2, $3, $4, NOW())
     ON CONFLICT (user_id) DO UPDATE SET
       difficulty_setting = EXCLUDED.difficulty_setting,
       focus_areas = EXCLUDED.focus_areas,
       preferred_platforms = EXCLUDED.preferred_platforms,
       updated_at = NOW()`,
    [userId, preferences.difficultySetting, JSON.stringify(preferences.focusAreas), JSON.stringify(preferences.preferredPlatforms)]
  )
}
```

**API Endpoints**:
```typescript
GET /api/personalization/context
Headers: Authorization: Bearer <accessToken>
Response: PersonalizationContext

POST /api/personalization/refresh
Headers: Authorization: Bearer <accessToken>
Response: PersonalizationContext
```

**Event Handlers**:
```typescript
// Listen for profile updates
eventBus.on('profile.updated', async ({ userId, profile }) => {
  // Invalidate cached context
  await redis.del(`personalization_context:${userId}`)

  // Rebuild context if profile is complete
  if (profile.profileComplete) {
    await buildPersonalizationContext(userId)
  }
})

// Listen for profile completion
eventBus.on('profile.completed', async ({ userId }) => {
  // Build initial personalization context
  await buildPersonalizationContext(userId)
})
```

**Acceptance Criteria**:
- [ ] Context built successfully from complete profiles
- [ ] User preferences persisted to Neon DB
- [ ] Context cached in Redis with 1-hour TTL
- [ ] Context invalidated on profile updates
- [ ] Context rebuilt on profile completion event

#### 4.2: Content Service Integration
**Actions**:
1. Define interface for content service
2. Implement recommendation generation
3. Add content filtering based on preferences
4. Integrate with existing RAG system

**Files**:
- `backend/services/content-service.ts` - Content service interface
- `backend/services/recommendation-engine.ts` - Recommendation logic

**Content Service Interface**:
```typescript
export interface ContentService {
  getChapterRecommendations(context: PersonalizationContext): Promise<ChapterRecommendation[]>
  getProjectRecommendations(context: PersonalizationContext): Promise<ProjectRecommendation[]>
  filterContentByDifficulty(content: Content[], difficulty: string): Content[]
  adaptContentForUser(content: Content, context: PersonalizationContext): AdaptedContent
}

// Integration with existing RAG/content system
export async function generateRecommendations(
  contentPreferences: ContentPreferences,
  softwareExpertise: SoftwareExpertise,
  hardwareExpertise: HardwareExpertise
): Promise<Recommendations> {
  // Query existing content database for matching chapters/projects
  const matchingChapters = await findMatchingChapters({
    difficulty: contentPreferences.difficultySetting,
    focusAreas: contentPreferences.focusAreas,
    platforms: contentPreferences.preferredPlatforms
  })

  const matchingProjects = await findMatchingProjects({
    frameworks: Object.keys(softwareExpertise.frameworks),
    platforms: Object.keys(hardwareExpertise.platforms)
  })

  return {
    chapters: matchingChapters.map(ch => ({
      id: ch.id,
      title: ch.title,
      reason: `Matches ${contentPreferences.difficultySetting} level and ${contentPreferences.focusAreas.join(', ')}`
    })),
    projects: matchingProjects.map(p => ({
      id: p.id,
      title: p.title,
      reason: `Uses familiar platforms (${p.platform}) and frameworks (${p.framework})`
    }))
  }
}
```

**Acceptance Criteria**:
- [ ] Recommendations generated based on user expertise
- [ ] Content filtered by difficulty level
- [ ] Integration with existing RAG system functional
- [ ] Recommendations updated when profile changes

**Dependencies**: Phase 3 complete
**Estimated Effort**: 4-5 days

---

## Phase 5: Testing, Documentation & Deployment

### Objectives
- Comprehensive testing of all flows
- API documentation
- Deployment configuration
- Performance optimization

### Tasks

#### 5.1: Unit & Integration Testing
**Actions**:
1. Write tests for all repository methods
2. Test authentication flows end-to-end
3. Test authorization policies
4. Test profile management and personalization

**Test Coverage**:
- [ ] User repository CRUD operations
- [ ] Profile repository with JOINs
- [ ] Token generation and validation
- [ ] Refresh token flow
- [ ] Token revocation
- [ ] Profile update with audit logging
- [ ] Authorization middleware
- [ ] Personalization context building
- [ ] Redis fallback to Neon DB

**Files**:
- `backend/tests/dal/*.test.ts`
- `backend/tests/services/*.test.ts`
- `backend/tests/agents/*.test.ts`
- `backend/tests/integration/*.test.ts`

#### 5.2: API Documentation
**Actions**:
1. Generate OpenAPI specification
2. Document all endpoints
3. Add authentication requirements
4. Provide example requests/responses

**Deliverables**:
- `backend/docs/api-spec.yaml` - OpenAPI 3.0 specification
- `backend/docs/authentication-guide.md` - Authentication flow documentation
- `backend/docs/profile-management.md` - Profile API guide

#### 5.3: Neon DB Optimization
**Actions**:
1. Create indexes on frequently queried columns
2. Optimize JOIN queries for profile retrieval
3. Implement connection pool monitoring
4. Set up query performance logging

**Optimizations**:
```sql
-- Indexes for performance
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_profiles_user_id ON profiles(user_id);
CREATE INDEX idx_token_sessions_token ON token_sessions(token);
CREATE INDEX idx_token_sessions_user_id ON token_sessions(user_id);
CREATE INDEX idx_revoked_tokens_token_id ON revoked_tokens(token_id);
CREATE INDEX idx_revoked_tokens_expires_at ON revoked_tokens(expires_at);
CREATE INDEX idx_auth_logs_timestamp ON auth_logs(timestamp);
CREATE INDEX idx_authz_logs_timestamp ON authz_logs(timestamp);
CREATE INDEX idx_audit_logs_timestamp ON audit_logs(timestamp);

-- Composite indexes for complex queries
CREATE INDEX idx_token_sessions_user_not_revoked ON token_sessions(user_id, is_revoked, expires_at);
CREATE INDEX idx_auth_logs_user_timestamp ON auth_logs(user_id, timestamp);
```

#### 5.4: Deployment Configuration
**Actions**:
1. Configure environment variables for production
2. Set up Neon DB production database
3. Configure Redis for production
4. Set up monitoring and logging

**Environment Variables**:
```bash
# Neon DB
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.region.neon.tech/db?sslmode=require
NEON_API_KEY=neon_api_key_xxx

# Redis
REDIS_URL=redis://user:pass@redis-host:6379

# Better Auth
BETTER_AUTH_SECRET=secret_xxx

# JWT
JWT_SECRET=jwt_secret_xxx
JWT_EXPIRY=900 # 15 minutes
REFRESH_TOKEN_EXPIRY=604800 # 7 days

# Encryption
ENCRYPTION_KEY=aes256_encryption_key_xxx

# Environment
NODE_ENV=production
PORT=8000
```

**Acceptance Criteria**:
- [ ] All tests passing
- [ ] API documentation complete
- [ ] Performance benchmarks meet success criteria
- [ ] Production deployment successful
- [ ] Monitoring and logging configured

**Dependencies**: Phases 1-4 complete
**Estimated Effort**: 4-5 days

---

## Data Model Summary

### Entity Relationships
```
User (1) ──< (1) Profile
Profile (1) ──< (1) SoftwareBackground
Profile (1) ──< (1) HardwareBackground
User (1) ──< (0..1) UserPreferences
User (1) ──< (*) TokenSessions
User (1) ──< (*) RevokedTokens
User (1) ──< (*) AuthLogs
User (1) ──< (*) AuthzLogs
User (1) ──< (*) AuditLogs
```

### Key Data Flows

**Signup → Profile Initialization**:
```
POST /auth/signup
  → Better Auth creates user
  → Callback creates empty profile
  → JWT generated (profileComplete: false)
  → Refresh token stored (Redis + Neon DB)
  → Return tokens
```

**Profile Completion → Personalization**:
```
PUT /profile/:userId
  → Validate input (Zod)
  → Update backgrounds (transaction)
  → Mark profile complete
  → Audit log mutation
  → Invalidate cache
  → Build personalization context
  → Store preferences (Neon DB)
  → Cache context (Redis)
  → Return new JWT
```

**Signin → Cached Profile Retrieval**:
```
POST /auth/signin
  → Better Auth validates credentials
  → Check Redis for profile (cache hit?)
  → [Cache miss] Query Neon DB with JOINs
  → [Cache hit] Return cached profile
  → Generate JWT with profile claims
  → Store refresh token
  → Log auth attempt
  → Return tokens + profile
```

**Protected Resource Access**:
```
GET /api/recommendations (requires personalized_content policy)
  → Extract JWT from Authorization header
  → Validate signature and expiration
  → Check Redis blacklist (cache hit?)
  → [Cache miss] Query Neon DB revoked_tokens
  → Verify profileComplete claim == true
  → Log authz attempt
  → Return personalized recommendations
```

---

## API Contract Summary

### Authentication Endpoints

```yaml
POST /api/auth/signup:
  Request:
    email: string (required, email format)
    password: string (required, min 8 chars, uppercase+lowercase+number)
    confirmPassword: string (required, must match password)
    name: string (optional)
  Response (201):
    userId: string (UUID)
    accessToken: string (JWT, 15min expiry)
    refreshToken: string (32-byte hex)
    profileComplete: boolean (always false on signup)
  Errors:
    400: Validation failed
    409: Email already registered
    500: Internal server error

POST /api/auth/signin:
  Request:
    email: string (required)
    password: string (required)
  Response (200):
    userId: string
    accessToken: string
    refreshToken: string
    profile: CompleteProfile
  Errors:
    401: Invalid credentials
    500: Internal server error

POST /api/auth/refresh:
  Request:
    refreshToken: string (required)
  Response (200):
    accessToken: string (new JWT)
    refreshToken: string (new or same depending on rotation)
  Errors:
    401: Invalid or expired refresh token
    500: Internal server error

POST /api/auth/signout:
  Headers:
    Authorization: Bearer <accessToken> (required)
  Request:
    refreshToken: string (required)
  Response (200):
    success: boolean
  Errors:
    401: Invalid or missing token
    500: Internal server error
```

### Profile Endpoints

```yaml
GET /api/profile/:userId:
  Headers:
    Authorization: Bearer <accessToken> (required)
  Authorization: self_profile (user can only access own profile)
  Response (200):
    userProfile: {id, email, name, profileComplete, createdAt}
    softwareBackground: {languages, frameworks, experienceLevel, specializations, yearsOfExperience}
    hardwareBackground: {familiarPlatforms, roboticsExperience, electronicsKnowledge, preferredTools}
  Errors:
    401: Unauthorized
    403: Forbidden (cannot access other user's profile)
    404: Profile not found
    500: Internal server error

PUT /api/profile/:userId:
  Headers:
    Authorization: Bearer <accessToken> (required)
  Authorization: self_profile
  Request:
    softwareBackground: SoftwareBackgroundData (required)
    hardwareBackground: HardwareBackgroundData (required)
  Response (200):
    profile: CompleteProfile
    accessToken: string (new JWT with updated claims)
  Errors:
    400: Validation failed
    401: Unauthorized
    403: Forbidden
    500: Internal server error
```

### Personalization Endpoints

```yaml
GET /api/personalization/context:
  Headers:
    Authorization: Bearer <accessToken> (required)
  Authorization: personalized_content (requires profileComplete: true)
  Response (200):
    PersonalizationContext (full structure)
  Errors:
    401: Unauthorized
    403: Profile not complete
    500: Internal server error

POST /api/personalization/refresh:
  Headers:
    Authorization: Bearer <accessToken> (required)
  Authorization: personalized_content
  Response (200):
    PersonalizationContext (rebuilt from latest profile data)
  Errors:
    401: Unauthorized
    403: Profile not complete
    500: Internal server error
```

---

## Dependencies & Integration Points

### External Services
1. **Better Auth**: User creation, credential verification
2. **Neon DB**: Primary persistent storage
3. **Redis**: Caching layer (tokens, profiles, context)

### Internal Services
1. **Frontend (Docusaurus)**: Signup/signin forms, profile completion forms, token storage
2. **Content Service**: Consumes personalization context for recommendations
3. **RAG System**: Existing Cohere + Qdrant + Gemini system (integration point for personalized responses)

### Environment Setup
```bash
# Backend dependencies
npm install better-auth
npm install @neondatabase/serverless
npm install jsonwebtoken
npm install redis
npm install zod

# Development dependencies
npm install --save-dev @types/node @types/jsonwebtoken
npm install --save-dev jest @types/jest ts-jest
```

---

## Risk Mitigation

### High Priority Risks

**Risk**: Neon DB connection failures impact authentication
**Mitigation**: Implement retry logic with exponential backoff (max 3 retries), use Redis for critical operations (tokens), monitor connection pool health

**Risk**: Redis unavailability breaks token validation
**Mitigation**: Fallback to Neon DB for token_sessions and revoked_tokens queries, accept performance degradation, implement circuit breaker pattern

**Risk**: JWT secret compromise
**Mitigation**: Secure secret storage in environment variables, implement secret rotation procedure (requires user re-authentication), monitor for anomalous token usage

**Risk**: Profile completion bottleneck during signup
**Mitigation**: Make profile completion optional initially, allow users to complete later, show clear progress indicator, validate incrementally

**Risk**: Personalization context build performance
**Mitigation**: Build context async after profile completion, cache aggressively in Redis, pre-warm cache on signin, paginate recommendations

### Medium Priority Risks

**Risk**: Audit log table growth
**Mitigation**: Implement log retention policy, archive old logs to cold storage, create indexes on timestamp for efficient queries

**Risk**: Token blacklist growth
**Mitigation**: Automatic cleanup job to delete expired tokens from revoked_tokens table, TTL-based expiration in Redis

**Risk**: Better Auth version updates breaking changes
**Mitigation**: Pin Better Auth version, test updates in staging, review changelog before upgrading, maintain fallback authentication

---

## Success Metrics

### Performance Targets (from spec.md)
- [ ] Signup completes in < 1 minute
- [ ] Signin completes in < 10 seconds
- [ ] Profile completion in < 3 minutes
- [ ] JWT validation < 50ms (including blacklist check)
- [ ] Profile retrieval (cache hit) < 100ms
- [ ] Profile retrieval (cache miss) < 500ms
- [ ] Personalization context build < 1 second
- [ ] System handles 1,000 concurrent auth requests
- [ ] Profile cache hit rate > 80%

### Reliability Targets
- [ ] 99.9% uptime for authentication services
- [ ] Zero authentication bypasses
- [ ] 100% revoked token rejection rate
- [ ] All auth/authz events logged with audit trail
- [ ] Graceful degradation when Redis unavailable

### Security Targets
- [ ] Password requirements enforced (min 8 chars, uppercase+lowercase+number)
- [ ] Rate limiting prevents brute force (max 5 attempts per email per minute)
- [ ] JWT signed with HS256
- [ ] Sensitive data encrypted (AES-256)
- [ ] Row-level security policies active in Neon DB
- [ ] All queries use prepared statements (SQL injection prevention)

---

## Timeline Estimate

| Phase | Tasks | Effort | Dependencies |
|-------|-------|--------|--------------|
| Phase 0 | Research & Validation | 2-3 days | None |
| Phase 1 | Data Layer & Schema | 4-5 days | Phase 0 |
| Phase 2 | Authentication Core | 5-6 days | Phase 1 |
| Phase 3 | Profile Management | 3-4 days | Phase 2 |
| Phase 4 | Personalization | 4-5 days | Phase 3 |
| Phase 5 | Testing & Deployment | 4-5 days | Phases 1-4 |
| **Total** | **All Phases** | **22-28 days** | Sequential |

**Critical Path**: Phase 0 → Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5

**Parallel Opportunities**: Phase 5 testing can begin once Phase 2 is complete (test authentication while building profile management)

---

## Next Steps

1. **Immediate**: Review and approve this implementation plan
2. **Phase 0**: Begin research on Neon DB setup and Better Auth integration patterns
3. **Kickoff**: Set up project structure, install dependencies, configure development environment
4. **Communication**: Share plan with frontend team for coordination on API integration
5. **Monitoring**: Set up development Neon DB project and Redis instance for testing

---

## Appendix: File Structure

```
backend/
├── config/
│   └── database.ts           # Neon DB connection config
├── db/
│   ├── connection.ts          # Connection pool manager
│   ├── query-builder.ts       # Query helpers
│   └── transactions.ts        # Transaction wrapper
├── migrations/
│   ├── 001_create_core_schema.sql
│   ├── 002_create_indexes.sql
│   └── 003_create_rls_policies.sql
├── dal/
│   ├── user-repository.ts
│   ├── profile-repository.ts
│   ├── token-repository.ts
│   └── audit-repository.ts
├── schemas/
│   └── profile-schemas.ts     # Zod validation
├── auth/
│   ├── better-auth-config.ts
│   └── providers/
│       └── email-password.ts
├── services/
│   ├── jwt-service.ts
│   ├── personalization-service.ts
│   └── content-service.ts
├── agents/
│   ├── auth-orchestration-agent.ts
│   ├── profile-manager-agent.ts
│   ├── authorization-guard-agent.ts
│   └── personalization-context-agent.ts
├── middleware/
│   ├── jwt-validation.ts
│   └── auth-guard.ts
├── policies/
│   └── authorization-policies.ts
├── cache/
│   ├── redis-client.ts
│   └── cache-manager.ts
├── routes/
│   ├── auth.ts
│   ├── profile.ts
│   └── personalization.ts
├── tests/
│   ├── dal/*.test.ts
│   ├── services/*.test.ts
│   ├── agents/*.test.ts
│   └── integration/*.test.ts
└── docs/
    ├── api-spec.yaml
    ├── authentication-guide.md
    └── profile-management.md
```

---

**Plan Status**: ✅ Complete and ready for implementation
**Next Command**: `/sp.tasks` to generate granular task breakdown
