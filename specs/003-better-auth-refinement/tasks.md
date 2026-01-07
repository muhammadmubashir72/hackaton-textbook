# Development Tasks: Better Auth with Neon DB Integration

**Feature**: Better Auth Authentication System with Neon DB Storage
**Branch**: `003-better-auth-neon-db`
**Created**: 2025-12-24
**Status**: Ready for Implementation

## Task Organization

Tasks are organized into 5 phases with clear dependencies. Each task includes:
- **Task ID**: Unique identifier (e.g., P0-T1)
- **Description**: What needs to be done
- **Expected Outcome**: Concrete deliverable or acceptance criteria
- **Dependencies**: Which tasks must complete first
- **Estimated Effort**: Time estimate (S=Small <4h, M=Medium 4-8h, L=Large 1-2d, XL=Multiple days)

---

## Phase 0: Research & Technical Validation

**Objective**: Validate technology choices and document integration patterns

### P0-T1: Research Neon DB Connection Pooling
**Description**: Research and document Neon DB connection pooling best practices for serverless environments. Investigate @neondatabase/serverless driver configuration for optimal performance.

**Expected Outcome**:
- Document `research/neon-db-setup.md` with connection pooling configuration
- Environment variables documented (NEON_DATABASE_URL, pool min/max settings)
- Retry logic and timeout recommendations

**Dependencies**: None
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] Connection pool configuration documented (min 2, max 10 connections)
- [ ] Timeout settings defined (connection, query, idle)
- [ ] Retry logic pattern documented (exponential backoff, max 3 retries)

---

### P0-T2: Research Neon DB Schema Migrations
**Description**: Research Neon DB branching capability for schema migrations. Document migration strategy for zero-downtime deployments.

**Expected Outcome**:
- Document `research/neon-db-migrations.md` with migration workflow
- Database branching strategy for testing migrations
- Rollback procedures

**Dependencies**: None
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] Migration workflow documented (branch creation, testing, merge)
- [ ] SQL migration file structure defined
- [ ] Rollback procedures documented

---

### P0-T3: Research Better Auth Integration
**Description**: Research Better Auth email/password provider setup. Document callback hooks for profile initialization and session management.

**Expected Outcome**:
- Document `research/better-auth-integration.md` with setup guide
- Better Auth configuration template
- Callback hook patterns

**Dependencies**: None
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Email/password provider configuration documented
- [ ] Password validation requirements (min 8 chars, uppercase, lowercase, number) configured
- [ ] Callback hooks for onSignUp documented
- [ ] Session management strategy defined

---

### P0-T4: Research JWT Security
**Description**: Research JWT secret rotation procedures, refresh token patterns, and blacklist storage strategies.

**Expected Outcome**:
- Document `research/jwt-security.md` with token management best practices
- Secret rotation procedure
- Blacklist cleanup job specifications

**Dependencies**: None
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] JWT signing algorithm (HS256) validated
- [ ] Token expiration strategy documented (15min access, 7d refresh)
- [ ] Secret rotation procedure defined
- [ ] Blacklist storage pattern (Redis + Neon DB) documented

---

### P0-T5: Research Redis-Neon DB Fallback
**Description**: Research graceful degradation patterns when Redis is unavailable. Document dual-write vs cache-aside patterns for token storage.

**Expected Outcome**:
- Document `research/redis-neon-fallback.md` with fallback patterns
- Performance trade-offs documented
- Circuit breaker implementation strategy

**Dependencies**: None
**Effort**: M (5h)

**Acceptance Criteria**:
- [ ] Fallback pattern documented (Redis primary, Neon DB secondary)
- [ ] Performance impact measured/estimated
- [ ] Circuit breaker pattern for Redis failures defined

---

### P0-T6: Consolidate Research Findings
**Description**: Consolidate all research findings into single `research.md` document with decisions, rationale, and alternatives considered.

**Expected Outcome**:
- Document `research.md` with all technical decisions
- Clear recommendations for each technology choice

**Dependencies**: P0-T1, P0-T2, P0-T3, P0-T4, P0-T5
**Effort**: S (2h)

**Acceptance Criteria**:
- [ ] All research findings consolidated
- [ ] Decision rationale documented for each choice
- [ ] Alternatives considered section complete

---

## Phase 1: Data Layer & Schema Implementation

**Objective**: Set up Neon DB with complete schema and data access layer

### P1-T1: Initialize Neon DB Project
**Description**: Create Neon DB project via dashboard/CLI. Configure connection string, API key, and environment variables.

**Expected Outcome**:
- Neon DB project created with development and production databases
- Environment variables configured
- Connection successful from backend

**Dependencies**: P0-T6 (research complete)
**Effort**: S (2h)

**Acceptance Criteria**:
- [ ] Neon DB project created
- [ ] `.env.example` created with NEON_DATABASE_URL, NEON_API_KEY
- [ ] Connection test passes from backend
- [ ] Development and production databases created

---

### P1-T2: Create Core Schema Migration
**Description**: Write SQL migration script for core tables (users, profiles, software_backgrounds, hardware_backgrounds, user_preferences).

**Expected Outcome**:
- Migration file `001_create_core_schema.sql` with all core tables
- Foreign key constraints defined
- Default values and NOT NULL constraints applied

**Dependencies**: P1-T1
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] All 5 core tables created with correct data types
- [ ] Foreign keys reference users/profiles correctly
- [ ] CASCADE DELETE constraints on profile deletion
- [ ] UNIQUE constraints on user_id in profiles
- [ ] TEXT[] arrays for languages, frameworks, platforms, tools
- [ ] ENUM-like constraints (CHECK) for experience levels

**SQL Structure**:
```sql
-- users table
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255),
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- profiles table
CREATE TABLE profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  profile_complete BOOLEAN DEFAULT FALSE,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(user_id)
);

-- software_backgrounds table
-- hardware_backgrounds table
-- user_preferences table
```

---

### P1-T3: Create Token Management Schema
**Description**: Write SQL migration for token_sessions and revoked_tokens tables.

**Expected Outcome**:
- Migration file `002_create_token_tables.sql` with token management tables
- Indexes on token columns for fast lookups

**Dependencies**: P1-T2
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] token_sessions table created (id, token, user_id, created_at, expires_at, last_used_at, is_revoked)
- [ ] revoked_tokens table created (id, token_id, user_id, revoked_at, expires_at, reason)
- [ ] Indexes on token, user_id, expires_at columns
- [ ] Foreign keys to users table

---

### P1-T4: Create Audit Log Schema
**Description**: Write SQL migration for audit log tables (auth_logs, authz_logs, audit_logs).

**Expected Outcome**:
- Migration file `003_create_audit_tables.sql` with all audit tables
- Indexes on timestamp columns for efficient queries

**Dependencies**: P1-T2
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] auth_logs table created (id, user_id, email, ip_address, user_agent, success, failure_reason, timestamp)
- [ ] authz_logs table created (id, user_id, resource, action, policy, authorized, timestamp)
- [ ] audit_logs table created (id, user_id, operation, table_name, record_id, old_values JSONB, new_values JSONB, timestamp)
- [ ] Indexes on user_id, timestamp columns
- [ ] INET type for ip_address

---

### P1-T5: Create Database Indexes
**Description**: Write SQL migration for performance indexes on foreign keys and frequently queried columns.

**Expected Outcome**:
- Migration file `004_create_indexes.sql` with all performance indexes

**Dependencies**: P1-T2, P1-T3, P1-T4
**Effort**: S (2h)

**Acceptance Criteria**:
- [ ] Index on users.email
- [ ] Index on profiles.user_id
- [ ] Index on token_sessions(user_id, is_revoked, expires_at) - composite
- [ ] Index on revoked_tokens.token_id
- [ ] Index on revoked_tokens.expires_at
- [ ] Indexes on all audit log timestamp columns

---

### P1-T6: Implement Row-Level Security Policies
**Description**: Write SQL migration for RLS policies to prevent cross-user data access.

**Expected Outcome**:
- Migration file `005_create_rls_policies.sql` with RLS policies
- Policies enforce user can only access their own data

**Dependencies**: P1-T2
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] RLS enabled on profiles, software_backgrounds, hardware_backgrounds, user_preferences
- [ ] Policy: users can SELECT/UPDATE own profile (user_id = current_user_id)
- [ ] Policy: admins can SELECT any profile
- [ ] Policies tested with different user contexts

---

### P1-T7: Run All Migrations
**Description**: Execute all migration scripts against Neon DB development database. Verify schema correctness.

**Expected Outcome**:
- All tables created successfully
- Indexes and constraints applied
- Migration history tracked

**Dependencies**: P1-T2, P1-T3, P1-T4, P1-T5, P1-T6
**Effort**: S (2h)

**Acceptance Criteria**:
- [ ] All migrations run without errors
- [ ] Database schema matches specification
- [ ] Can query all tables successfully
- [ ] Foreign key constraints enforced

---

### P1-T8: Create Neon DB Connection Module
**Description**: Implement database connection wrapper using @neondatabase/serverless with connection pooling and retry logic.

**Expected Outcome**:
- File `backend/db/connection.ts` with connection pool manager
- Retry logic for transient failures

**Dependencies**: P1-T1
**Effort**: M (5h)

**Acceptance Criteria**:
- [ ] Connection pool configured (min 2, max 10 connections)
- [ ] Retry logic implemented (max 3 retries, exponential backoff)
- [ ] Connection timeout configured (10s)
- [ ] Graceful error handling
- [ ] Connection pool health monitoring

**Code Structure**:
```typescript
import { Pool } from '@neondatabase/serverless'

export const pool = new Pool({
  connectionString: process.env.NEON_DATABASE_URL,
  min: 2,
  max: 10,
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 10000
})

export async function query(sql: string, params: any[]): Promise<QueryResult>
export async function queryWithRetry(sql: string, params: any[], maxRetries = 3): Promise<QueryResult>
```

---

### P1-T9: Create Query Builder Helpers
**Description**: Implement query helper functions with prepared statements for SELECT, INSERT, UPDATE, DELETE operations.

**Expected Outcome**:
- File `backend/db/query-builder.ts` with query helpers
- All queries use prepared statements (SQL injection prevention)

**Dependencies**: P1-T8
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Helper functions for common queries
- [ ] Prepared statements used for all queries
- [ ] Type-safe parameter binding
- [ ] Query logging for debugging

---

### P1-T10: Create Transaction Wrapper
**Description**: Implement transaction management wrapper for atomic multi-table operations.

**Expected Outcome**:
- File `backend/db/transactions.ts` with transaction helper
- ROLLBACK on error, COMMIT on success

**Dependencies**: P1-T8
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] Transaction wrapper function created
- [ ] Automatic ROLLBACK on errors
- [ ] COMMIT on successful completion
- [ ] Nested transaction support (savepoints)

**Code Structure**:
```typescript
export async function transaction<T>(callback: (client: PoolClient) => Promise<T>): Promise<T> {
  const client = await pool.connect()
  try {
    await client.query('BEGIN')
    const result = await callback(client)
    await client.query('COMMIT')
    return result
  } catch (error) {
    await client.query('ROLLBACK')
    throw error
  } finally {
    client.release()
  }
}
```

---

### P1-T11: Create User Repository
**Description**: Implement user repository with CRUD operations (create, findByEmail, findById).

**Expected Outcome**:
- File `backend/dal/user-repository.ts` with user operations
- All operations use prepared statements

**Dependencies**: P1-T9, P1-T10
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] createUser(email, passwordHash, name) implemented
- [ ] findUserByEmail(email) implemented
- [ ] findUserById(id) implemented
- [ ] All queries use prepared statements
- [ ] Error handling for duplicate emails

---

### P1-T12: Create Profile Repository
**Description**: Implement profile repository with CRUD operations and complete profile retrieval with JOINs.

**Expected Outcome**:
- File `backend/dal/profile-repository.ts` with profile operations
- Efficient JOIN query for complete profile data

**Dependencies**: P1-T9, P1-T10
**Effort**: L (8h)

**Acceptance Criteria**:
- [ ] createProfile(userId) implemented
- [ ] updateProfile(userId, data) implemented
- [ ] getCompleteProfile(userId) implemented with JOINs
- [ ] markProfileComplete(userId) implemented
- [ ] checkProfileCompletion(softwareBackground, hardwareBackground) logic
- [ ] Single query retrieves user + profile + backgrounds

**JOIN Query Structure**:
```sql
SELECT
  u.id, u.email, u.name, u.created_at,
  p.id as profile_id, p.profile_complete,
  sb.programming_languages, sb.frameworks, sb.experience_level, sb.specializations, sb.years_of_experience,
  hb.familiar_platforms, hb.robotics_experience, hb.electronics_knowledge, hb.preferred_tools
FROM users u
JOIN profiles p ON p.user_id = u.id
JOIN software_backgrounds sb ON sb.profile_id = p.id
JOIN hardware_backgrounds hb ON hb.profile_id = p.id
WHERE u.id = $1
```

---

### P1-T13: Create Token Repository
**Description**: Implement token repository for refresh token and blacklist management.

**Expected Outcome**:
- File `backend/dal/token-repository.ts` with token operations
- Support for both Redis and Neon DB storage

**Dependencies**: P1-T9, P1-T10
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] storeRefreshToken(token, userId, expiresAt) implemented
- [ ] validateRefreshToken(token) implemented
- [ ] revokeRefreshToken(token) implemented
- [ ] addToBlacklist(tokenId, userId, expiresAt, reason) implemented
- [ ] isTokenBlacklisted(tokenId) implemented
- [ ] Cleanup expired tokens method

---

### P1-T14: Create Audit Repository
**Description**: Implement audit repository for logging authentication, authorization, and database mutations.

**Expected Outcome**:
- File `backend/dal/audit-repository.ts` with audit logging functions

**Dependencies**: P1-T9
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] logAuthAttempt(email, success, ipAddress, userAgent, failureReason) implemented
- [ ] logAuthzAttempt(userId, resource, policy, authorized) implemented
- [ ] logDatabaseMutation(userId, operation, tableName, recordId, oldValues, newValues) implemented
- [ ] All logs written asynchronously (non-blocking)

---

## Phase 2: Authentication Core (Better Auth + JWT)

**Objective**: Implement authentication flows with Better Auth and JWT token management

### P2-T1: Install Dependencies
**Description**: Install all required npm packages for authentication and database access.

**Expected Outcome**:
- All dependencies installed and package.json updated

**Dependencies**: P1-T7 (schema ready)
**Effort**: S (1h)

**Acceptance Criteria**:
- [ ] better-auth installed
- [ ] @neondatabase/serverless installed
- [ ] jsonwebtoken installed
- [ ] redis installed
- [ ] zod installed
- [ ] TypeScript types installed

---

### P2-T2: Configure Better Auth
**Description**: Initialize Better Auth with email/password provider and Neon DB integration.

**Expected Outcome**:
- File `backend/auth/better-auth-config.ts` with Better Auth setup
- Callback hook for profile initialization

**Dependencies**: P2-T1, P1-T14 (repositories ready)
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] Better Auth initialized with Neon DB provider
- [ ] Email/password provider configured
- [ ] Password validation rules enforced (min 8 chars, uppercase, lowercase, number)
- [ ] onSignUp callback creates empty profile
- [ ] Session settings configured (15min expiry)

**Code Structure**:
```typescript
import { betterAuth } from "better-auth"
import { profileRepository } from '../dal/profile-repository'

export const auth = betterAuth({
  database: {
    provider: "neon",
    url: process.env.NEON_DATABASE_URL
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    requireUppercase: true,
    requireLowercase: true,
    requireNumber: true
  },
  callbacks: {
    async onSignUp(user) {
      await profileRepository.createProfile(user.id)
    }
  }
})
```

---

### P2-T3: Implement JWT Generation
**Description**: Create JWT generation function with custom claims (profileComplete, software/hardware backgrounds).

**Expected Outcome**:
- File `backend/services/jwt-service.ts` with JWT operations
- Access and refresh token generation

**Dependencies**: P2-T1
**Effort**: M (5h)

**Acceptance Criteria**:
- [ ] generateAccessToken(user, profile) creates JWT with claims
- [ ] generateRefreshToken() creates random 32-byte hex token
- [ ] JWT signed with HS256 algorithm
- [ ] Claims include: sub, email, name, profileComplete, softwareBackground, hardwareBackground, roles, iat, exp, iss
- [ ] Access token expiry 15 minutes
- [ ] Issuer set to 'physical-ai-textbook'

---

### P2-T4: Implement JWT Validation
**Description**: Create JWT validation function with signature verification and blacklist checking.

**Expected Outcome**:
- JWT validation function in `backend/services/jwt-service.ts`
- Blacklist check integrated

**Dependencies**: P2-T3, P1-T13 (token repository)
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] validateAccessToken(token) verifies signature
- [ ] Expiration checked
- [ ] Blacklist checked (Redis → Neon DB fallback)
- [ ] Returns decoded payload or null
- [ ] Invalid/expired tokens rejected

---

### P2-T5: Configure Redis Connection
**Description**: Set up Redis client with connection retry logic and error handling.

**Expected Outcome**:
- File `backend/cache/redis-client.ts` with Redis setup
- Retry logic for connection failures

**Dependencies**: P2-T1
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] Redis client configured with REDIS_URL
- [ ] Connection retry logic (max 3 retries)
- [ ] Graceful handling when Redis unavailable
- [ ] Connection health monitoring

---

### P2-T6: Implement Token Caching
**Description**: Create cache helper functions for refresh tokens and JWT blacklist with Redis-Neon DB dual storage.

**Expected Outcome**:
- File `backend/cache/cache-manager.ts` with caching helpers
- Dual-write pattern for tokens

**Dependencies**: P2-T5, P1-T13
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] cacheRefreshToken(token, userId, ttl) writes to Redis + Neon DB
- [ ] getRefreshToken(token) checks Redis → Neon DB fallback
- [ ] addToBlacklist(tokenId, userId, expiresAt, reason) writes to both
- [ ] isTokenBlacklisted(tokenId) checks Redis → Neon DB fallback
- [ ] Fallback to Neon DB when Redis unavailable

---

### P2-T7: Implement Profile Caching
**Description**: Create cache functions for complete user profiles with 1-hour TTL.

**Expected Outcome**:
- Profile caching functions in `backend/cache/cache-manager.ts`
- Cache invalidation on profile updates

**Dependencies**: P2-T5, P1-T12
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] cacheProfile(userId, profile) stores in Redis (1h TTL)
- [ ] getCachedProfile(userId) retrieves from Redis
- [ ] invalidateProfileCache(userId) deletes from Redis
- [ ] Graceful handling when Redis unavailable

---

### P2-T8: Implement Auth Orchestration Agent - Signup Flow
**Description**: Create signup flow coordinator that integrates Better Auth, profile initialization, and JWT generation.

**Expected Outcome**:
- File `backend/agents/auth-orchestration-agent.ts` with signup handler
- POST /api/auth/signup endpoint

**Dependencies**: P2-T2, P2-T3, P2-T6, P1-T11, P1-T12, P1-T14
**Effort**: L (8h)

**Acceptance Criteria**:
- [ ] handleSignup(email, password, name) function implemented
- [ ] Input validation (email format, password strength, confirmation match)
- [ ] Email uniqueness check
- [ ] User created via Better Auth
- [ ] Empty profile initialized (callback triggered)
- [ ] JWT generated with profileComplete: false
- [ ] Refresh token generated and stored (Redis + Neon DB)
- [ ] Auth attempt logged
- [ ] Returns {userId, accessToken, refreshToken, profileComplete}
- [ ] Errors: 400 (validation), 409 (duplicate email), 500 (server error)

---

### P2-T9: Implement Auth Orchestration Agent - Signin Flow
**Description**: Create signin flow coordinator that validates credentials, retrieves profile, and generates tokens.

**Expected Outcome**:
- Signin handler in `backend/agents/auth-orchestration-agent.ts`
- POST /api/auth/signin endpoint

**Dependencies**: P2-T2, P2-T3, P2-T4, P2-T6, P2-T7, P1-T12, P1-T14
**Effort**: L (8h)

**Acceptance Criteria**:
- [ ] handleSignin(email, password) function implemented
- [ ] Credentials validated via Better Auth
- [ ] Failed attempts logged to auth_logs
- [ ] Complete profile retrieved (check Redis cache first)
- [ ] Profile cached if not in Redis
- [ ] JWT generated with profile claims
- [ ] Refresh token generated and stored
- [ ] Successful auth logged
- [ ] Returns {userId, accessToken, refreshToken, profile}
- [ ] Errors: 401 (invalid credentials), 500 (server error)

---

### P2-T10: Implement Auth Orchestration Agent - Token Refresh Flow
**Description**: Create token refresh flow that validates refresh token and generates new JWT with updated profile claims.

**Expected Outcome**:
- Refresh handler in `backend/agents/auth-orchestration-agent.ts`
- POST /api/auth/refresh endpoint

**Dependencies**: P2-T3, P2-T4, P2-T6, P1-T12, P1-T13
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] handleRefresh(refreshToken) function implemented
- [ ] Refresh token validated (Redis → Neon DB fallback)
- [ ] Token not revoked check
- [ ] last_used_at updated in token_sessions
- [ ] Current profile data retrieved
- [ ] New JWT generated with latest claims
- [ ] New refresh token generated (if rotation enabled)
- [ ] Returns {accessToken, refreshToken}
- [ ] Errors: 401 (invalid/expired refresh token), 500 (server error)

---

### P2-T11: Implement Auth Orchestration Agent - Signout Flow
**Description**: Create signout flow that revokes both access and refresh tokens.

**Expected Outcome**:
- Signout handler in `backend/agents/auth-orchestration-agent.ts`
- POST /api/auth/signout endpoint

**Dependencies**: P2-T4, P2-T6, P1-T13
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] handleSignout(accessToken, refreshToken) function implemented
- [ ] Access token validated
- [ ] JWT added to blacklist (Redis + Neon DB)
- [ ] Refresh token revoked (Redis + Neon DB)
- [ ] Better Auth session cleared
- [ ] Returns {success: true}
- [ ] Errors: 401 (invalid token), 500 (server error)

---

### P2-T12: Create Authentication Routes
**Description**: Create Express/Fastify routes for all authentication endpoints.

**Expected Outcome**:
- File `backend/routes/auth.ts` with all auth routes

**Dependencies**: P2-T8, P2-T9, P2-T10, P2-T11
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] POST /api/auth/signup route
- [ ] POST /api/auth/signin route
- [ ] POST /api/auth/refresh route
- [ ] POST /api/auth/signout route
- [ ] Input validation middleware
- [ ] Error handling middleware
- [ ] Request/response logging

---

## Phase 3: Profile Management & Background Collection

**Objective**: Implement profile data collection, validation, and authorization

### P3-T1: Create Profile Validation Schemas
**Description**: Define Zod schemas for software and hardware background validation.

**Expected Outcome**:
- File `backend/schemas/profile-schemas.ts` with validation schemas

**Dependencies**: P2-T1 (Zod installed)
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] SoftwareBackgroundSchema defined (languages min 1 max 20, frameworks max 20, experienceLevel enum, specializations max 10, yearsOfExperience 0-50)
- [ ] HardwareBackgroundSchema defined (platforms min 1 max 10, roboticsExperience enum, electronicsKnowledge enum, tools max 10)
- [ ] ProfileUpdateSchema combines both backgrounds
- [ ] Enum validation for experience levels

**Code Structure**:
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

---

### P3-T2: Implement Profile Manager Agent - Profile Update
**Description**: Create profile update handler with validation, transaction management, and audit logging.

**Expected Outcome**:
- File `backend/agents/profile-manager-agent.ts` with update logic

**Dependencies**: P3-T1, P1-T12, P1-T14, P2-T7, P2-T3
**Effort**: L (10h)

**Acceptance Criteria**:
- [ ] updateProfile(userId, data) function implemented
- [ ] Input validated with Zod schemas
- [ ] Authorization check (self-profile access only)
- [ ] Current profile retrieved for audit log
- [ ] Update in database transaction (software_backgrounds + hardware_backgrounds + profiles)
- [ ] Profile completion check logic implemented
- [ ] Profile marked complete if criteria met
- [ ] Audit log written (old values vs new values)
- [ ] Profile cache invalidated
- [ ] New JWT generated with updated claims
- [ ] profile.updated event emitted
- [ ] Returns {profile, accessToken}
- [ ] Errors: 400 (validation), 401 (unauthorized), 403 (forbidden), 500 (server error)

**Profile Completion Criteria**:
```typescript
function checkProfileCompletion(software, hardware): boolean {
  return (
    software.programmingLanguages.length >= 1 &&
    software.experienceLevel !== undefined &&
    hardware.familiarPlatforms.length >= 1 &&
    hardware.roboticsExperience !== undefined &&
    hardware.electronicsKnowledge !== undefined
  )
}
```

---

### P3-T3: Implement Profile Manager Agent - Profile Retrieval
**Description**: Create profile retrieval handler with caching and authorization.

**Expected Outcome**:
- Profile retrieval in `backend/agents/profile-manager-agent.ts`

**Dependencies**: P1-T12, P2-T7
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] getProfile(userId) function implemented
- [ ] Check Redis cache first
- [ ] Fallback to Neon DB with JOIN query if cache miss
- [ ] Cache profile in Redis after retrieval
- [ ] Returns CompleteProfile object
- [ ] Errors: 401 (unauthorized), 404 (not found), 500 (server error)

---

### P3-T4: Define Authorization Policies
**Description**: Create authorization policy definitions for authenticated, self_profile, admin, and personalized_content access.

**Expected Outcome**:
- File `backend/policies/authorization-policies.ts` with all policies

**Dependencies**: P2-T4 (JWT validation)
**Effort**: M (5h)

**Acceptance Criteria**:
- [ ] authenticated policy (valid JWT present)
- [ ] self_profile policy (userId in JWT == requested userId)
- [ ] admin policy (roles include 'admin')
- [ ] personalized_content policy (profileComplete == true)
- [ ] Policy inheritance (extends authenticated)

**Code Structure**:
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
      return req.user.sub === req.params.userId
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

---

### P3-T5: Implement Authorization Guard Middleware
**Description**: Create Express/Fastify middleware for JWT validation and policy enforcement.

**Expected Outcome**:
- File `backend/middleware/auth-guard.ts` with authorization middleware

**Dependencies**: P3-T4, P1-T14 (audit logging)
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] requireAuth(policyName) middleware function
- [ ] Extracts JWT from Authorization header (Bearer scheme)
- [ ] Validates token via JWT service
- [ ] Enforces specified policy
- [ ] Checks extended policies (inheritance)
- [ ] Attaches user context to request (req.user)
- [ ] Logs authorization attempts (authz_logs)
- [ ] Returns 401 for invalid/missing token
- [ ] Returns 403 for insufficient permissions
- [ ] Returns specific error for profile incomplete

**Error Responses**:
```typescript
// 401 Unauthorized
{ error: 'Unauthorized', message: 'Invalid or missing authentication token', code: 'AUTH_TOKEN_INVALID' }

// 403 Forbidden
{ error: 'Forbidden', message: 'You do not have permission to access this resource', code: 'AUTH_INSUFFICIENT_PERMISSIONS' }

// 403 Profile Incomplete
{ error: 'Forbidden', message: 'Please complete your profile to access personalized content', code: 'AUTH_PROFILE_INCOMPLETE', redirectTo: '/profile/complete' }
```

---

### P3-T6: Create Profile Routes
**Description**: Create Express/Fastify routes for profile endpoints with authorization.

**Expected Outcome**:
- File `backend/routes/profile.ts` with profile routes

**Dependencies**: P3-T2, P3-T3, P3-T5
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] GET /api/profile/:userId route with requireAuth('self_profile')
- [ ] PUT /api/profile/:userId route with requireAuth('self_profile')
- [ ] Input validation middleware
- [ ] Error handling middleware
- [ ] Request/response logging

---

## Phase 4: Personalization Context & Integration

**Objective**: Build personalization context and integrate with content system

### P4-T1: Implement Expertise Mapping Functions
**Description**: Create helper functions to map user background data to expertise levels.

**Expected Outcome**:
- File `backend/services/personalization-service.ts` with expertise mapping

**Dependencies**: P1-T12 (profile repository)
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] mapSoftwareExpertise(softwareBackground) function
- [ ] mapHardwareExpertise(hardwareBackground) function
- [ ] inferLanguageProficiency() helper
- [ ] inferFrameworkProficiency() helper
- [ ] inferPlatformProficiency() helper
- [ ] Returns structured expertise object

**Mapping Logic**:
```typescript
function inferLanguageProficiency(experienceLevel: string, specializations: string[]): string {
  // Map experience level to language proficiency
  // Consider specializations for boost
}

function inferPlatformProficiency(roboticsExp: string, electronicsKnowledge: string): string {
  // Map robotics/electronics to platform proficiency
}
```

---

### P4-T2: Implement Learning Path Generation
**Description**: Create function to generate suggested next topics and identify skill gaps based on expertise.

**Expected Outcome**:
- Learning path generation in `backend/services/personalization-service.ts`

**Dependencies**: P4-T1
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] generateLearningPath(softwareExpertise, hardwareExpertise) function
- [ ] Identifies current level
- [ ] Suggests 3-5 next topics based on level and specializations
- [ ] Identifies skill gaps (required skills not in user's background)
- [ ] Returns LearningPath object

---

### P4-T3: Implement Content Preferences Builder
**Description**: Create function to build content preferences from expertise data.

**Expected Outcome**:
- Content preferences builder in `backend/services/personalization-service.ts`

**Dependencies**: P4-T1
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] buildContentPreferences(softwareExpertise, hardwareExpertise) function
- [ ] Sets difficulty setting based on overall level
- [ ] Identifies focus areas from specializations
- [ ] Determines preferred platforms
- [ ] Sets visibility flags (showAdvancedTopics, showBeginnerTopics)
- [ ] Returns ContentPreferences object

---

### P4-T4: Implement UI Adaptations Builder
**Description**: Create function to build UI adaptation settings from expertise.

**Expected Outcome**:
- UI adaptations builder in `backend/services/personalization-service.ts`

**Dependencies**: P4-T1
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] buildUIAdaptations(softwareExpertise, hardwareExpertise) function
- [ ] Sets code language (first from languages list, default 'Python')
- [ ] Sets preferred platform (first from platforms list, default 'Raspberry Pi')
- [ ] Determines code example visibility
- [ ] Determines hardware setup visibility
- [ ] Sets beginner tips visibility (hide for advanced/expert)
- [ ] Sets advanced notes visibility (show for advanced/expert)
- [ ] Returns UIAdaptations object

---

### P4-T5: Implement Recommendation Generation
**Description**: Create function to generate chapter and project recommendations based on user expertise.

**Expected Outcome**:
- Recommendation generation in `backend/services/personalization-service.ts`

**Dependencies**: P4-T1, P4-T3
**Effort**: L (8h)

**Acceptance Criteria**:
- [ ] generateRecommendations(contentPreferences, softwareExpertise, hardwareExpertise) function
- [ ] Queries content database for matching chapters (difficulty, focus areas, platforms)
- [ ] Queries content database for matching projects (frameworks, platforms)
- [ ] Returns top 5 chapter recommendations with reasons
- [ ] Returns top 5 project recommendations with reasons
- [ ] Integration with existing RAG/content system

---

### P4-T6: Implement User Preferences Persistence
**Description**: Create function to store user content preferences in Neon DB.

**Expected Outcome**:
- Preferences persistence in `backend/services/personalization-service.ts`

**Dependencies**: P1-T9 (query builder)
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] storeUserPreferences(userId, preferences) function
- [ ] Upsert to user_preferences table (INSERT ... ON CONFLICT UPDATE)
- [ ] Stores difficulty_setting, focus_areas (JSONB), preferred_platforms (JSONB)
- [ ] Updates updated_at timestamp

---

### P4-T7: Implement Personalization Context Agent - Build Context
**Description**: Create main context building function that orchestrates all personalization components.

**Expected Outcome**:
- File `backend/agents/personalization-context-agent.ts` with context builder

**Dependencies**: P4-T1, P4-T2, P4-T3, P4-T4, P4-T5, P4-T6, P2-T7
**Effort**: L (8h)

**Acceptance Criteria**:
- [ ] buildPersonalizationContext(userId) function implemented
- [ ] Retrieves complete profile
- [ ] Validates profile is complete
- [ ] Maps software expertise
- [ ] Maps hardware expertise
- [ ] Generates learning path
- [ ] Builds content preferences
- [ ] Generates recommendations
- [ ] Builds UI adaptations
- [ ] Stores preferences in Neon DB
- [ ] Caches context in Redis (1h TTL)
- [ ] Returns PersonalizationContext object
- [ ] Error if profile not complete

---

### P4-T8: Implement Context Cache Management
**Description**: Create cache functions for personalization context with invalidation on profile updates.

**Expected Outcome**:
- Context caching in `backend/agents/personalization-context-agent.ts`

**Dependencies**: P4-T7, P2-T5 (Redis client)
**Effort**: S (3h)

**Acceptance Criteria**:
- [ ] cachePersonalizationContext(userId, context) function
- [ ] getCachedPersonalizationContext(userId) function
- [ ] invalidatePersonalizationContext(userId) function
- [ ] 1-hour TTL for cached context
- [ ] Graceful handling when Redis unavailable

---

### P4-T9: Implement Event Handlers for Profile Updates
**Description**: Create event listeners that rebuild personalization context when profile changes.

**Expected Outcome**:
- Event handlers in `backend/agents/personalization-context-agent.ts`

**Dependencies**: P4-T7, P4-T8
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Listen for 'profile.updated' event
- [ ] Invalidate cached context on profile update
- [ ] Rebuild context if profile is complete
- [ ] Listen for 'profile.completed' event
- [ ] Build initial context on profile completion

---

### P4-T10: Create Personalization Routes
**Description**: Create Express/Fastify routes for personalization endpoints.

**Expected Outcome**:
- File `backend/routes/personalization.ts` with personalization routes

**Dependencies**: P4-T7, P3-T5 (auth middleware)
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] GET /api/personalization/context route with requireAuth('personalized_content')
- [ ] POST /api/personalization/refresh route with requireAuth('personalized_content')
- [ ] Error handling for incomplete profiles
- [ ] Request/response logging

---

## Phase 5: Testing, Documentation & Deployment

**Objective**: Comprehensive testing, documentation, and production deployment

### P5-T1: Write Unit Tests for Repositories
**Description**: Write comprehensive unit tests for all repository methods.

**Expected Outcome**:
- Test files in `backend/tests/dal/` for each repository

**Dependencies**: P1-T11, P1-T12, P1-T13, P1-T14
**Effort**: L (10h)

**Acceptance Criteria**:
- [ ] user-repository.test.ts covers createUser, findByEmail, findById
- [ ] profile-repository.test.ts covers all CRUD operations and JOIN query
- [ ] token-repository.test.ts covers token storage, validation, revocation, blacklist
- [ ] audit-repository.test.ts covers all log types
- [ ] All tests use test database
- [ ] Tests cover success and error cases
- [ ] Test coverage > 80% for repositories

---

### P5-T2: Write Unit Tests for Services
**Description**: Write unit tests for JWT service and personalization service.

**Expected Outcome**:
- Test files in `backend/tests/services/`

**Dependencies**: P2-T3, P2-T4, P4-T1, P4-T2, P4-T3, P4-T4, P4-T5
**Effort**: L (10h)

**Acceptance Criteria**:
- [ ] jwt-service.test.ts covers token generation, validation, refresh
- [ ] personalization-service.test.ts covers expertise mapping, learning path, recommendations
- [ ] Mock dependencies (repositories, Redis)
- [ ] Test coverage > 80% for services

---

### P5-T3: Write Integration Tests for Auth Flows
**Description**: Write end-to-end integration tests for all authentication flows.

**Expected Outcome**:
- Test files in `backend/tests/integration/`

**Dependencies**: P2-T8, P2-T9, P2-T10, P2-T11
**Effort**: L (12h)

**Acceptance Criteria**:
- [ ] auth-flows.test.ts covers signup → signin → refresh → signout
- [ ] Tests use real test database (not mocked)
- [ ] Tests verify JWT claims
- [ ] Tests verify database state changes
- [ ] Tests verify audit logs written
- [ ] Tests cover error scenarios (invalid credentials, duplicate email, expired tokens)

---

### P5-T4: Write Integration Tests for Profile Management
**Description**: Write integration tests for profile update and retrieval flows.

**Expected Outcome**:
- Profile integration tests in `backend/tests/integration/`

**Dependencies**: P3-T2, P3-T3, P3-T6
**Effort**: M (8h)

**Acceptance Criteria**:
- [ ] profile-flows.test.ts covers profile retrieval, update, completion
- [ ] Tests verify authorization (self-profile access)
- [ ] Tests verify profile completion logic
- [ ] Tests verify new JWT issued with updated claims
- [ ] Tests verify audit logs
- [ ] Tests verify cache invalidation

---

### P5-T5: Write Integration Tests for Personalization
**Description**: Write integration tests for personalization context building and retrieval.

**Expected Outcome**:
- Personalization integration tests in `backend/tests/integration/`

**Dependencies**: P4-T7, P4-T10
**Effort**: M (8h)

**Acceptance Criteria**:
- [ ] personalization-flows.test.ts covers context building, caching, retrieval
- [ ] Tests verify context structure
- [ ] Tests verify recommendations generated
- [ ] Tests verify cache behavior
- [ ] Tests verify authorization (requires complete profile)

---

### P5-T6: Generate API Documentation
**Description**: Create OpenAPI 3.0 specification for all API endpoints.

**Expected Outcome**:
- File `backend/docs/api-spec.yaml` with complete API documentation

**Dependencies**: P2-T12, P3-T6, P4-T10
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] All 8 endpoints documented (4 auth, 2 profile, 2 personalization)
- [ ] Request/response schemas defined
- [ ] Authentication requirements specified
- [ ] Error responses documented with codes
- [ ] Example requests/responses provided

---

### P5-T7: Write Authentication Guide
**Description**: Create developer documentation for authentication flows.

**Expected Outcome**:
- File `backend/docs/authentication-guide.md`

**Dependencies**: P2-T12
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Signup flow documented with examples
- [ ] Signin flow documented with examples
- [ ] Token refresh flow documented
- [ ] Signout flow documented
- [ ] JWT structure explained
- [ ] Authorization header format documented
- [ ] Error handling guide

---

### P5-T8: Write Profile Management Guide
**Description**: Create developer documentation for profile management.

**Expected Outcome**:
- File `backend/docs/profile-management-guide.md`

**Dependencies**: P3-T6
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Profile completion workflow documented
- [ ] Software/hardware background schemas explained
- [ ] Profile update process documented
- [ ] Authorization policies explained
- [ ] Validation rules documented

---

### P5-T9: Optimize Database Indexes
**Description**: Review query patterns and add additional indexes for performance.

**Expected Outcome**:
- Additional indexes created based on query analysis

**Dependencies**: P1-T5, P5-T3, P5-T4, P5-T5
**Effort**: M (4h)

**Acceptance Criteria**:
- [ ] Analyze slow query log
- [ ] Identify missing indexes
- [ ] Create composite indexes for complex queries
- [ ] Verify query performance improved
- [ ] Document index creation rationale

---

### P5-T10: Configure Production Environment
**Description**: Set up production environment variables, Neon DB production database, and Redis production instance.

**Expected Outcome**:
- Production environment ready for deployment

**Dependencies**: All phases complete
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] Production Neon DB project created
- [ ] Production Redis instance configured
- [ ] Environment variables documented in .env.production.example
- [ ] Secrets stored securely (not in code)
- [ ] Connection pooling configured for production workload
- [ ] Monitoring and logging configured

---

### P5-T11: Run Performance Benchmarks
**Description**: Execute performance tests to validate success criteria from specification.

**Expected Outcome**:
- Performance benchmark report

**Dependencies**: P5-T10
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] Signup completes in < 1 minute (test with 100 concurrent users)
- [ ] Signin completes in < 10 seconds (test with 100 concurrent users)
- [ ] JWT validation < 50ms (test 1000 validations)
- [ ] Profile retrieval (cache hit) < 100ms
- [ ] Profile retrieval (cache miss) < 500ms
- [ ] Personalization context build < 1 second
- [ ] System handles 1,000 concurrent auth requests
- [ ] Profile cache hit rate > 80% after warmup

---

### P5-T12: Deploy to Production
**Description**: Deploy backend to production environment (Hugging Face Spaces or similar).

**Expected Outcome**:
- Backend deployed and accessible via production URL

**Dependencies**: P5-T10, P5-T11
**Effort**: M (6h)

**Acceptance Criteria**:
- [ ] Backend deployed to production environment
- [ ] All endpoints accessible via HTTPS
- [ ] Health check endpoint responding
- [ ] Connection to production Neon DB verified
- [ ] Connection to production Redis verified
- [ ] Logs flowing to monitoring system
- [ ] Error tracking configured

---

## Summary Statistics

### Total Tasks by Phase
- **Phase 0**: 6 tasks (Research & Validation)
- **Phase 1**: 14 tasks (Data Layer)
- **Phase 2**: 12 tasks (Authentication Core)
- **Phase 3**: 6 tasks (Profile Management)
- **Phase 4**: 10 tasks (Personalization)
- **Phase 5**: 12 tasks (Testing & Deployment)

**Total**: 60 tasks

### Effort Distribution
- **Small (S)**: 19 tasks (~3h each) = 57 hours
- **Medium (M)**: 29 tasks (~5h each) = 145 hours
- **Large (L)**: 12 tasks (~8h each) = 96 hours

**Total Estimated Effort**: 298 hours (~37 working days at 8h/day)

### Critical Path
```
P0-T6 (Research Complete)
  → P1-T7 (Schema Ready)
    → P2-T12 (Auth Complete)
      → P3-T6 (Profile Management Complete)
        → P4-T10 (Personalization Complete)
          → P5-T12 (Production Deployed)
```

### Parallel Work Opportunities
- P0 tasks can be done in parallel (6 research tasks)
- P1-T2, P1-T3, P1-T4 can be done in parallel (schema creation)
- P2 caching tasks (P2-T5, P2-T6, P2-T7) can be done in parallel
- P5 testing tasks (P5-T1, P5-T2) can be done in parallel
- P5 documentation tasks (P5-T6, P5-T7, P5-T8) can be done in parallel

### Risk Mitigation Tasks
Each phase includes validation and testing tasks to catch issues early:
- P1-T7: Verify schema correctness before proceeding
- P2-T12: Test all auth flows before profile management
- P3-T6: Test authorization before personalization
- P5-T11: Performance validation before production

---

## Task Completion Checklist Template

For each task, use this checklist:

```markdown
## Task: [Task ID] [Task Name]

**Status**: [ ] Not Started | [ ] In Progress | [ ] Blocked | [ ] Complete

**Started**: YYYY-MM-DD
**Completed**: YYYY-MM-DD

**Assignee**: [Developer Name]

**Acceptance Criteria**:
- [ ] Criterion 1
- [ ] Criterion 2
- [ ] Criterion 3

**Blockers**: None | [Description of blocker]

**Notes**: [Any relevant notes or decisions made during implementation]

**Testing**: [ ] Unit Tests Passing | [ ] Integration Tests Passing

**Code Review**: [ ] Pending | [ ] Approved

**Documentation**: [ ] Updated
```

---

## Next Steps

1. **Team Assignment**: Assign tasks to developers based on expertise
2. **Sprint Planning**: Organize tasks into 2-week sprints
3. **Phase 0 Kickoff**: Begin research phase immediately
4. **Daily Standups**: Track progress and blockers
5. **Code Reviews**: Ensure quality and knowledge sharing
6. **Continuous Integration**: Set up CI/CD pipeline for automated testing

**Recommended First Sprint** (2 weeks):
- Complete all Phase 0 tasks (P0-T1 through P0-T6)
- Begin Phase 1 tasks (P1-T1 through P1-T7)

**Status**: ✅ Tasks document complete and ready for development team
