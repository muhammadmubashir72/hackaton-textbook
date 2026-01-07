# Feature Specification: Better Auth Authentication System with Neon DB Storage

**Feature Branch**: `003-better-auth-neon-db`
**Created**: 2025-12-24
**Status**: Draft
**Input**: User description: "Refine and specify the behavior, inputs, outputs, triggers, and boundaries of existing skills and sub-agents for Better Auth signup/signin, JWT-based authentication, user background data collection, personalization readiness, and persistent storage using Neon DB"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration and Profile Setup (Priority: P1)

A new user visits the Physical AI Textbook platform and wants to create an account to access personalized learning content based on their technical background.

**Why this priority**: This is the foundation of the entire system. Without user registration, no other features can function. It represents the primary entry point for all users and must work flawlessly to capture both authentication credentials and personalization data.

**Independent Test**: Can be fully tested by completing a signup form with email/password, receiving authentication tokens, and being prompted to complete profile information. Delivers immediate value by creating a functional authenticated user account.

**Acceptance Scenarios**:

1. **Given** a visitor is on the signup page, **When** they enter valid email, password (min 8 chars with uppercase, lowercase, number), name, and confirmation password, **Then** system creates account, returns JWT with profileComplete:false, and redirects to profile completion page
2. **Given** a visitor attempts signup with existing email, **When** they submit the form, **Then** system returns 409 Conflict error with message "Email already registered"
3. **Given** a visitor enters mismatched passwords, **When** they submit the form, **Then** system returns 400 Bad Request error with message "Passwords do not match"
4. **Given** a new user completes account creation, **When** they are redirected to profile completion, **Then** they see forms for software background (languages, frameworks, experience level) and hardware background (platforms, robotics experience, electronics knowledge)
5. **Given** a new user completes their software and hardware background, **When** they submit the profile data, **Then** system validates data, marks profile as complete, issues new JWT with updated claims, and grants access to personalized content

---

### User Story 2 - Existing User Authentication (Priority: P1)

A returning user needs to sign in to access their personalized learning content and continue their progress.

**Why this priority**: Equal priority to registration as it's the primary workflow for returning users. Without reliable signin, existing users cannot access the platform, making it a critical path.

**Independent Test**: Can be fully tested by entering valid credentials and receiving JWT tokens with complete user profile data embedded in claims. Delivers immediate value by authenticating users and providing access to their personalized experience.

**Acceptance Scenarios**:

1. **Given** a registered user with completed profile visits signin page, **When** they enter correct email and password, **Then** system authenticates user, retrieves profile data, returns JWT with profileComplete:true and background claims, and redirects to personalized dashboard
2. **Given** a registered user with incomplete profile signs in, **When** authentication succeeds, **Then** system returns JWT with profileComplete:false and redirects to profile completion page
3. **Given** a user enters invalid credentials, **When** they submit signin form, **Then** system returns 401 Unauthorized error with message "Invalid email or password"
4. **Given** an authenticated user's JWT expires after 15 minutes, **When** they make a request, **Then** system returns 401 and requires user to use refresh token or re-authenticate

---

### User Story 3 - Profile Data Management (Priority: P2)

An authenticated user wants to update their software and hardware background information to receive more accurate content recommendations.

**Why this priority**: Important for maintaining accurate personalization but not blocking for initial access. Users can update their profile later as their skills evolve.

**Independent Test**: Can be fully tested by authenticated user updating their profile data and verifying new JWT contains updated claims. Delivers value by keeping personalization context current.

**Acceptance Scenarios**:

1. **Given** an authenticated user with completed profile, **When** they update programming languages or frameworks, **Then** system validates new data, updates profile, emits profile update event, invalidates cached personalization context, and returns updated profile
2. **Given** an authenticated user updates hardware platforms, **When** they add new platforms or tools, **Then** system updates hardware background, triggers JWT refresh to include new claims in future requests
3. **Given** a user accesses their own profile data, **When** they request profile retrieval, **Then** system returns complete profile including software and hardware backgrounds
4. **Given** a non-admin user attempts to access another user's profile, **When** they make the request, **Then** system returns 403 Forbidden with message "Cannot access other user profiles"

---

### User Story 4 - Secure Access to Personalized Content (Priority: P2)

An authenticated user with complete profile wants to access content personalized to their skill level and interests without re-authenticating frequently.

**Why this priority**: Core value proposition of the system but depends on successful authentication and profile completion (P1 items). Enables the personalization features that differentiate this platform.

**Independent Test**: Can be fully tested by making requests to personalized endpoints with valid JWT and verifying access is granted based on token claims. Delivers value by providing seamless access to personalized content.

**Acceptance Scenarios**:

1. **Given** an authenticated user with complete profile, **When** they request personalized content, **Then** authorization guard validates JWT, checks profileComplete claim is true, and grants access
2. **Given** an authenticated user with incomplete profile, **When** they attempt to access personalized content, **Then** system returns 403 with message "Please complete your profile" and redirect to profile completion page
3. **Given** a user's JWT is about to expire, **When** they use their refresh token, **Then** system validates refresh token, retrieves current profile data, generates new JWT with updated claims, and returns new token pair
4. **Given** an authenticated user, **When** they access multiple protected endpoints, **Then** each request validates JWT signature, checks expiration, verifies not blacklisted, and enforces resource-specific authorization policies

---

### User Story 5 - Session Termination (Priority: P3)

An authenticated user wants to securely sign out from the platform, especially when using a shared or public computer.

**Why this priority**: Important for security but less critical than core authentication flows. Users can still be secure with JWT expiration even without explicit signout.

**Independent Test**: Can be fully tested by signing out and verifying JWT and refresh tokens are revoked. Delivers value by ensuring tokens cannot be reused after signout.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they click signout, **Then** system validates current JWT, adds it to blacklist, invalidates refresh token, clears Better Auth session, and returns success
2. **Given** a user has signed out, **When** they attempt to use the revoked JWT, **Then** system returns 401 Unauthorized because token is in blacklist
3. **Given** a user has signed out, **When** they attempt to use the refresh token, **Then** system returns 401 because refresh token has been revoked

---

### Edge Cases

- **What happens when user attempts to complete profile with invalid enum values?**
  System validates experience level, robotics experience, and electronics knowledge against allowed enums and returns 400 Bad Request with specific field errors if invalid values provided.

- **How does system handle concurrent profile updates by same user?**
  Last write wins. System uses database timestamps to ensure most recent update is persisted. Cached personalization context is invalidated and rebuilt on next request.

- **What happens if JWT secret key is rotated?**
  All existing JWTs become invalid. System returns 401 for all requests with old tokens. Users must re-authenticate to receive new tokens signed with new secret.

- **How does system handle refresh token that has been used (token rotation enabled)?**
  If refresh token rotation is enabled and a used refresh token is presented, system returns 401 and may trigger security alert for potential token theft.

- **What happens when Better Auth service is unavailable during signup/signin?**
  System returns 500 Internal Server Error. Authentication operations fail gracefully with retry message. Existing authenticated users with valid JWTs can continue accessing protected resources.

- **How does system handle user attempting to register with email from different case (Email@example.com vs email@example.com)?**
  System normalizes all emails to lowercase before uniqueness check and storage to prevent duplicate accounts with case variations.

- **What happens when JWT blacklist/Redis cache is unavailable?**
  System fails closed for security - returns 503 Service Unavailable for authentication operations rather than allowing potentially revoked tokens. Alerts are triggered for infrastructure team.

- **How does system handle profile data exceeding reasonable limits (e.g., 1000 programming languages)?**
  System enforces reasonable array limits (e.g., max 20 languages, 20 frameworks, 10 platforms) and returns 400 Bad Request if exceeded, preventing storage bloat and performance issues.

## Requirements *(mandatory)*

### Functional Requirements

**Authentication & Authorization:**

- **FR-001**: System MUST allow new users to create accounts using email and password via Better Auth integration
- **FR-002**: System MUST validate password strength (minimum 8 characters, at least one uppercase letter, one lowercase letter, one number)
- **FR-003**: System MUST check email uniqueness before account creation and return 409 Conflict if email already exists
- **FR-004**: System MUST authenticate existing users via Better Auth credential verification
- **FR-005**: System MUST issue JWT tokens with 15-minute expiration upon successful authentication
- **FR-006**: System MUST issue refresh tokens with 7-day expiration for token renewal without re-authentication
- **FR-007**: System MUST include user profile data (profileComplete status, background data) as claims in JWT payload
- **FR-008**: System MUST validate JWT signature, expiration, and blacklist status on every protected endpoint request
- **FR-009**: System MUST allow users to refresh expired JWTs using valid refresh tokens
- **FR-010**: System MUST revoke JWT and refresh tokens on user signout by adding to blacklist
- **FR-011**: System MUST enforce authorization policies: authenticated access, self-profile access, admin access, and personalized content access

**Profile & Background Collection:**

- **FR-012**: System MUST collect software background data including programming languages (array), frameworks (array), experience level (enum: beginner/intermediate/advanced/expert), specializations (array), and years of experience (number)
- **FR-013**: System MUST collect hardware background data including familiar platforms (array), robotics experience (enum: none/hobbyist/professional), electronics knowledge (enum: none/basic/intermediate/advanced), and preferred tools (array)
- **FR-014**: System MUST validate all profile data against defined schemas before storage
- **FR-015**: System MUST initialize empty profile records for new users with profileComplete set to false
- **FR-016**: System MUST mark profile as complete when user provides minimum required data: at least 1 programming language, experience level, at least 1 familiar platform, robotics experience, and electronics knowledge
- **FR-017**: System MUST allow authenticated users to update their own profile and background data
- **FR-018**: System MUST prevent non-admin users from accessing or modifying other users' profiles
- **FR-019**: System MUST trigger JWT refresh to include updated claims when profile data changes
- **FR-020**: System MUST emit profile update events for cache invalidation when background data changes

**Data Storage & Caching:**

- **FR-021**: System MUST persist user credentials, profile data, software backgrounds, and hardware backgrounds in Neon DB (PostgreSQL-compatible) with proper foreign key relationships and cascade delete constraints
- **FR-022**: System MUST store refresh tokens in Redis cache with automatic TTL expiration (7 days) for performance, with fallback to Neon DB token_sessions table for durability
- **FR-023**: System MUST maintain JWT blacklist in Redis cache with TTL matching original token expiration, with Neon DB revoked_tokens table for persistence across Redis failures
- **FR-024**: System MUST cache complete user profiles (including backgrounds) in Redis with 1-hour TTL for performance optimization
- **FR-025**: System MUST invalidate profile cache immediately upon profile updates
- **FR-026-NeonDB**: System MUST use Neon DB connection pooling with minimum 2 and maximum 10 concurrent connections for optimal serverless performance
- **FR-027-NeonDB**: System MUST leverage Neon DB's branching capability for database schema migrations and testing without affecting production data
- **FR-028-NeonDB**: System MUST implement automatic retry logic with exponential backoff for transient Neon DB connection failures
- **FR-029-NeonDB**: System MUST log all database operations (create, update, delete) to Neon DB audit_logs table with timestamp, userId, operation type, and affected tables

**Personalization Context:**

- **FR-030**: System MUST build personalization context object from user profile data including expertise analysis, learning paths, content preferences, and UI adaptations
- **FR-031**: System MUST cache personalization context in Redis with 1-hour TTL
- **FR-032**: System MUST invalidate personalization context cache when profile or background data changes
- **FR-033**: System MUST restrict access to personalized content endpoints to users with profileComplete:true in JWT claims
- **FR-034-NeonDB**: System MUST store personalization preferences (difficulty settings, focus areas) in Neon DB user_preferences table for persistence across sessions

**Security & Audit:**

- **FR-035**: System MUST log all authentication attempts (successful and failed) to Neon DB auth_logs table with timestamp, userId, IP address, user agent, and success/failure status
- **FR-036**: System MUST log all authorization attempts to Neon DB authz_logs table with timestamp, userId, requested resource, policy evaluated, and result
- **FR-037**: System MUST normalize email addresses to lowercase for consistency and duplicate prevention
- **FR-038**: System MUST enforce rate limiting on authentication endpoints to prevent brute force attacks
- **FR-039**: System MUST use HS256 algorithm for JWT signing with securely stored secret keys
- **FR-040-NeonDB**: System MUST encrypt sensitive profile data (name, email) at application layer before storing in Neon DB using AES-256 encryption
- **FR-041-NeonDB**: System MUST leverage Neon DB's automatic backup capability with point-in-time recovery (PITR) for data protection
- **FR-042-NeonDB**: System MUST implement database-level row-level security (RLS) policies in Neon DB to prevent cross-user data access

### Key Entities

**User:**
- Represents a registered account
- Attributes: id (UUID), email (unique), name, passwordHash, createdAt, updatedAt
- Relationships: Has one Profile, authenticated via Better Auth

**Profile:**
- Represents user's completion status and metadata
- Attributes: id (UUID), userId (foreign key), profileComplete (boolean), createdAt, updatedAt
- Relationships: Belongs to one User, has one SoftwareBackground, has one HardwareBackground

**SoftwareBackground:**
- Represents user's software development expertise
- Attributes: id (UUID), profileId (foreign key), programmingLanguages (array), frameworks (array), experienceLevel (enum), specializations (array), yearsOfExperience (integer), createdAt, updatedAt
- Relationships: Belongs to one Profile
- Validation: experienceLevel must be one of [beginner, intermediate, advanced, expert], yearsOfExperience range 0-50

**HardwareBackground:**
- Represents user's hardware and robotics expertise
- Attributes: id (UUID), profileId (foreign key), familiarPlatforms (array), roboticsExperience (enum), electronicsKnowledge (enum), preferredTools (array), createdAt, updatedAt
- Relationships: Belongs to one Profile
- Validation: roboticsExperience must be one of [none, hobbyist, professional], electronicsKnowledge must be one of [none, basic, intermediate, advanced]

**JWT Token:**
- Represents authenticated session
- Claims: sub (userId), email, name, profileComplete (boolean), softwareBackground (array summary), hardwareBackground (array summary), roles (array), iat (timestamp), exp (timestamp), iss (issuer)
- Lifecycle: 15-minute expiration, stored in blacklist on revocation

**Refresh Token:**
- Represents long-lived session renewal capability
- Storage: Primary in Redis (token → userId), fallback in Neon DB token_sessions table
- Neon DB Schema: id (UUID), token (hashed, indexed), userId (FK), createdAt, expiresAt, lastUsedAt, isRevoked
- Lifecycle: 7-day expiration, deleted on use if rotation enabled, revoked on signout

**PersonalizationContext:**
- Represents derived user expertise and preferences
- Attributes: userId, expertise (software/hardware analysis), learningPath, contentPreferences, recommendations, uiAdaptations, metadata
- Storage: Primary in Redis cache with 1-hour TTL, preferences persisted to Neon DB user_preferences table
- Neon DB Schema (user_preferences): id (UUID), userId (FK), difficultySetting (VARCHAR), focusAreas (JSONB), preferredPlatforms (JSONB), updatedAt
- Lifecycle: Built on-demand, cached in Redis, invalidated on profile updates

**Neon DB Additional Tables:**

**revoked_tokens** (JWT Blacklist Persistence):
- id (UUID PRIMARY KEY)
- tokenId (VARCHAR UNIQUE, indexed)
- userId (UUID FK to users.id)
- revokedAt (TIMESTAMP)
- expiresAt (TIMESTAMP, indexed for cleanup)
- reason (VARCHAR - 'signout', 'security', 'admin')

**auth_logs** (Authentication Audit):
- id (UUID PRIMARY KEY)
- userId (UUID FK to users.id, nullable)
- email (VARCHAR)
- ipAddress (INET)
- userAgent (TEXT)
- success (BOOLEAN)
- failureReason (VARCHAR, nullable)
- timestamp (TIMESTAMP, indexed)

**authz_logs** (Authorization Audit):
- id (UUID PRIMARY KEY)
- userId (UUID FK to users.id)
- resource (VARCHAR)
- action (VARCHAR)
- policy (VARCHAR)
- authorized (BOOLEAN)
- timestamp (TIMESTAMP, indexed)

**audit_logs** (General System Audit):
- id (UUID PRIMARY KEY)
- userId (UUID FK to users.id, nullable)
- operation (VARCHAR - 'CREATE', 'UPDATE', 'DELETE')
- tableName (VARCHAR)
- recordId (UUID)
- oldValues (JSONB, nullable)
- newValues (JSONB)
- timestamp (TIMESTAMP, indexed)

## Success Criteria *(mandatory)*

### Measurable Outcomes

**User Experience:**

- **SC-001**: New users can complete account registration in under 1 minute
- **SC-002**: Existing users can sign in and access personalized content in under 10 seconds
- **SC-003**: Profile completion form can be filled out in under 3 minutes
- **SC-004**: 95% of authentication requests succeed on first attempt for valid credentials
- **SC-005**: Users can access protected resources without re-authentication for 15 minutes (JWT lifetime)

**System Performance:**

- **SC-006**: System handles 1,000 concurrent authentication requests without degradation
- **SC-007**: JWT validation completes in under 50ms including blacklist check
- **SC-008**: Profile retrieval (with cache hit) completes in under 100ms
- **SC-009**: Profile retrieval (cache miss) completes in under 500ms including database queries
- **SC-010**: Personalization context building completes in under 1 second

**Security & Reliability:**

- **SC-011**: Zero authentication bypasses - all protected endpoints enforce JWT validation
- **SC-012**: Revoked tokens are rejected 100% of the time within 1 second of revocation
- **SC-013**: Rate limiting prevents more than 5 failed login attempts per email per minute
- **SC-014**: All authentication and authorization events are logged with complete audit trail
- **SC-015**: System maintains 99.9% uptime for authentication services

**Data Quality:**

- **SC-016**: 100% of user profiles pass schema validation before storage
- **SC-017**: Profile completion status accurately reflects data completeness (no false positives)
- **SC-018**: Cache invalidation occurs within 1 second of profile updates
- **SC-019**: Zero duplicate accounts created due to race conditions

**Personalization Readiness:**

- **SC-020**: Users with complete profiles have access to personalized content recommendations within 2 seconds
- **SC-021**: Personalization context cache hit rate exceeds 80%
- **SC-022**: Profile updates trigger JWT refresh and cache invalidation in under 500ms
- **SC-023**: Users understand profile completion requirements through clear UI messaging

## Assumptions *(mandatory)*

1. **Better Auth Service Availability**: Better Auth SDK and service will be available and operational. System will fail gracefully with 500 errors if Better Auth is unavailable.

2. **Neon DB Availability**: Neon DB PostgreSQL-compatible database is available with connection pooling configured. System implements retry logic for transient failures. Critical operations have fallback mechanisms.

3. **Redis Infrastructure**: Redis instance is available for caching and token storage with sufficient memory and configured persistence. If Redis is unavailable, system falls back to Neon DB for refresh tokens and blacklist queries with degraded performance.

4. **Email Format**: All emails will use standard format (local@domain). System normalizes to lowercase for consistency.

5. **Password Security**: Users are responsible for password strength beyond minimum requirements. System does not enforce password history or rotation policies.

6. **Single Device Sessions**: Each refresh token is associated with one session. Multiple devices require separate authentication and refresh tokens.

7. **Profile Data Limits**: Users will not attempt to store unreasonably large profile data. System enforces limits (max 20 languages, 20 frameworks, 10 platforms, 10 tools) to prevent abuse.

8. **Token Rotation Strategy**: Refresh token rotation is optional and configurable. Default implementation does not rotate to simplify client handling.

9. **Admin Role Assignment**: Admin role assignment is manual/external to this system. No self-service admin elevation is provided.

10. **Profile Completion UX**: Frontend will guide users through profile completion with clear requirements and validation feedback.

11. **Personalization Algorithm**: Content recommendation algorithm is provided by external content service. This system only provides personalization context data.

12. **Audit Log Retention**: Audit logs in Neon DB will be retained per organizational security policy. System writes logs but does not implement automatic cleanup or archival.

13. **JWT Secret Rotation**: JWT secret rotation is a manual administrative operation requiring all users to re-authenticate. Not automated.

14. **Neon DB Connection Pooling**: Neon DB connection pool is properly configured (min 2, max 10 connections) to handle serverless cold starts and connection reuse efficiently.

15. **Neon DB Schema Migrations**: Database schema changes will be managed through migration scripts that leverage Neon DB branching for zero-downtime deployments.

16. **Encryption Key Management**: AES-256 encryption keys for sensitive data are securely stored in environment variables or secret management service and rotated according to security policy.

## Out of Scope *(mandatory)*

1. **Social Authentication**: OAuth/SSO providers (Google, GitHub, etc.) are not included. Only email/password authentication via Better Auth.

2. **Multi-Factor Authentication (MFA)**: Two-factor authentication, SMS codes, authenticator apps are not implemented.

3. **Password Reset Flow**: Forgot password, reset password, email verification for password reset are external to this specification.

4. **Email Verification**: Email address verification via confirmation links is not included in account creation flow.

5. **Account Deletion**: User-initiated account deletion and GDPR right-to-be-forgotten workflows are not covered.

6. **Session Management UI**: User-facing interface to view/revoke active sessions across multiple devices is not included.

7. **Role-Based Access Control (RBAC) Management**: Admin interfaces to create roles, assign permissions, manage user roles are not implemented.

8. **Content Recommendation Engine**: Algorithm that generates personalized content recommendations based on personalization context is external.

9. **Analytics Dashboard**: Visualization and reporting of authentication metrics, user growth, profile completion rates are not included.

10. **Password Strength Meter**: Real-time password strength visualization during signup is a frontend concern, not covered here.

11. **Account Lockout**: Automatic account disabling after repeated failed login attempts is not implemented (rate limiting only).

12. **API Rate Limiting Configuration**: Admin interface to configure rate limits per user/IP is not included.

13. **Webhook Notifications**: External system notifications on authentication events (new signup, signin, etc.) are not implemented.

14. **User Impersonation**: Admin capability to impersonate users for support purposes is not included.

15. **Profile Export**: User ability to export their profile and background data is not covered.

## Dependencies *(mandatory)*

### External Dependencies:

1. **Better Auth SDK**: Authentication service integration for user creation and credential verification
2. **Neon DB (PostgreSQL-compatible)**: Primary persistent storage for users, profiles, software_backgrounds, hardware_backgrounds, token_sessions, revoked_tokens, user_preferences, auth_logs, authz_logs, audit_logs
3. **Redis**: Cache layer for refresh tokens, JWT blacklist, profile cache, personalization context cache (with Neon DB fallback)
4. **JWT Library**: jsonwebtoken or jose library for token generation, signing, and validation
5. **Validation Library**: Zod or Joi for schema validation of profile data
6. **Neon DB Node.js Driver**: @neondatabase/serverless for connection pooling and serverless optimization
7. **Encryption Library**: crypto (Node.js native) or bcrypt for AES-256 encryption of sensitive data

### Internal Dependencies:

1. **Frontend Application**: Must implement signup/signin forms, profile completion forms, token storage, and Authorization header management
2. **Content Service**: Must consume personalization context to provide tailored content recommendations
3. **Event Bus** (optional): For publishing profile update events if real-time synchronization needed
4. **Secret Management Service**: For securely storing JWT_SECRET, BETTER_AUTH_SECRET, and REDIS credentials

### Infrastructure Dependencies:

1. **Environment Variables**: JWT_SECRET, JWT_EXPIRY, REFRESH_TOKEN_EXPIRY, BETTER_AUTH_SECRET, REDIS_URL, NEON_DATABASE_URL, NEON_API_KEY (for branching), ENCRYPTION_KEY
2. **Network Access**: Backend must reach Better Auth service, Redis, and Neon DB (typically via TLS/SSL connection)
3. **HTTPS/TLS**: All authentication endpoints must be served over HTTPS in production
4. **Neon DB Project**: Active Neon DB project with appropriate compute resources and storage limits configured
5. **Connection Pooling**: Configured connection pool manager (e.g., @neondatabase/serverless with neonConfig.poolQueryViaFetch = true for serverless environments)

## Refined Skills & Sub-Agents Specifications

### Skill Boundaries and Responsibilities

**Skills** represent discrete, testable capabilities triggered by specific events. They define WHAT the system does.

**Sub-Agents** represent persistent services that orchestrate skills and manage state. They define HOW the system coordinates behaviors.

### Overlap Resolution:

1. **JWT Management Skill vs JWT Service Agent**:
   - **Skill (jwt-management.skill.md)**: Defines the three operations (generate, validate, refresh) as discrete, testable behaviors with clear inputs/outputs
   - **Agent (jwt-service.agent.md)**: Implements the skill operations, manages Redis connections, handles secret management, maintains token blacklist
   - **Resolution**: Skill is the contract/interface, Agent is the implementation. No overlap.

2. **Profile Collection Skill vs Profile Manager Agent**:
   - **Skill (profile-collection.skill.md)**: Defines the data collection workflow, schemas, and validation rules
   - **Agent (profile-manager.agent.md)**: Implements CRUD operations, database interactions, cache management, completion criteria checking
   - **Resolution**: Skill defines WHAT data is collected and validated, Agent defines HOW it's stored and retrieved. Clear separation.

3. **Access Control Skill vs Authorization Guard Agent**:
   - **Skill (access-control.skill.md)**: Defines authorization policies, validation rules, and decision-making logic
   - **Agent (authorization-guard.agent.md)**: Implements middleware integration, request interception, policy evaluation engine
   - **Resolution**: Skill is the policy definition, Agent is the enforcement mechanism. No duplication.

4. **Auth Signup/Signin Skills vs Auth Orchestration Agent**:
   - **Skills (auth-signup.skill.md, auth-signin.skill.md)**: Define discrete registration and authentication workflows with specific inputs/outputs
   - **Agent (auth-orchestration.agent.md)**: Coordinates between Better Auth, JWT Service, and Profile Manager to execute the workflows
   - **Resolution**: Skills are atomic operations, Agent orchestrates multi-step flows. Complementary.

### Data Flow Specification with Neon DB Integration:

```
[User Signup Request]
  → auth-signup.skill (validation)
  → auth-orchestration.agent (coordinates)
  → Better Auth SDK (creates user)
  → profile-manager.agent (initializes profile)
    → Neon DB: INSERT into users, profiles, software_backgrounds, hardware_backgrounds tables
    → Neon DB: INSERT into audit_logs (operation: CREATE)
  → jwt-service.agent (generates JWT with profileComplete:false)
    → Redis: SET refresh_token (with 7d TTL)
    → Neon DB: INSERT into token_sessions (fallback/persistence)
  → [Response: tokens + redirect to profile completion]

[Profile Completion]
  → profile-collection.skill (validates background data)
  → profile-manager.agent (stores + marks complete)
    → Neon DB: UPDATE software_backgrounds, hardware_backgrounds
    → Neon DB: UPDATE profiles SET profileComplete = true
    → Neon DB: INSERT into audit_logs (operation: UPDATE)
    → Redis: DEL profile:{userId} (invalidate cache)
  → jwt-service.agent (refreshes JWT with updated claims)
  → personalization-context.agent (builds context)
    → Neon DB: INSERT/UPDATE user_preferences
    → Redis: SET personalization_context:{userId} (with 1h TTL)
  → [Response: updated profile + tokens]

[User Signin Request]
  → auth-signin.skill (validates credentials)
  → auth-orchestration.agent (coordinates)
  → Better Auth SDK (verifies credentials)
  → Neon DB: INSERT into auth_logs (success/failure)
  → profile-manager.agent (retrieves profile)
    → Redis: GET profile:{userId} (cache hit?)
    → [If cache miss] → Neon DB: SELECT users JOIN profiles JOIN software_backgrounds JOIN hardware_backgrounds
    → [If retrieved] → Redis: SET profile:{userId} (cache for 1h)
  → jwt-service.agent (generates JWT with profile claims)
    → Redis: SET refresh_token
    → Neon DB: INSERT into token_sessions
  → [Response: tokens + profile data]

[Protected Resource Access]
  → authorization-guard.agent (middleware intercepts)
  → access-control.skill (validates JWT + policies)
  → jwt-service.agent (validates signature + expiration + blacklist)
    → Redis: GET blacklist:{tokenId} (revoked?)
    → [If Redis miss] → Neon DB: SELECT from revoked_tokens WHERE tokenId
  → Neon DB: INSERT into authz_logs (authorized/denied)
  → [If authorized] → Resource Handler
  → [If unauthorized] → 401/403 error response

[Token Refresh]
  → jwt-management.skill (refresh operation)
  → jwt-service.agent (validates refresh token)
    → Redis: GET refresh_token:{token}
    → [If Redis miss] → Neon DB: SELECT from token_sessions WHERE token AND NOT isRevoked
    → Neon DB: UPDATE token_sessions SET lastUsedAt
  → profile-manager.agent (retrieves current profile)
    → Redis/Neon DB: Profile retrieval (as in signin flow)
  → jwt-service.agent (generates new JWT)
    → Redis: SET new refresh_token
    → Neon DB: INSERT/UPDATE token_sessions
  → personalization-context.agent (retrieves cached context)
    → Redis: GET personalization_context:{userId}
    → [If miss] → Neon DB: SELECT from user_preferences + build context
  → [Response: new token pair]

[User Signout]
  → auth-orchestration.agent (revoke flow)
  → jwt-service.agent (revoke JWT)
    → Redis: SET blacklist:{tokenId} (with TTL = token expiry)
    → Neon DB: INSERT into revoked_tokens
    → Redis: DEL refresh_token:{token}
    → Neon DB: UPDATE token_sessions SET isRevoked = true
  → [Response: success]

[Neon DB Cleanup Jobs (Background)]
  → Periodic cleanup of expired tokens:
    → DELETE from revoked_tokens WHERE expiresAt < NOW()
    → DELETE from token_sessions WHERE expiresAt < NOW() AND isRevoked = true
  → Log retention policy:
    → Archive/delete old auth_logs, authz_logs, audit_logs per retention policy
```

## Skills & Sub-Agents Integration Matrix with Neon DB

| Skill | Primary Agent | Supporting Agents | Data Flow Direction | Neon DB Tables Used |
|-------|---------------|-------------------|---------------------|---------------------|
| auth-signup | auth-orchestration | jwt-service, profile-manager | User → Auth → Neon DB → JWT + Profile | users, profiles, software_backgrounds, hardware_backgrounds, audit_logs, token_sessions |
| auth-signin | auth-orchestration | jwt-service, profile-manager | User → Auth → Neon DB → JWT + Profile | users, profiles, software_backgrounds, hardware_backgrounds, auth_logs, token_sessions |
| jwt-management | jwt-service | (standalone) | Request → Validate/Generate → Neon DB → Response | token_sessions, revoked_tokens |
| profile-collection | profile-manager | jwt-service, personalization-context | User → Validate → Neon DB → JWT Refresh | profiles, software_backgrounds, hardware_backgrounds, user_preferences, audit_logs |
| profile-retrieval | profile-manager | authorization-guard | Request → Authorize → Neon DB → Response | users, profiles, software_backgrounds, hardware_backgrounds |
| access-control | authorization-guard | jwt-service | Request → Validate → Neon DB → Authorize → Allow/Deny | revoked_tokens, authz_logs |

## Consistency Requirements with Neon DB

1. **JWT Claims Consistency**: All agents must use identical claim structure defined in jwt-service.agent.md (sub, email, profileComplete, backgrounds, roles, iat, exp, iss)

2. **Error Response Format**: All agents must return consistent error format: `{ error: string, message: string, code: string }`

3. **Profile Completion Criteria**: auth-orchestration, profile-manager, and personalization-context must use identical completion check (1+ language, experience level, 1+ platform, robotics exp, electronics knowledge)

4. **Cache Key Naming**: Standardize cache keys across agents:
   - Profile: `profile:{userId}`
   - Refresh Token: `refresh_token:{token}`
   - JWT Blacklist: `blacklist:{tokenId}`
   - Personalization: `personalization_context:{userId}`

5. **Event Naming**: Standardize event names for profile updates:
   - Profile Created: `profile.created`
   - Profile Updated: `profile.updated`
   - Profile Completed: `profile.completed`

6. **Timestamp Format**: All timestamps must use ISO 8601 format (YYYY-MM-DDTHH:mm:ss.sssZ) in application code, stored as PostgreSQL TIMESTAMP in Neon DB

7. **Authorization Policy Names**: Exact naming across access-control skill and authorization-guard agent: `authenticated`, `self_profile`, `admin`, `personalized_content`

8. **Neon DB Connection Handling**: All agents must:
   - Use connection pooling (@neondatabase/serverless)
   - Implement retry logic with exponential backoff (max 3 retries)
   - Handle connection timeouts gracefully (default 10s timeout)
   - Close connections properly in error scenarios

9. **Neon DB Transaction Management**: All multi-table operations must use database transactions with ROLLBACK on error:
   - User signup: INSERT users + profiles + backgrounds (atomic)
   - Profile update: UPDATE profiles + backgrounds + user_preferences (atomic)
   - Token revocation: UPDATE token_sessions + INSERT revoked_tokens (atomic)

10. **Neon DB Query Optimization**: All agents must:
    - Use prepared statements to prevent SQL injection
    - Create indexes on foreign keys and frequently queried columns
    - Use JOINs efficiently (users ⟕ profiles ⟕ backgrounds in single query)
    - Implement query result pagination for large datasets

11. **Redis-Neon DB Fallback Strategy**: When Redis is unavailable:
    - Check Neon DB for refresh tokens (token_sessions table)
    - Query Neon DB for revoked tokens (revoked_tokens table)
    - Degrade gracefully with warning logs
    - Performance degrades but system remains functional

12. **Audit Trail Consistency**: All database mutations must write to audit_logs table with:
    - userId (who performed action)
    - operation (CREATE/UPDATE/DELETE)
    - tableName (which table affected)
    - oldValues and newValues (JSONB for diff)
    - timestamp (when action occurred)
