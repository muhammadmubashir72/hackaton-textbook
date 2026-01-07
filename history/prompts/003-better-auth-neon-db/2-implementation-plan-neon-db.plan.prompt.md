---
id: 2
title: Implementation Plan Neon DB Integration
stage: plan
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5
feature: 003-better-auth-neon-db
branch: 003-better-auth-neon-db
user: developer
command: /sp.plan
labels: ["authentication", "neon-db", "better-auth", "jwt", "implementation-plan", "architecture"]
links:
  spec: specs/003-better-auth-refinement/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-better-auth-refinement/plan.md
tests:
  - Phase breakdown validation
  - Dependency chain validation
  - Timeline estimation
---

## Prompt

mode. Based strictly on the already refined skills and sub-agents defined in .claude/skills/ and .claude/sub_agents/ specs, PHR, create a clear, step-by-step implementation plan. The plan must cover Better Auth signup/signin, JWT-based authentication, user software and hardware background collection, persistent storage using Neon DB, and personalization readiness. Do not introduce new skills, agents, or features. Focus on sequencing, dependencies, data flow, and integration order between authentication, JWT handling, Neon DB persistence, and content personalization hooks. Output the plan in concise, technical Markdown suitable for direct execution.

## Response snapshot

Created comprehensive 5-phase implementation plan for Better Auth with Neon DB integration:

**Plan Document**: `specs/003-better-auth-refinement/plan.md`

**Architecture Overview**:
- Frontend (Docusaurus) → FastAPI Backend → Auth Orchestration Agent
- JWT Service + Profile Manager + Authorization Guard agents
- Dual storage: Redis (cache) + Neon DB (persistence)
- Better Auth for credential management

**Phase Breakdown**:

**Phase 0: Research & Technical Validation** (2-3 days)
- Neon DB setup and configuration (connection pooling, migrations, branching)
- Better Auth integration patterns (email/password provider, callbacks)
- JWT security and token management (secret rotation, blacklist)
- Redis-Neon DB fallback strategy (graceful degradation)
- Output: `research.md` with all technical decisions documented

**Phase 1: Data Layer & Schema Implementation** (4-5 days)
- Neon DB project initialization with connection pooling (min 2, max 10)
- 10-table schema creation (users, profiles, backgrounds, preferences, tokens, logs)
- Index creation for performance (FK, timestamps, query columns)
- Row-level security (RLS) policies for cross-user protection
- Data Access Layer (DAL) with repository pattern
- Audit logging for all mutations
- Dependencies: Phase 0 complete

**Phase 2: Authentication Core** (5-6 days)
- Better Auth configuration (email/password provider, callbacks)
- JWT service implementation (generation with custom claims, validation)
- Redis cache layer with fallback to Neon DB
- Auth orchestration agent (signup, signin, refresh, signout flows)
- API endpoints: `/auth/signup`, `/auth/signin`, `/auth/refresh`, `/auth/signout`
- Dependencies: Phase 1 complete

**Phase 3: Profile Management & Background Collection** (3-4 days)
- Profile collection skill with Zod validation
- Software/hardware background schemas
- Profile update endpoint with transaction management
- Authorization guard middleware (policy-based: authenticated, self_profile, admin, personalized_content)
- Audit logging for profile mutations
- API endpoints: `/profile/:userId` (GET, PUT)
- Dependencies: Phase 2 complete

**Phase 4: Personalization Context & Integration** (4-5 days)
- Personalization context agent (expertise mapping, learning path generation)
- User preferences persistence to Neon DB
- Redis caching for personalization context (1-hour TTL)
- Content service integration (recommendations based on expertise)
- Event handlers for profile updates
- API endpoints: `/personalization/context`, `/personalization/refresh`
- Dependencies: Phase 3 complete

**Phase 5: Testing, Documentation & Deployment** (4-5 days)
- Unit tests (repositories, services, agents)
- Integration tests (end-to-end flows)
- API documentation (OpenAPI spec)
- Performance optimization (indexes, query tuning)
- Deployment configuration (environment variables, production setup)
- Dependencies: Phases 1-4 complete

**Database Schema** (10 tables):
- Core: users, profiles, software_backgrounds, hardware_backgrounds, user_preferences
- Tokens: token_sessions, revoked_tokens
- Audit: auth_logs, authz_logs, audit_logs

**Key Data Flows Documented**:
1. Signup → Profile Initialization: Better Auth creates user → Callback creates profile → JWT issued (profileComplete: false)
2. Profile Completion → Personalization: Update backgrounds → Mark complete → Build context → Store preferences → Cache context
3. Signin → Cached Profile Retrieval: Validate credentials → Check Redis cache → Fallback to Neon DB JOINs → Generate JWT
4. Protected Resource Access: Extract JWT → Validate → Check blacklist → Verify claims → Log authz → Grant/deny

**API Contract Summary**: 8 endpoints documented with request/response schemas, error codes, and authorization requirements

**Constitution Alignment**: ✅ PASS
- VII. Authentication and Personalization: Implements better-auth with background questions
- II. AI-Native Educational Experience: Foundation for personalized learning
- V. Advanced Technology Integration: Uses specified stack (better-auth + Neon Postgres)

**Timeline**: 22-28 days (sequential phases)
**Critical Path**: Phase 0 → 1 → 2 → 3 → 4 → 5

**Risk Mitigation Documented**:
- Neon DB failures: Retry logic, Redis fallback
- Redis unavailability: Fallback to Neon DB with degraded performance
- JWT compromise: Secret rotation procedure
- Performance: Caching, connection pooling, query optimization

**Success Metrics Defined**:
- Performance: Signup < 1 min, signin < 10s, JWT validation < 50ms
- Reliability: 99.9% uptime, zero auth bypasses, 100% revoked token rejection
- Security: Password enforcement, rate limiting, AES-256 encryption, RLS policies

## Outcome

- ✅ Impact: Complete implementation plan ready for development with clear sequencing and dependencies
- 🧪 Tests: All phase dependencies validated, timeline estimated, success metrics defined
- 📁 Files: 1 file created (plan.md with 5 phases, 10 database tables, 8 API endpoints)
- 🔁 Next prompts: `/sp.tasks` to generate granular task breakdown for Phase 0 and Phase 1
- 🧠 Reflection: Plan maintains strict adherence to existing skills/sub-agents without introducing new components. Focus on integration, sequencing, and data flow ensures smooth implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan follows specification exactly
- Graders run and results (PASS/FAIL): Constitutional alignment check: PASS
- Prompt variant (if applicable): Standard implementation planning with database integration focus
- Next experiment (smallest change to try): Consider parallel execution of Phase 5 testing alongside Phase 3/4 development to accelerate timeline
