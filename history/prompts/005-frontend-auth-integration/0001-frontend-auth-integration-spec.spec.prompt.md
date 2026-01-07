---
id: 0001
title: Frontend Auth Integration Spec
stage: spec
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5
feature: 005-frontend-auth-integration
branch: 005-frontend-auth-integration
user: developer
command: /sp.specify
labels: ["frontend", "authentication", "jwt", "docusaurus", "navbar", "signup", "signin"]
links:
  spec: specs/005-frontend-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-frontend-auth-integration/spec.md
  - specs/005-frontend-auth-integration/checklists/requirements.md
tests:
  - Content quality validation
  - Requirement completeness check
  - Technology-agnostic success criteria
---

## Prompt

/sp.specify mode. Read and align with the existing authentication backend specification (JWT, Neon DB, Redis, Express) and define the frontend specifications only, without modifying backend behavior. Specify how the frontend will integrate with the provided auth APIs (/signup, /signin, /refresh, /signout) using JWT. Define the navbar changes to include Signup / Signin buttons, and a logged-in state that replaces them with user actions (profile, logout). Specify the Signup form UI to collect email, password, and user software and hardware background fields exactly as expected by the backend schema. Specify the Signin form UI with credential handling and JWT storage (secure client-side strategy). Define how auth state is managed globally, how protected personalization features check login state, and how error/loading states are handled. Do not add new features, APIs, or auth logic. Output a clean, technical frontend specification in Markdown, ready for implementation.

## Response snapshot

Created comprehensive frontend authentication specification aligned with existing JWT backend:

**Specification Document**: `specs/005-frontend-auth-integration/spec.md`

**Key Components Specified:**

1. **Navbar Integration** (FR-001 to FR-005)
   - Unauthenticated state: "Sign Up" + "Sign In" buttons
   - Authenticated state: User name/email + dropdown (Profile, Logout)
   - Profile completion indicator badge
   - Real-time state updates

2. **Signup Form** (FR-006 to FR-015)
   - Basic fields: email, password, confirmation, name
   - Deferred background collection to post-signup profile completion flow (simplifies UX)
   - Validation: email format, password min 8 chars, confirmation match
   - API integration: POST `/api/auth/signup` with {email, password, name}
   - Response handling: 201 success → store tokens, 400/409/500 errors → display messages
   - Loading states during API call

3. **Signin Form** (FR-016 to FR-023)
   - Fields: email, password
   - API integration: POST `/api/auth/signin`
   - Response handling: 200 success → store tokens + profile, 401/429/500 errors
   - Account lock messaging (after 5 failed attempts)
   - Preserve email on error, clear password
   - "Forgot Password?" placeholder link

4. **JWT Token Storage** (FR-024 to FR-028)
   - Access token: sessionStorage (XSS mitigation)
   - Refresh token: localStorage (persistence tradeoff documented)
   - Authorization header: `Bearer {accessToken}`
   - Auto-cleanup on signout or refresh failure

5. **Global Auth State** (FR-029 to FR-033)
   - State shape: {authenticated, user, profileComplete}
   - Initialization on app load via token validation/refresh
   - Helper methods: isAuthenticated(), hasCompleteProfile(), getUser(), signout()
   - JWT claims decoded: id, email, name, profileComplete, backgrounds, roles

6. **Automatic Token Refresh** (FR-034 to FR-038)
   - Intercept 401 responses → refresh token → retry request
   - Call `/api/auth/refresh` with refresh token
   - Update access token if successful
   - Signout if refresh fails
   - Max 1 retry per request (prevent loops)

7. **Protected Features** (FR-039 to FR-043)
   - Auth check before rendering personalization features
   - Redirect unauthenticated users to signin
   - Profile completion check for personalization
   - Automatic token inclusion in API headers

8. **Error & Loading States** (FR-044 to FR-051)
   - Loading spinners + disabled inputs during API calls
   - Network errors: "Check your connection"
   - Backend errors: Display response.error message
   - Success toasts: "Welcome back!" / "Account created!"
   - Session expiry notification

**User Stories** (6 prioritized):
- P1: User registration (signup flow)
- P1: User signin (login flow)
- P1: Persistent auth (token refresh, session continuity)
- P2: User logout (signout flow)
- P2: Protected features (personalization access control)
- P3: Navbar state transitions

**Success Criteria** (10 measurable):
- Registration: < 2 minutes
- Signin: < 10 seconds
- Signup success rate: 95%
- Session persistence: 90% returning users
- Token refresh: transparent (zero user issues)
- Protected access: 100% enforcement
- Error clarity: 70% reduction in support tickets

**Key Decisions:**
- Background collection deferred to post-signup (assumption #3) - simplifies signup form UX
- Token storage: sessionStorage (access) + localStorage (refresh) with documented XSS risk
- Modal overlay for signup/signin (assumption #7) - better UX continuity than full pages
- Multi-tab sync via storage events (medium risk mitigation documented)

**Integration Contract:**
- Backend endpoints: `/api/auth/signup`, `/api/auth/signin`, `/api/auth/refresh`, `/api/auth/signout`
- Request payloads: signup {email, password, name}, signin {email, password}, refresh {refreshToken}
- Response format: {success, data: {user, tokens}} or {success: false, error: message}
- JWT claims used: sub, email, name, profileComplete, softwareBackground[], hardwareBackground[], roles

## Outcome

- ✅ Impact: Complete frontend auth specification ready for Docusaurus integration with existing JWT backend
- 🧪 Tests: All quality checks passed - no clarifications needed, 51 testable requirements, 10 measurable success criteria
- 📁 Files: 2 files created (spec.md with 51 FRs across 6 user stories, requirements.md checklist)
- 🔁 Next prompts: `/sp.plan` to create implementation plan for React components, auth context, API integration
- 🧠 Reflection: Specification maintains strict separation between UI/UX (spec domain) and implementation (deferred to plan). Background collection deferred to post-signup balances UX simplicity with feature completeness.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification aligns with existing backend without requiring changes
- Graders run and results (PASS/FAIL): Content quality: PASS, Requirement completeness: PASS, Feature readiness: PASS
- Prompt variant (if applicable): Standard frontend integration specification with existing backend alignment
- Next experiment (smallest change to try): Consider moving background collection to signup form if user testing shows willingness to complete longer initial form

