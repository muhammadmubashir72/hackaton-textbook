---
id: 0002
title: Frontend Auth Implementation Plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5
feature: 005-frontend-auth-integration
branch: 005-frontend-auth-integration
user: developer
command: /sp.plan
labels: ["frontend", "authentication", "react", "docusaurus", "jwt", "implementation-plan"]
links:
  spec: specs/005-frontend-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-frontend-auth-integration/plan.md
tests:
  - Constitutional alignment check
  - Phase dependency validation
  - Integration contract verification
---

## Prompt

/sp.plan mode. Based strictly on the existing authentication backend specification (JWT, Neon DB, Redis) and the previously defined frontend specs, create a clear, step-by-step frontend implementation plan. The plan must cover adding Signup and Signin buttons in the navbar, building Signup and Signin forms, integrating with the auth APIs (/api/auth/signup, /api/auth/signin, /api/auth/refresh, /api/auth/signout), handling JWT storage and refresh, managing global auth state, updating navbar UI based on login status, and gating personalization features behind authentication. Do not introduce new backend logic or features. Focus on sequence, dependencies, API wiring, state management, and error handling. Output the plan in concise, technical Markdown suitable for direct execution.

## Response snapshot

Created comprehensive 8-phase implementation plan for frontend authentication integration with existing JWT backend:

**Plan Document**: `specs/005-frontend-auth-integration/plan.md`

**Phase Breakdown**:

**Phase 0: Research & Technical Validation** (2-3 hours)
- R0-T1: Docusaurus theme swizzling for navbar modification
- R0-T2: React Context API patterns for global auth state
- R0-T3: JWT token storage strategies (sessionStorage vs localStorage, XSS mitigation)
- R0-T4: Axios interceptor for automatic token refresh on 401
- R0-T5: Multi-tab auth synchronization via storage events
- R0-T6: Docusaurus-compatible modal overlay patterns
- Output: `research.md` with technical decisions and rationale

**Phase 1: Architecture & Design** (2-3 hours)
- Data models: AuthState, User, UserProfile, SoftwareBackground, HardwareBackground, JWTTokens
- API contracts: OpenAPI spec for 4 auth endpoints (signup, signin, refresh, signout)
- JWT claims structure: Document expected payload fields and their frontend usage
- Component hierarchy: App → Navbar (AuthButtons | UserDropdown) → Modals
- State transitions: Unauthenticated ↔ Authenticated (Profile Complete/Incomplete)
- Output: `data-model.md`, `contracts/auth-api.yaml`, `contracts/jwt-claims.md`, `quickstart.md`

**Phase 2.1: Core Infrastructure** (3-4 hours, 5 tasks)
- P2.1-T1: Auth API client (axios instance, 4 endpoint methods)
- P2.1-T2: Token service (store/get/clear tokens, sessionStorage + localStorage)
- P2.1-T3: JWT decoder utility (jwt-decode wrapper)
- P2.1-T4: Auth context provider (React Context + useReducer, state initialization)
- P2.1-T5: useAuth hook (context consumer with helper methods)

**Phase 2.2: Authentication Forms** (4-5 hours, 4 tasks)
- P2.2-T1: Validators (email format, password strength, password match)
- P2.2-T2: SignupModal (form with validation, API integration, error handling)
- P2.2-T3: SigninModal (form with validation, account lock handling)
- P2.2-T4: Base Modal component (React portal, focus trap, ARIA)

**Phase 2.3: Navbar Integration** (3-4 hours, 4 tasks)
- P2.3-T1: Swizzle Docusaurus navbar (npm run swizzle)
- P2.3-T2: AuthButtons component (Sign Up / Sign In buttons)
- P2.3-T3: UserDropdown component (name display, Profile/Logout menu)
- P2.3-T4: Integrate auth components into swizzled navbar

**Phase 2.4: Automatic Token Refresh** (2-3 hours, 4 tasks)
- P2.4-T1: Request interceptor (add Authorization header)
- P2.4-T2: Response interceptor (catch 401, trigger refresh, retry)
- P2.4-T3: Refresh token function (call /api/auth/refresh)
- P2.4-T4: Concurrent refresh handling (promise queue pattern)

**Phase 2.5: Protected Features** (2-3 hours, 3 tasks)
- P2.5-T1: useProtectedFeature hook (check auth + profile complete)
- P2.5-T2: ProtectedFeature wrapper component (guard with signin/profile prompts)
- P2.5-T3: Wrap personalization button with protection

**Phase 2.6: Multi-Tab Synchronization** (2-3 hours, 3 tasks)
- P2.6-T1: Storage event listener (sync logout across tabs)
- P2.6-T2: Session expiry warning (1 min before token expiry)
- P2.6-T3: Page visibility refresh (refresh when tab regains focus)

**Phase 2.7: Error Handling & User Feedback** (2-3 hours, 4 tasks)
- P2.7-T1: Toast notification component
- P2.7-T2: Integrate toasts into auth context
- P2.7-T3: Standardize error messages (user-friendly mapping)
- P2.7-T4: Add loading states to forms

**Phase 2.8: Testing & Integration Verification** (3-4 hours, 4 tasks)
- P2.8-T1: Manual test checklist (all auth flows)
- P2.8-T2: Verify backend integration (CORS, JWT claims, expiry)
- P2.8-T3: Browser compatibility test (Chrome, Firefox, Safari, Edge)
- P2.8-T4: Accessibility check (keyboard nav, screen readers)

**File Manifest**: 16 new files
- 3 services (authAPI, tokenService, jwtDecoder)
- 2 state management (AuthContext, useAuth hook)
- 6 components (modals, buttons, dropdown, toast, protected wrapper)
- 2 hooks (useAuth, useProtectedFeature)
- 2 utilities (validators, errorMessages)
- 1 theme (swizzled navbar)

**Integration Points** (Read-Only):
- POST /api/auth/signup → {email, password, name} → {user, tokens}
- POST /api/auth/signin → {email, password} → {user, tokens, profile}
- POST /api/auth/refresh → {refreshToken} → {accessToken}
- POST /api/auth/signout → Authorization header + {refreshToken} → {success}

**Token Storage Strategy**:
- Access token: sessionStorage (cleared on tab close, XSS mitigation)
- Refresh token: localStorage (persists across sessions, documented tradeoff)

**Critical Path**: Phase 0 → 1 → 2.1 → 2.2 → 2.3 → 2.8

**Parallel Opportunities**: Phases 2.4-2.7 can overlap once core infrastructure (2.1) complete

**Total Effort**: 25-35 hours (3-5 working days)

## Outcome

- ✅ Impact: Complete frontend implementation plan ready for Docusaurus authentication UI with existing JWT backend
- 🧪 Tests: Constitutional alignment PASS, all phases have clear deliverables and acceptance criteria
- 📁 Files: 1 file created (plan.md with 8 phases, 25+ tasks, 16 component files specified)
- 🔁 Next prompts: `/sp.tasks` to generate granular task breakdown with step-by-step implementation guides
- 🧠 Reflection: Plan maintains strict frontend-only scope, no backend modifications. Token storage security tradeoff documented. Deferred background collection to post-signup simplifies UX while preserving personalization capability.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan integrates with existing backend without requiring API changes
- Graders run and results (PASS/FAIL): Constitutional alignment: PASS, Phase dependencies: PASS, Integration contracts: PASS
- Prompt variant (if applicable): Standard frontend integration plan with existing backend
- Next experiment (smallest change to try): Consider adding optimistic UI updates (update navbar before API response) to reduce perceived latency
