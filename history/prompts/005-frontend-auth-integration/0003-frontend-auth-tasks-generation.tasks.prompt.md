---
id: 0003
title: Frontend Auth Tasks Generation
stage: tasks
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5
feature: 005-frontend-auth-integration
branch: 005-frontend-auth-integration
user: developer
command: /sp.task
labels: ["frontend", "authentication", "tasks", "implementation", "docusaurus", "react", "jwt"]
links:
  spec: specs/005-frontend-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-frontend-auth-integration/tasks.md
tests:
  - Task dependency validation
  - Critical path verification
  - Acceptance criteria completeness
---

## Prompt

/sp.task mode. Based on the approved frontend plan and the existing authentication backend specification, break the work into clear, executable frontend development tasks. Tasks must include adding Signup and Signin buttons to the navbar, implementing Signup and Signin forms (including software and hardware background fields), wiring forms to the auth APIs, handling JWT storage and refresh, managing global auth state, updating navbar UI on login/logout, protecting personalization features behind authentication, and implementing loading and error states. Do not add new features or modify backend behavior. Output concise, implementation-focused tasks in Markdown suitable for direct execution by a frontend team.

## Response snapshot

Generated comprehensive 37-task breakdown for frontend authentication integration across 10 phases:

**Task Document**: `specs/005-frontend-auth-integration/tasks.md`

**Phase Structure**:

**Phase 0: Research & Technical Validation** (2-3 hours, 6 tasks)
- R0-T1: Docusaurus theme swizzling research (swizzle navbar, preserve features)
- R0-T2: React Context API patterns (state shape, useReducer pattern)
- R0-T3: JWT token storage strategy (sessionStorage + localStorage, security tradeoffs)
- R0-T4: Axios interceptor pattern (auto-refresh on 401, retry logic)
- R0-T5: Multi-tab auth sync (storage events, cross-tab logout)
- R0-T6: Modal component pattern (React portal, focus management, ARIA)
- Output: `research.md` with 6 technical decision sections

**Phase 1: Architecture & Design** (2-3 hours, 4 tasks)
- P1-T1: Define data models (AuthState, User, Backgrounds, JWTTokens interfaces)
- P1-T2: Document API contracts (OpenAPI spec for 4 endpoints)
- P1-T3: Document JWT claims structure (all fields + frontend usage)
- P1-T4: Create quickstart guide (9-step implementation, ~70 min total)
- Output: `data-model.md`, `contracts/auth-api.yaml`, `contracts/jwt-claims.md`, `quickstart.md`

**Phase 2.1: Core Infrastructure** (3-4 hours, 5 tasks)
- P2.1-T1: Auth API client (axios instance, 4 methods: signup/signin/refresh/signout)
- P2.1-T2: Token service (store/get/clear tokens, sessionStorage + localStorage)
- P2.1-T3: JWT decoder utility (jwt-decode wrapper)
- P2.1-T4: Auth context provider (React Context + useReducer, auto-init on mount)
- P2.1-T5: useAuth hook (context consumer, helper methods)
- Files: `authAPI.js`, `tokenService.js`, `jwtDecoder.js`, `AuthContext.jsx`, `useAuth.js`

**Phase 2.2: Authentication Forms** (4-5 hours, 4 tasks)
- P2.2-T1: Validators (email format, password strength, password match)
- P2.2-T2: Signup modal (form with name/email/password/confirmation, validation, API integration)
- P2.2-T3: Signin modal (email/password, account lock handling, error preservation)
- P2.2-T4: Base modal component (React portal, focus trap, Escape handler, ARIA)
- Files: `validators.js`, `SignupModal.jsx`, `SigninModal.jsx`, `Modal.jsx`

**Phase 2.3: Navbar Integration** (3-4 hours, 4 tasks)
- P2.3-T1: Swizzle Docusaurus navbar (`npm run swizzle`, preserve features)
- P2.3-T2: AuthButtons component (Sign Up/Sign In, open modals)
- P2.3-T3: UserDropdown component (name display, profile/logout menu, completion badge)
- P2.3-T4: Integrate into swizzled navbar (conditional rendering based on auth state)
- Files: `theme/Navbar/index.jsx`, `AuthButtons.jsx`, `UserDropdown.jsx`

**Phase 2.4: Automatic Token Refresh** (2-3 hours, 4 tasks)
- P2.4-T1: Request interceptor (add Authorization header, skip public endpoints)
- P2.4-T2: Response interceptor (catch 401, trigger refresh, retry)
- P2.4-T3: Refresh token function (POST /api/auth/refresh)
- P2.4-T4: Concurrent refresh handling (promise queue, prevent multiple calls)
- Modified: `authAPI.js` (add interceptors)

**Phase 2.5: Protected Features** (2-3 hours, 3 tasks)
- P2.5-T1: useProtectedFeature hook (check auth + profileComplete, return access + reason)
- P2.5-T2: ProtectedFeature wrapper component (intercept clicks, show signin/profile prompts)
- P2.5-T3: Wrap personalization button (requireCompleteProfile={true})
- Files: `useProtectedFeature.js`, `ProtectedFeature.jsx`

**Phase 2.6: Multi-Tab Synchronization** (2-3 hours, 3 tasks)
- P2.6-T1: Storage event listener (sync logout across tabs)
- P2.6-T2: Session expiry warning (1 min before token expiry)
- P2.6-T3: Page visibility refresh (refresh when tab regains focus)
- Modified: `AuthContext.jsx` (add event listeners)

**Phase 2.7: Error Handling & User Feedback** (2-3 hours, 4 tasks)
- P2.7-T1: Toast notification component (success/error/warning, auto-dismiss)
- P2.7-T2: Integrate toasts into auth context (all auth events)
- P2.7-T3: Standardize error messages (map status codes to user-friendly text)
- P2.7-T4: Add loading states to forms (spinner, disabled inputs)
- Files: `Toast.jsx`, `errorMessages.js`

**Phase 2.8: Testing & Integration Verification** (3-4 hours, 4 tasks)
- P2.8-T1: Manual test checklist (8 categories: signup, signin, refresh, logout, multi-tab, protected, errors, persistence)
- P2.8-T2: Verify backend integration (CORS, JWT claims, expiry times)
- P2.8-T3: Browser compatibility test (Chrome, Firefox, Safari, Edge)
- P2.8-T4: Accessibility check (keyboard nav, screen reader, focus trap)

**File Manifest**: 16 new files
- 3 services (authAPI, tokenService, jwtDecoder)
- 2 state management (AuthContext, useAuth hook)
- 6 components (SignupModal, SigninModal, Modal, AuthButtons, UserDropdown, Toast, ProtectedFeature)
- 2 hooks (useAuth, useProtectedFeature)
- 2 utilities (validators, errorMessages)
- 1 theme (swizzled Navbar)

**Key Implementation Details**:
- Token storage: sessionStorage (access, 15min), localStorage (refresh, 7d)
- State management: React Context + useReducer
- Token refresh: Axios interceptors with promise queue (prevents concurrent refresh calls)
- Multi-tab sync: Storage events for cross-tab logout
- Protected features: HOC checking auth + profileComplete
- Error handling: Toast notifications, user-friendly messages mapped from status codes
- Background collection: Deferred to post-signup profile completion (noted in tasks but not implemented in initial signup form)

**Critical Path**: Phase 0 → 1 → 2.1 → 2.2 → 2.3 → 2.8

**Parallel Opportunities**: Phases 2.4-2.7 can overlap after Phase 2.1 complete

**Total Effort**: 25-35 hours (3-5 working days)

## Outcome

- ✅ Impact: Complete implementation task breakdown ready for frontend development team with 37 executable tasks
- 🧪 Tests: Task dependency validation PASS, Critical path verified, All acceptance criteria defined
- 📁 Files: 1 file created (tasks.md with 37 tasks across 10 phases, 16 component files specified)
- 🔁 Next prompts: Begin Phase 0 research, or directly implement Phase 2.1 if research already completed
- 🧠 Reflection: Task breakdown maintains frontend-only scope, includes comprehensive testing phase. Each task has clear steps, testing instructions, and acceptance criteria. Background collection correctly deferred to post-signup.

## Evaluation notes (flywheel)

- Failure modes observed: None - tasks align with plan and spec without introducing new features
- Graders run and results (PASS/FAIL): Task dependency validation: PASS, Acceptance criteria completeness: PASS, Implementation readiness: PASS
- Prompt variant (if applicable): Standard frontend task breakdown with existing backend integration
- Next experiment (smallest change to try): Consider adding automated testing tasks (unit tests, integration tests) in addition to manual testing checklist
