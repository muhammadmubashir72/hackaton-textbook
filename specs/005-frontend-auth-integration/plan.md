# Implementation Plan: Frontend Authentication Integration

**Branch**: `005-frontend-auth-integration` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-frontend-auth-integration/spec.md`

## Summary

Integrate frontend authentication UI with existing JWT backend (auth-backend on port 3001). Implement navbar auth buttons (Sign Up/Sign In for unauthenticated, user dropdown with Profile/Logout for authenticated), signup/signin modal forms, global auth state management, secure JWT token storage (sessionStorage for access, localStorage for refresh), automatic token refresh on 401 responses, and protection of personalization features behind authentication checks. Frontend is pure integration layer - no backend modifications.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Docusaurus default), React 18+
**Primary Dependencies**: Docusaurus 3.x (existing), axios/fetch for API calls, React Context API for global state, jwt-decode for token parsing
**Storage**: Browser APIs (sessionStorage for access tokens, localStorage for refresh tokens)
**Testing**: Manual testing via test.html, Docusaurus dev server verification
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge latest 2 versions)
**Project Type**: Web application (Docusaurus static site with client-side auth)
**Performance Goals**: Page load < 2s, token validation < 500ms, API calls < 3s, UI state updates < 100ms
**Constraints**: Must work within Docusaurus framework (limited routing, theme swizzling for navbar), cannot modify backend APIs, XSS mitigation for token storage
**Scale/Scope**: Single-user sessions, support for concurrent multi-device logins, 10k+ users expected

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution

**✅ II. AI-Native Educational Experience**
- Integrates authentication with existing AI-native stack (Docusaurus + FastAPI + Neon + better-auth backend)
- Enables personalized learning experiences based on user authentication and profile data

**✅ V. Advanced Technology Integration**
- Uses specified stack: better-auth (backend already implemented), integrates with Neon Postgres + Redis via existing APIs
- No new technology introduced, pure frontend integration

**✅ VI. Accessibility and Inclusion**
- Authentication enables personalized content delivery based on user software/hardware background
- Supports diverse learners by gating personalization features appropriately

**✅ VII. Authentication and Personalization**
- Implements frontend for better-auth signup/signin flows
- Collects user background during signup (deferred to profile completion for UX simplicity)
- Enables content adjustment based on user level via JWT claims

### Gate Status: ✅ PASS

No constitutional violations. Feature aligns with principles II, V, VI, VII directly. No new backend logic or technology stack changes required.

## Project Structure

### Documentation (this feature)

```text
specs/005-frontend-auth-integration/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (technical research)
├── data-model.md        # Phase 1 output (state models)
├── contracts/           # Phase 1 output (API integration contracts)
│   ├── auth-api.yaml    # OpenAPI spec for backend auth endpoints
│   └── jwt-claims.md    # JWT payload structure documentation
├── quickstart.md        # Phase 1 output (developer quick start)
├── checklists/
│   └── requirements.md  # Quality validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (Docusaurus frontend)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupModal.jsx       # Signup form modal
│   │   │   ├── SigninModal.jsx       # Signin form modal
│   │   │   ├── AuthButtons.jsx       # Sign Up/Sign In navbar buttons
│   │   │   └── UserDropdown.jsx      # Authenticated user menu
│   │   └── ProtectedFeature.jsx      # HOC/wrapper for protected features
│   ├── context/
│   │   └── AuthContext.jsx           # Global auth state provider
│   ├── hooks/
│   │   ├── useAuth.js                # Auth state hook
│   │   └── useProtectedFeature.js    # Protected feature hook
│   ├── services/
│   │   ├── authAPI.js                # API client for auth endpoints
│   │   └── tokenService.js           # Token storage/retrieval/refresh
│   ├── utils/
│   │   ├── jwtDecoder.js             # JWT parsing utility
│   │   └── validators.js             # Email/password validation
│   └── theme/
│       └── Navbar/                   # Swizzled Docusaurus navbar
│           └── index.jsx             # Modified navbar with auth buttons
└── static/
    └── test.html                     # Auth API tester (already created)
```

**Structure Decision**: Existing Docusaurus web application structure. Authentication components integrated into `src/components/Auth/`, global state via `src/context/AuthContext.jsx`, navbar modified via Docusaurus theme swizzling (`src/theme/Navbar/`). No backend structure changes needed.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. This section is not applicable.

---

## Phase 0: Research & Technical Validation

### Research Tasks

**R0-T1: Docusaurus Theme Swizzling for Navbar**
- Research: How to swizzle Docusaurus navbar component to add custom auth buttons
- Deliverable: Documentation on theme swizzling process, file locations, component override patterns
- Acceptance: Clear steps to override navbar without breaking Docusaurus updates

**R0-T2: React Context API for Global Auth State**
- Research: Best practices for React Context with auth state (user, tokens, authentication status)
- Deliverable: Context provider pattern, state shape definition, update patterns
- Acceptance: Context accessible to all components, performant (no unnecessary re-renders)

**R0-T3: Secure JWT Token Storage in Browser**
- Research: sessionStorage vs localStorage vs cookies for JWT tokens, XSS mitigation strategies
- Deliverable: Token storage strategy with security tradeoffs documented, CSP header recommendations
- Acceptance: Access tokens in sessionStorage (cleared on tab close), refresh tokens in localStorage (persists), XSS risks documented

**R0-T4: Axios Interceptor for Automatic Token Refresh**
- Research: How to intercept 401 responses, refresh token, retry original request without infinite loops
- Deliverable: Interceptor implementation pattern, retry logic with max attempts
- Acceptance: 401 errors trigger refresh → retry, refresh failure signs out, max 1 retry per request

**R0-T5: Multi-Tab Auth Synchronization**
- Research: Browser storage events for cross-tab communication, logout synchronization patterns
- Deliverable: Storage event listener pattern for auth state broadcast
- Acceptance: Logout in one tab updates all tabs to unauthenticated state within 1 second

**R0-T6: Docusaurus Modal Integration**
- Research: Modal overlay patterns in Docusaurus (portal rendering, focus management, accessibility)
- Deliverable: Modal component structure, Docusaurus-compatible implementation
- Acceptance: Modal overlays existing content, handles keyboard (Escape to close), focuses first input, accessible (ARIA)

### Research Output

**File**: `specs/005-frontend-auth-integration/research.md`

**Expected Structure**:
```markdown
# Frontend Auth Integration - Research Findings

## R0-T1: Docusaurus Theme Swizzling
**Decision**: Swizzle navbar via `npm run swizzle @docusaurus/theme-classic Navbar`
**Rationale**: Official Docusaurus method, maintains upgrade compatibility
**Implementation**: Copy navbar to `src/theme/Navbar/index.jsx`, inject AuthButtons component

## R0-T2: React Context for Auth State
**Decision**: Use React Context API with useReducer for state management
**Rationale**: Built-in React feature, no external dependencies, sufficient for auth state complexity
**State Shape**:
```javascript
{
  isAuthenticated: boolean,
  user: { id, email, name, profileComplete } | null,
  loading: boolean,
  error: string | null
}
```

## R0-T3: JWT Token Storage
**Decision**: sessionStorage (access), localStorage (refresh)
**Rationale**: sessionStorage cleared on tab close (limits XSS window), localStorage enables persistent sessions
**Security**: CSP headers (script-src 'self'), input sanitization, future: httpOnly cookies for refresh token

## R0-T4: Axios Interceptor Pattern
**Decision**: Single axios instance with request/response interceptors
**Rationale**: Centralized token injection, automatic 401 handling
**Retry Logic**: Track retry count in request config, max 1 retry per request

## R0-T5: Multi-Tab Sync
**Decision**: window.addEventListener('storage', handler)
**Rationale**: Standard browser API for cross-tab communication
**Implementation**: Listen for 'authTokens' key changes, update auth context on removal

## R0-T6: Modal Component
**Decision**: React portal + Docusaurus-safe modal
**Rationale**: Renders modal outside Docusaurus DOM tree, prevents z-index conflicts
**Accessibility**: Focus trap, Escape key handler, ARIA dialog role
```

---

## Phase 1: Architecture & Design

### Data Models

**File**: `specs/005-frontend-auth-integration/data-model.md`

**Auth State Model**:
```typescript
interface AuthState {
  isAuthenticated: boolean;
  user: User | null;
  loading: boolean;
  error: string | null;
}

interface User {
  id: string;
  email: string;
  name: string;
  profileComplete: boolean;
  softwareBackground: SoftwareBackground | null;
  hardwareBackground: HardwareBackground | null;
}

interface SoftwareBackground {
  programmingLanguages: string[];
  frameworks: string[];
  experienceLevel: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  specializations: string[];
  yearsOfExperience: number;
}

interface HardwareBackground {
  familiarPlatforms: string[];
  roboticsExperience: 'none' | 'hobbyist' | 'professional';
  electronicsKnowledge: 'none' | 'basic' | 'intermediate' | 'advanced';
  preferredTools: string[];
}

interface JWTTokens {
  accessToken: string;
  refreshToken: string;
  expiresIn: number;
}
```

**State Transitions**:
```
[Unauthenticated]
  └─ signup/signin success → [Authenticated (Profile Incomplete)]
       └─ profile completion → [Authenticated (Profile Complete)]
            └─ signout → [Unauthenticated]

Token expiry triggers:
  - Access token expired + valid refresh → auto-refresh → stay [Authenticated]
  - Both expired → auto-signout → [Unauthenticated]
```

### API Integration Contracts

**File**: `specs/005-frontend-auth-integration/contracts/auth-api.yaml`

**Endpoints Consumed** (read-only, no modifications):

```yaml
openapi: 3.0.0
info:
  title: Authentication Backend API
  version: 1.0.0
  description: Existing JWT authentication backend (auth-backend on port 3001)

servers:
  - url: http://localhost:3001
    description: Development server
  - url: https://auth.yourapp.com
    description: Production server

paths:
  /api/auth/signup:
    post:
      summary: Register new user
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password, name]
              properties:
                email:
                  type: string
                  format: email
                password:
                  type: string
                  minLength: 8
                name:
                  type: string
                  minLength: 1
      responses:
        '201':
          description: User created successfully
          content:
            application/json:
              schema:
                type: object
                properties:
                  success:
                    type: boolean
                    example: true
                  data:
                    type: object
                    properties:
                      user:
                        $ref: '#/components/schemas/User'
                      tokens:
                        $ref: '#/components/schemas/TokenPair'
        '400':
          description: Validation error
        '409':
          description: Email already registered

  /api/auth/signin:
    post:
      summary: Sign in existing user
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password]
              properties:
                email:
                  type: string
                  format: email
                password:
                  type: string
      responses:
        '200':
          description: Sign in successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  success:
                    type: boolean
                  data:
                    type: object
                    properties:
                      user:
                        $ref: '#/components/schemas/UserWithProfile'
                      tokens:
                        $ref: '#/components/schemas/TokenPair'
        '401':
          description: Invalid credentials
        '429':
          description: Account locked (too many failed attempts)

  /api/auth/refresh:
    post:
      summary: Refresh access token
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [refreshToken]
              properties:
                refreshToken:
                  type: string
      responses:
        '200':
          description: Token refreshed
          content:
            application/json:
              schema:
                type: object
                properties:
                  success:
                    type: boolean
                  data:
                    type: object
                    properties:
                      accessToken:
                        type: string
                      expiresIn:
                        type: integer
                        example: 900
        '401':
          description: Invalid or expired refresh token

  /api/auth/signout:
    post:
      summary: Sign out (revoke tokens)
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [refreshToken]
              properties:
                refreshToken:
                  type: string
      responses:
        '200':
          description: Signed out successfully
        '401':
          description: Invalid token

components:
  schemas:
    User:
      type: object
      properties:
        id:
          type: string
          format: uuid
        email:
          type: string
        name:
          type: string
        profileComplete:
          type: boolean

    UserWithProfile:
      allOf:
        - $ref: '#/components/schemas/User'
        - type: object
          properties:
            softwareBackground:
              type: object
              nullable: true
            hardwareBackground:
              type: object
              nullable: true

    TokenPair:
      type: object
      properties:
        accessToken:
          type: string
        refreshToken:
          type: string
        expiresIn:
          type: integer

  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
```

### JWT Claims Structure

**File**: `specs/005-frontend-auth-integration/contracts/jwt-claims.md`

**Access Token Payload** (decoded client-side):
```json
{
  "sub": "user-uuid",
  "email": "user@example.com",
  "name": "User Name",
  "profileComplete": false,
  "softwareBackground": ["Python", "JavaScript"],
  "hardwareBackground": ["Raspberry Pi", "Arduino"],
  "roles": ["user"],
  "iss": "physical-ai-textbook",
  "jti": "token-uuid",
  "iat": 1703001234,
  "exp": 1703002134
}
```

**Claims Used in Frontend**:
- `sub` - User ID (for API calls)
- `email` - Display in navbar
- `name` - Display in navbar, personalization
- `profileComplete` - Gate personalization features
- `softwareBackground[]` - First 5 languages for UI hints
- `hardwareBackground[]` - First 5 platforms for UI hints
- `exp` - Token expiry (for refresh timing)

### Component Architecture

**File**: `specs/005-frontend-auth-integration/data-model.md`

#### Component Hierarchy

```
App (with AuthProvider)
└─ Navbar (swizzled)
    ├─ AuthButtons (if !isAuthenticated)
    │   ├─ SignupButton → SignupModal
    │   └─ SigninButton → SigninModal
    └─ UserDropdown (if isAuthenticated)
        ├─ UserName display
        ├─ ProfileLink
        └─ LogoutButton

Page (any)
└─ ProtectedFeature (personalization button)
    └─ [If authenticated + profileComplete] → execute
    └─ [If authenticated + !profileComplete] → prompt profile completion
    └─ [If !authenticated] → open SigninModal
```

#### State Flow

```
[App Load]
  → Check sessionStorage for accessToken
  → If exists → verify not expired
  → If expired → call /api/auth/refresh with localStorage refreshToken
  → If refresh succeeds → update accessToken in sessionStorage
  → If refresh fails → clear storage, set isAuthenticated = false
  → Update AuthContext state

[Signup Flow]
  → User clicks "Sign Up" button
  → SignupModal opens
  → User fills email, password, confirmation, name
  → Validate client-side (email format, password length, match)
  → POST /api/auth/signup
  → On 201: store tokens, update AuthContext, close modal, show toast
  → On error: display error message, keep modal open

[Signin Flow]
  → User clicks "Sign In" button
  → SigninModal opens
  → User fills email, password
  → Validate client-side (email format, password not empty)
  → POST /api/auth/signin
  → On 200: store tokens, store user profile, update AuthContext, close modal, show toast
  → On error: display error, preserve email, clear password

[Logout Flow]
  → User clicks "Logout" in dropdown
  → Confirm (optional: skip for now)
  → POST /api/auth/signout with accessToken header + refreshToken body
  → Clear sessionStorage, clear localStorage
  → Update AuthContext to unauthenticated
  → Broadcast logout to other tabs via storage event
  → Show toast "Signed out successfully"

[Protected Feature Access]
  → User clicks personalization button
  → Check AuthContext.isAuthenticated
  → If false → open SigninModal with message
  → If true → check AuthContext.user.profileComplete
  → If false → show prompt "Complete your profile"
  → If true → execute personalization API call with accessToken in header
  → If API returns 401 → trigger token refresh → retry
  → If refresh fails → sign out user
```

### Quickstart Guide

**File**: `specs/005-frontend-auth-integration/quickstart.md`

```markdown
# Frontend Auth Integration - Developer Quickstart

## Prerequisites

1. Auth backend running on `http://localhost:3001`
2. Docusaurus frontend running on `http://localhost:3000`
3. Database migrations completed

## Implementation Steps

### Step 1: Install Dependencies (2 min)
```bash
cd frontend
npm install axios jwt-decode
```

### Step 2: Create Auth API Client (5 min)
File: `src/services/authAPI.js`
- Configure axios base URL: `http://localhost:3001/api/auth`
- Export: signup(), signin(), refresh(), signout()

### Step 3: Create Token Service (5 min)
File: `src/services/tokenService.js`
- Export: storeTokens(), getAccessToken(), getRefreshToken(), clearTokens()
- Use sessionStorage for access, localStorage for refresh

### Step 4: Create Auth Context (10 min)
File: `src/context/AuthContext.jsx`
- Provider with state: {isAuthenticated, user, loading, error}
- Methods: login(), signup(), logout(), refreshToken()
- Initialize on mount by checking stored tokens

### Step 5: Swizzle Navbar (5 min)
```bash
npm run swizzle @docusaurus/theme-classic Navbar -- --eject
```
- Modify: `src/theme/Navbar/index.jsx`
- Add: AuthButtons or UserDropdown based on auth state

### Step 6: Create Auth Components (15 min)
- `src/components/Auth/SignupModal.jsx`
- `src/components/Auth/SigninModal.jsx`
- `src/components/Auth/UserDropdown.jsx`

### Step 7: Add Axios Interceptor (10 min)
- Configure in `src/services/authAPI.js`
- Intercept 401 → refresh → retry

### Step 8: Protect Personalization Features (5 min)
- Wrap personalization button with auth check
- Use AuthContext.isAuthenticated + user.profileComplete

### Step 9: Test (10 min)
- Signup new user
- Signin existing user
- Test token refresh (wait 15 min or manually expire)
- Test logout
- Test protected features

**Total Time**: ~70 minutes
```

---

## Phase 2: Implementation Phases

### Phase 2.1: Core Infrastructure (Foundation)

**Goal**: Establish auth API client, token service, and global state management

**Tasks**:

**P2.1-T1: Create Auth API Client**
- File: `frontend/src/services/authAPI.js`
- Create axios instance with base URL from environment variable
- Implement methods: signup(email, password, name), signin(email, password), refresh(refreshToken), signout(accessToken, refreshToken)
- Configure timeout: 5 seconds
- Export API object

**P2.1-T2: Create Token Service**
- File: `frontend/src/services/tokenService.js`
- Implement: storeTokens({accessToken, refreshToken}), getAccessToken(), getRefreshToken(), clearTokens()
- Use sessionStorage.setItem('accessToken', token) for access token
- Use localStorage.setItem('refreshToken', token) for refresh token
- Implement: isTokenExpired(token) using jwt-decode to check exp claim

**P2.1-T3: Create JWT Decoder Utility**
- File: `frontend/src/utils/jwtDecoder.js`
- Use jwt-decode library to parse JWT payload
- Implement: decodeAccessToken(token) → returns claims {sub, email, name, profileComplete, backgrounds, roles}
- Handle decode errors gracefully (return null on invalid token)

**P2.1-T4: Create Auth Context Provider**
- File: `frontend/src/context/AuthContext.jsx`
- Create AuthContext with React.createContext()
- Create AuthProvider component with useReducer for state management
- State: {isAuthenticated: false, user: null, loading: true, error: null}
- Actions: SET_USER, SET_LOADING, SET_ERROR, CLEAR_USER
- Implement methods: signup(email, password, name), signin(email, password), logout(), refreshAccessToken()
- Initialize state on mount: check stored tokens → validate → auto-refresh if needed

**P2.1-T5: Create useAuth Hook**
- File: `frontend/src/hooks/useAuth.js`
- Export useAuth() hook to access AuthContext
- Return: {isAuthenticated, user, loading, error, signup, signin, logout, hasCompleteProfile()}
- Throw error if used outside AuthProvider

**Dependencies**: None (foundation layer)
**Acceptance**: Auth context accessible to all components, tokens stored/retrieved correctly, API client configured

---

### Phase 2.2: Authentication Forms

**Goal**: Implement signup and signin modal forms with validation and API integration

**Prerequisites**: Phase 2.1 complete (Auth context, API client available)

**Tasks**:

**P2.2-T1: Create Validators**
- File: `frontend/src/utils/validators.js`
- Implement: validateEmail(email) → boolean (regex: /^[^\\s@]+@[^\\s@]+\\.[^\\s@]+$/)
- Implement: validatePassword(password) → {valid: boolean, message: string} (min 8 chars)
- Implement: validatePasswordMatch(password, confirmation) → boolean

**P2.2-T2: Create Signup Modal Component**
- File: `frontend/src/components/Auth/SignupModal.jsx`
- Props: isOpen, onClose
- Form fields: email, password, passwordConfirmation, name
- Client-side validation on blur and submit
- On submit: call authContext.signup(email, password, name)
- Handle loading state: disable inputs, show spinner
- Handle errors: display error message from context
- Handle success: close modal, show toast "Account created!"
- Note in UI: "Complete your profile after signup for personalized content"

**P2.2-T3: Create Signin Modal Component**
- File: `frontend/src/components/Auth/SigninModal.jsx`
- Props: isOpen, onClose
- Form fields: email, password
- Client-side validation on blur and submit
- On submit: call authContext.signin(email, password)
- Handle loading state: disable inputs, show spinner
- Handle errors: display error, preserve email, clear password
- Handle account lock error (429): show "Too many attempts. Try again in 1 hour"
- Handle success: close modal, show toast "Welcome back, {name}!"
- Include "Forgot Password?" link (disabled/grayed out with tooltip: "Coming soon")

**P2.2-T4: Create Modal Base Component**
- File: `frontend/src/components/Auth/Modal.jsx`
- Reusable modal wrapper using React portal
- Props: isOpen, onClose, title, children
- Render to document.body via createPortal
- Overlay: semi-transparent dark background, click to close
- Modal: centered white card, close X button, escape key handler
- Focus management: trap focus within modal, return focus on close
- ARIA: role="dialog", aria-labelledby, aria-modal="true"

**Dependencies**: P2.1-T4 (Auth context), P2.1-T1 (API client), P2.2-T1 (Validators)
**Acceptance**: Modals open/close correctly, forms submit to backend, errors display, success updates auth state

---

### Phase 2.3: Navbar Integration

**Goal**: Swizzle Docusaurus navbar and integrate auth buttons/user dropdown

**Prerequisites**: Phase 2.1 complete (Auth context), Phase 2.2 complete (Modals)

**Tasks**:

**P2.3-T1: Swizzle Docusaurus Navbar**
- Command: `npm run swizzle @docusaurus/theme-classic Navbar -- --eject`
- This copies navbar component to `frontend/src/theme/Navbar/index.jsx`
- Preserve existing navbar functionality (logo, links, search, theme toggle)

**P2.3-T2: Create Auth Buttons Component**
- File: `frontend/src/components/Auth/AuthButtons.jsx`
- Render two buttons: "Sign Up" (primary style) and "Sign In" (secondary style)
- On click: open respective modal (useState to control modal visibility)
- Style: Match Docusaurus theme colors, responsive (stack vertically on mobile)

**P2.3-T3: Create User Dropdown Component**
- File: `frontend/src/components/Auth/UserDropdown.jsx`
- Display: User name (or truncated email if name unavailable)
- On click: toggle dropdown menu
- Menu items: "Profile" (link to /profile), "Logout" (button)
- If profileComplete: false → show badge/indicator "Complete Profile"
- On logout click: call authContext.logout()
- Style: Match Docusaurus navbar dropdown style

**P2.3-T4: Integrate Auth Components into Navbar**
- File: `frontend/src/theme/Navbar/index.jsx`
- Import AuthButtons and UserDropdown
- Import useAuth hook
- Conditional rendering: `{isAuthenticated ? <UserDropdown /> : <AuthButtons />}`
- Position: Right side of navbar (before theme toggle)
- Test: Navbar updates when auth state changes (signin, signout)

**Dependencies**: P2.1-T4 (Auth context), P2.1-T5 (useAuth hook), P2.2-T2, P2.2-T3 (Modals), P2.3-T2, P2.3-T3 (Auth components)
**Acceptance**: Navbar shows correct buttons based on auth state, modals open on click, dropdown menu works, logout triggers signout

---

### Phase 2.4: Automatic Token Refresh

**Goal**: Implement axios interceptor for automatic token refresh on 401 responses

**Prerequisites**: Phase 2.1 complete (Token service, API client)

**Tasks**:

**P2.4-T1: Add Request Interceptor**
- File: `frontend/src/services/authAPI.js`
- Axios request interceptor: add Authorization header with accessToken if available
- Skip Authorization header for /signup and /signin endpoints
- Implementation: `config.headers.Authorization = Bearer ${getAccessToken()}`

**P2.4-T2: Add Response Interceptor for 401 Handling**
- File: `frontend/src/services/authAPI.js`
- Axios response interceptor: catch 401 errors
- On 401: check if request already retried (config._retry flag)
- If not retried: call refreshAccessToken() from token service
- If refresh succeeds: update stored access token, set config._retry = true, retry original request
- If refresh fails (401): clear tokens, update auth context to unauthenticated, throw error
- Max 1 retry per request (check config._retry to prevent infinite loops)

**P2.4-T3: Implement Refresh Token Function**
- File: `frontend/src/services/tokenService.js`
- Add refreshAccessToken() method
- Get refresh token from localStorage
- Call POST /api/auth/refresh with {refreshToken}
- On success: extract new accessToken, store in sessionStorage, return token
- On failure: throw error (caught by interceptor → triggers signout)

**P2.4-T4: Handle Concurrent Refresh Requests**
- Problem: Multiple 401s at same time trigger multiple refresh calls
- Solution: Implement promise queue - first refresh creates promise, subsequent requests wait for same promise
- Implementation: Use global variable to track ongoing refresh promise, return existing promise if already refreshing

**Dependencies**: P2.1-T1 (API client), P2.1-T2 (Token service), P2.1-T4 (Auth context)
**Acceptance**: 401 errors trigger automatic refresh, original request retries with new token, refresh failure signs user out, no infinite loops

---

### Phase 2.5: Protected Features

**Goal**: Implement auth guards for personalization features

**Prerequisites**: Phase 2.1 complete (Auth context), Phase 2.4 complete (Token refresh)

**Tasks**:

**P2.5-T1: Create useProtectedFeature Hook**
- File: `frontend/src/hooks/useProtectedFeature.js`
- Check isAuthenticated from useAuth
- Check user.profileComplete from useAuth
- Return: {canAccess: boolean, reason: string | null}
- Reasons: "signin_required", "profile_incomplete", null (can access)

**P2.5-T2: Create ProtectedFeature Wrapper Component**
- File: `frontend/src/components/Auth/ProtectedFeature.jsx`
- Props: children, requireCompleteProfile (default: false), onUnauthorized (callback)
- Use useProtectedFeature hook
- If canAccess: render children
- If !canAccess && reason === "signin_required": show SigninModal with message "Please sign in to access this feature"
- If !canAccess && reason === "profile_incomplete": show prompt "Complete your profile to unlock personalization"

**P2.5-T3: Wrap Personalization Button**
- File: Existing personalization button component (location TBD, likely in chapter pages)
- Wrap with ProtectedFeature component
- Pass requireCompleteProfile={true}
- On unauthorized: display signin modal or profile completion prompt based on reason

**Dependencies**: P2.1-T4 (Auth context), P2.1-T5 (useAuth hook), P2.2-T3 (SigninModal)
**Acceptance**: Unauthenticated users see signin prompt, authenticated users with incomplete profile see completion prompt, authenticated users with complete profile access feature

---

### Phase 2.6: Multi-Tab Synchronization & Session Management

**Goal**: Synchronize auth state across browser tabs and handle session expiry

**Prerequisites**: Phase 2.1 complete (Auth context, Token service)

**Tasks**:

**P2.6-T1: Implement Storage Event Listener**
- File: `frontend/src/context/AuthContext.jsx`
- In AuthProvider useEffect: add window.addEventListener('storage', handleStorageChange)
- handleStorageChange: if 'refreshToken' key removed → update state to unauthenticated
- If 'accessToken' key changed → reload user data from JWT claims
- Cleanup: removeEventListener on unmount

**P2.6-T2: Implement Session Expiry Warning**
- File: `frontend/src/context/AuthContext.jsx`
- On auth state change: schedule warning timer for 1 minute before access token expiry
- Use setTimeout based on JWT exp claim
- At 1 minute before expiry: show toast notification "Your session will expire in 1 minute"
- User can interact to trigger refresh, or ignore (auto-refresh on next API call)

**P2.6-T3: Handle Page Visibility for Token Refresh**
- File: `frontend/src/context/AuthContext.jsx`
- Add visibilitychange event listener
- When tab becomes visible after being hidden: check if access token expired
- If expired: trigger automatic refresh
- Purpose: Refresh token when user returns to tab after extended absence

**Dependencies**: P2.1-T2 (Token service), P2.1-T4 (Auth context), P2.4-T3 (Refresh function)
**Acceptance**: Logout in one tab signs out all tabs, session expiry warnings appear, returning to tab refreshes expired tokens

---

### Phase 2.7: Error Handling & User Feedback

**Goal**: Comprehensive error handling and user notifications

**Prerequisites**: Phase 2.1-2.6 complete (All components integrated)

**Tasks**:

**P2.7-T1: Create Toast Notification Component**
- File: `frontend/src/components/Auth/Toast.jsx`
- Display temporary success/error messages
- Props: message, type (success/error/warning), duration (default 3s)
- Auto-dismiss after duration
- Dismissible via close button
- Positioned: top-right corner, z-index above modals

**P2.7-T2: Integrate Toasts into Auth Context**
- File: `frontend/src/context/AuthContext.jsx`
- Add showToast(message, type) to context
- On signup success: showToast("Account created! Complete your profile for personalization", "success")
- On signin success: showToast("Welcome back, {name}!", "success")
- On signout success: showToast("Signed out successfully", "success")
- On session expiry: showToast("Your session has expired. Please sign in again", "warning")
- On network error: showToast("Network error. Please check your connection", "error")

**P2.7-T3: Standardize Error Messages**
- File: `frontend/src/utils/errorMessages.js`
- Map backend error codes to user-friendly messages
- 400: "Please check your input and try again"
- 401: "Invalid email or password"
- 409: "Email already registered. Please sign in or use a different email"
- 429: "Too many attempts. Please try again in 1 hour"
- 500: "Service temporarily unavailable. Please try again later"
- Network error: "Network error. Please check your internet connection"

**P2.7-T4: Add Loading States to Forms**
- Files: SignupModal.jsx, SigninModal.jsx
- During API call: disable all input fields, disable submit button, show spinner on button
- Use loading state from Auth context: authContext.loading
- Reset loading state on success or error

**Dependencies**: P2.2-T2, P2.2-T3 (Form components), P2.1-T4 (Auth context)
**Acceptance**: All auth operations show loading feedback, success/error toasts appear, error messages are user-friendly and actionable

---

### Phase 2.8: Testing & Integration Verification

**Goal**: End-to-end testing of complete auth flow

**Prerequisites**: Phase 2.1-2.7 complete (All features implemented)

**Tasks**:

**P2.8-T1: Manual Test Checklist**
- Test signup: new user registration, token storage, navbar update
- Test signin: existing user login, profile data loading, navbar update
- Test token refresh: wait 15 min or manually clear accessToken, verify auto-refresh on next API call
- Test logout: single tab, verify token cleared, navbar update
- Test multi-tab logout: logout in tab 1, verify tab 2 updates
- Test protected features: access when unauthenticated (signin prompt), access when authenticated without complete profile (profile prompt), access when authenticated with complete profile (feature executes)
- Test error scenarios: wrong password, duplicate email, network offline, backend down

**P2.8-T2: Verify Auth Backend Integration**
- Ensure auth backend running on localhost:3001
- Test all 4 endpoints via frontend UI (not just cURL)
- Verify CORS allows frontend origin (localhost:3000)
- Verify JWT tokens received have correct claims structure
- Verify token expiry times match backend config (15min access, 7d refresh)

**P2.8-T3: Browser Compatibility Test**
- Test in Chrome, Firefox, Safari, Edge
- Verify sessionStorage/localStorage work in all browsers
- Verify modals render correctly in all browsers
- Verify storage events fire in all browsers (multi-tab sync)

**P2.8-T4: Accessibility Check**
- Verify modals keyboard navigable (Tab, Escape)
- Verify form labels associated with inputs (screen reader support)
- Verify error messages announced to screen readers
- Verify focus management: modal opens → focus first input, modal closes → return focus to trigger button

**Dependencies**: All prior phases
**Acceptance**: All test scenarios pass, no console errors, auth flow works end-to-end across all supported browsers

---

## Phase Summary

| Phase | Description | Key Deliverables | Effort |
|-------|-------------|------------------|--------|
| Phase 0 | Research & Technical Validation | research.md (6 research tasks) | 2-3 hours |
| Phase 1 | Architecture & Design | data-model.md, contracts/auth-api.yaml, contracts/jwt-claims.md, quickstart.md | 2-3 hours |
| Phase 2.1 | Core Infrastructure | authAPI.js, tokenService.js, AuthContext.jsx, useAuth.js | 3-4 hours |
| Phase 2.2 | Authentication Forms | SignupModal.jsx, SigninModal.jsx, Modal.jsx, validators.js | 4-5 hours |
| Phase 2.3 | Navbar Integration | Swizzled Navbar, AuthButtons.jsx, UserDropdown.jsx | 3-4 hours |
| Phase 2.4 | Token Refresh | Axios interceptors, refresh logic, retry handling | 2-3 hours |
| Phase 2.5 | Protected Features | useProtectedFeature hook, ProtectedFeature wrapper | 2-3 hours |
| Phase 2.6 | Multi-Tab Sync | Storage events, session expiry warnings | 2-3 hours |
| Phase 2.7 | Error Handling | Toast component, error messages, loading states | 2-3 hours |
| Phase 2.8 | Testing | Manual test checklist, browser compatibility, accessibility | 3-4 hours |

**Total Effort**: 25-35 hours (3-5 working days)

---

## Dependencies & Sequencing

### Critical Path

Phase 0 → Phase 1 → Phase 2.1 → Phase 2.2 → Phase 2.3 → Phase 2.8

**Parallel Work Opportunities**:
- Phase 2.4 (Token Refresh) can start alongside Phase 2.3 (Navbar) once Phase 2.1 complete
- Phase 2.5 (Protected Features) can start alongside Phase 2.4 once Phase 2.1-2.2 complete
- Phase 2.6 (Multi-Tab Sync) can start alongside Phase 2.5 once Phase 2.1 complete
- Phase 2.7 (Error Handling) can be incremental throughout phases 2.2-2.6

### External Dependencies

- **Auth Backend**: Must be running and accessible (http://localhost:3001)
- **Database**: Migrations must be completed (10 tables created)
- **CORS**: Backend must allow frontend origin (configured in auth-backend/.env)
- **Docusaurus**: Existing site must be functional

---

## Risk Mitigation

### Technical Risks

**Risk**: Docusaurus theme swizzling breaks on Docusaurus updates
- **Mitigation**: Document swizzled components, test navbar after each Docusaurus upgrade, consider using Docusaurus plugin API in future

**Risk**: Token storage in localStorage vulnerable to XSS attacks
- **Mitigation**: Implement CSP headers, sanitize all user inputs, document security tradeoff, plan migration to httpOnly cookies for refresh tokens

**Risk**: Token refresh race condition (multiple 401s trigger multiple refresh calls)
- **Mitigation**: Implement promise queue pattern in interceptor (P2.4-T4)

### Integration Risks

**Risk**: Backend CORS not configured for frontend origin
- **Mitigation**: Test CORS early (Phase 2.8-T2), update auth-backend/.env if needed

**Risk**: JWT claims structure changes break frontend
- **Mitigation**: Document expected claims in contracts/jwt-claims.md, validate claims on token decode, graceful fallback for missing claims

### User Experience Risks

**Risk**: Users lose form data if token expires during signup
- **Mitigation**: Implement auto-refresh before form submission (check token expiry), save form state to sessionStorage

**Risk**: Confusing multi-tab behavior (signed out in one tab, unclear in others)
- **Mitigation**: Implement storage events (Phase 2.6-T1), show clear signout confirmation in all tabs

---

## File Manifest

### New Files Created (16 files)

**Services** (3 files):
1. `frontend/src/services/authAPI.js` - Axios client for auth endpoints
2. `frontend/src/services/tokenService.js` - JWT token storage/retrieval
3. `frontend/src/utils/jwtDecoder.js` - JWT parsing utility

**State Management** (2 files):
4. `frontend/src/context/AuthContext.jsx` - Global auth state provider
5. `frontend/src/hooks/useAuth.js` - Auth context consumer hook

**Components** (6 files):
6. `frontend/src/components/Auth/SignupModal.jsx` - Signup form modal
7. `frontend/src/components/Auth/SigninModal.jsx` - Signin form modal
8. `frontend/src/components/Auth/Modal.jsx` - Base modal component
9. `frontend/src/components/Auth/AuthButtons.jsx` - Navbar sign up/in buttons
10. `frontend/src/components/Auth/UserDropdown.jsx` - Authenticated user menu
11. `frontend/src/components/Auth/Toast.jsx` - Notification component
12. `frontend/src/components/Auth/ProtectedFeature.jsx` - Auth guard wrapper

**Hooks** (1 file):
13. `frontend/src/hooks/useProtectedFeature.js` - Protected feature access check

**Utilities** (2 files):
14. `frontend/src/utils/validators.js` - Email/password validation
15. `frontend/src/utils/errorMessages.js` - User-friendly error mapping

**Theme** (1 file):
16. `frontend/src/theme/Navbar/index.jsx` - Swizzled navbar with auth integration

**Documentation** (already in specs/005-frontend-auth-integration/):
- research.md (Phase 0 output)
- data-model.md (Phase 1 output)
- contracts/auth-api.yaml (Phase 1 output)
- contracts/jwt-claims.md (Phase 1 output)
- quickstart.md (Phase 1 output)

---

## Integration Points

### Backend APIs (Read-Only, No Modifications)

1. **POST /api/auth/signup**
   - Request: `{email, password, name}`
   - Response: `{success, data: {user, tokens}}`
   - Frontend action: Store tokens, update auth state, close modal

2. **POST /api/auth/signin**
   - Request: `{email, password}`
   - Response: `{success, data: {user, tokens}}`
   - Frontend action: Store tokens, store user profile, update auth state, close modal

3. **POST /api/auth/refresh**
   - Request: `{refreshToken}`
   - Response: `{success, data: {accessToken, expiresIn}}`
   - Frontend action: Update access token in sessionStorage, continue operation

4. **POST /api/auth/signout**
   - Request: `{refreshToken}`, Header: `Authorization: Bearer {accessToken}`
   - Response: `{success, message}`
   - Frontend action: Clear all tokens, update auth state, broadcast to tabs

### Existing Docusaurus Features (Preserve)

- Theme toggle (dark/light mode)
- Search functionality
- Sidebar navigation
- Mobile responsive menu
- Existing page routing
- RAG chatbot component
- Urdu translation feature
- Text selection popup

**Integration Strategy**: Auth components inject into navbar via swizzling, do not replace or interfere with existing features. Auth context wraps root app, accessible to all pages/components.

---

## Testing Strategy

### Manual Testing Scenarios

1. **Signup Flow**:
   - Invalid email format → client validation error before submit
   - Short password (<8 chars) → client validation error
   - Mismatched password confirmation → client validation error
   - Valid data → API call → 201 → tokens stored → navbar updates → toast shown
   - Duplicate email → API call → 409 → error message "Email already registered"

2. **Signin Flow**:
   - Invalid email format → client validation error
   - Wrong password → API call → 401 → error message "Invalid email or password"
   - Correct credentials → API call → 200 → tokens stored → profile loaded → navbar updates → toast shown
   - Account locked (5 failures) → API call → 429 → error message with retry guidance

3. **Token Refresh**:
   - Access token expired + valid refresh → auto-refresh on next API call → new token stored → request succeeds
   - Access token expired + invalid refresh → auto-refresh fails → user signed out → signin modal appears
   - Manual test: clear sessionStorage accessToken, make authenticated API call, verify refresh triggered

4. **Logout**:
   - Single tab logout → tokens cleared → navbar updates → toast shown
   - Multi-tab logout → tab 1 logout → tab 2 navbar updates within 1s

5. **Protected Features**:
   - Unauthenticated user clicks personalization → signin modal appears
   - Authenticated user (profileComplete: false) clicks personalization → profile completion prompt
   - Authenticated user (profileComplete: true) clicks personalization → feature executes

### Browser Testing Matrix

| Browser | Version | Signup | Signin | Refresh | Logout | Protected | Multi-Tab |
|---------|---------|--------|--------|---------|--------|-----------|-----------|
| Chrome | Latest | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Firefox | Latest | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Safari | Latest | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Edge | Latest | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

---

## Rollout Plan

### Development Phase (Week 1)
- Days 1-2: Phase 0 research + Phase 1 design (research.md, data-model.md, contracts/)
- Days 3-4: Phase 2.1-2.3 implementation (core infrastructure, forms, navbar)
- Day 5: Phase 2.4-2.7 implementation (token refresh, protected features, error handling, multi-tab sync)

### Testing Phase (Week 1, Day 5)
- Phase 2.8: Manual testing, browser compatibility, accessibility

### Deployment Phase (Week 2)
- Deploy frontend with auth integration
- Verify production auth backend URL configured
- Test end-to-end on production environment
- Monitor for auth-related errors (console logs, user reports)

---

## Success Validation

### Functional Validation

- [ ] Signup creates account, stores tokens, updates navbar
- [ ] Signin loads user, stores tokens, updates navbar with dropdown
- [ ] Token refresh works automatically on 401 responses
- [ ] Logout clears tokens, updates navbar, works across tabs
- [ ] Protected features check auth state correctly
- [ ] Profile completion check gates personalization
- [ ] All error messages are user-friendly
- [ ] Loading states appear during API calls
- [ ] Modals open/close correctly with keyboard support

### Performance Validation

- [ ] Token validation on page load < 500ms
- [ ] Signin API call < 3s (network dependent)
- [ ] UI state updates < 100ms after auth state change
- [ ] Token refresh transparent to user (no UI blocking)

### Security Validation

- [ ] Access tokens in sessionStorage (cleared on tab close)
- [ ] Refresh tokens in localStorage (persists across sessions)
- [ ] No tokens exposed in URL or console logs
- [ ] Authorization header includes Bearer token for protected calls
- [ ] Tokens cleared completely on logout
- [ ] XSS mitigation documented (CSP headers, input sanitization)

---

## Next Steps

After Phase 2 implementation complete:
1. Run `/sp.tasks` to generate granular task breakdown with acceptance criteria
2. Execute tasks incrementally, testing each component independently
3. Integrate components, test end-to-end flow
4. Deploy to staging environment for user acceptance testing
5. Collect feedback, iterate on UX (error messages, loading states, modal styling)

---

**Plan Status**: ✅ Complete and ready for task breakdown (/sp.tasks)
**Research Required**: Phase 0 (6 research tasks) - must complete before implementation
**Design Required**: Phase 1 (data models, API contracts) - must complete before coding
**Implementation**: Phase 2 (8 sub-phases, 25 tasks estimated)
**Total Timeline**: 3-5 working days for complete frontend auth integration
