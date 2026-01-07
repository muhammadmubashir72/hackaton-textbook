# Implementation Tasks: Frontend Authentication Integration

**Branch**: `005-frontend-auth-integration` | **Date**: 2025-12-25 | **Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Overview

Comprehensive task breakdown for integrating frontend authentication UI with existing JWT backend (auth-backend on port 3001). Tasks cover navbar auth buttons, signup/signin forms, JWT token management, global auth state, automatic token refresh, and protected feature access.

**Total Tasks**: 37 implementation tasks across 10 phases
**Estimated Effort**: 25-35 hours (3-5 working days)
**Dependencies**: Auth backend running, database migrations complete

---

## Phase 0: Research & Technical Validation (2-3 hours)

### R0-T1: Research Docusaurus Theme Swizzling

**Goal**: Understand how to override Docusaurus navbar component

**Steps**:
1. Review Docusaurus swizzling documentation: https://docusaurus.io/docs/swizzling
2. Identify navbar component path: `@docusaurus/theme-classic/Navbar`
3. Test swizzle command: `npm run swizzle @docusaurus/theme-classic Navbar -- --eject`
4. Document:
   - File location after swizzle (`src/theme/Navbar/index.jsx`)
   - Component props and structure
   - How to preserve existing navbar features (logo, links, search, theme toggle)
   - Update compatibility implications

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T1)

**Acceptance Criteria**:
- [ ] Swizzling process documented with exact commands
- [ ] Component structure mapped
- [ ] Risks and compatibility notes included

---

### R0-T2: Research React Context for Auth State

**Goal**: Define global auth state management pattern

**Steps**:
1. Review React Context API best practices
2. Design state shape:
   ```javascript
   {
     isAuthenticated: boolean,
     user: { id, email, name, profileComplete } | null,
     loading: boolean,
     error: string | null
   }
   ```
3. Choose pattern: Context + useReducer vs Context + useState
4. Document performance considerations (memo, context splitting)
5. Define action types: SET_USER, SET_LOADING, SET_ERROR, CLEAR_USER

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T2)

**Acceptance Criteria**:
- [ ] State shape defined with TypeScript interface
- [ ] Chosen pattern documented with rationale
- [ ] Action types enumerated

---

### R0-T3: Research JWT Token Storage Strategy

**Goal**: Determine secure token storage approach

**Steps**:
1. Compare storage options:
   - sessionStorage: Cleared on tab close, XSS vulnerable
   - localStorage: Persists, XSS vulnerable
   - httpOnly cookies: XSS immune, CSRF vulnerable (requires backend support)
   - In-memory: Secure, lost on refresh
2. Document decision:
   - Access token: sessionStorage (balance security + UX)
   - Refresh token: localStorage (persistence needed)
3. List security tradeoffs and mitigations:
   - XSS risk: CSP headers (`script-src 'self'`), input sanitization
   - Future improvement: httpOnly cookies for refresh token
4. Document token lifecycle:
   - Store on signup/signin
   - Read on page load for auto-authentication
   - Clear on signout or refresh failure

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T3)

**Acceptance Criteria**:
- [ ] Storage strategy documented for both token types
- [ ] Security tradeoffs explicitly stated
- [ ] Mitigation measures listed

---

### R0-T4: Research Axios Interceptor Pattern

**Goal**: Design automatic token refresh on 401 responses

**Steps**:
1. Review axios interceptor documentation
2. Design request interceptor:
   - Add `Authorization: Bearer {accessToken}` header to all requests
   - Skip header for public endpoints (/signup, /signin)
3. Design response interceptor:
   - Catch 401 errors
   - Check if request already retried (`config._retry` flag)
   - If not retried: call refresh token endpoint
   - Update stored access token on success
   - Retry original request with new token
   - Sign out if refresh fails
4. Document retry logic:
   - Max 1 retry per request (prevent infinite loops)
   - Track retry state in request config object

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T4)

**Acceptance Criteria**:
- [ ] Request interceptor pattern documented
- [ ] Response interceptor flow diagram included
- [ ] Retry logic with max attempts specified

---

### R0-T5: Research Multi-Tab Auth Synchronization

**Goal**: Design cross-tab logout synchronization

**Steps**:
1. Review browser Storage API events: `window.addEventListener('storage', ...)`
2. Document event trigger conditions:
   - Fires when localStorage or sessionStorage changes in OTHER tabs (not same tab)
3. Design synchronization logic:
   - Listen for 'storage' event
   - Check if 'refreshToken' key removed → trigger signout in current tab
   - Check if 'accessToken' key changed → reload user data from new token
4. Handle edge case: User signs in on Tab 1, then Tab 2 → Tab 2 should detect new login

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T5)

**Acceptance Criteria**:
- [ ] Storage event listener pattern documented
- [ ] Logout synchronization flow explained
- [ ] Edge cases identified

---

### R0-T6: Research Modal Component Pattern

**Goal**: Design accessible modal for signup/signin forms

**Steps**:
1. Review React portal API: `ReactDOM.createPortal(child, container)`
2. Design modal structure:
   - Overlay: Semi-transparent backdrop, click to close
   - Modal card: Centered white box, close X button
   - Content: Form fields, submit button
3. Document accessibility requirements:
   - ARIA attributes: `role="dialog"`, `aria-modal="true"`, `aria-labelledby`
   - Focus management: Trap focus within modal, restore focus on close
   - Keyboard support: Escape key closes modal, Tab cycles through form fields
4. Handle body scroll lock (prevent scrolling page behind modal)

**Output**: `specs/005-frontend-auth-integration/research.md` (section: R0-T6)

**Acceptance Criteria**:
- [ ] Portal rendering approach documented
- [ ] ARIA accessibility checklist included
- [ ] Focus trap implementation notes

---

## Phase 1: Architecture & Design (2-3 hours)

### P1-T1: Define Data Models

**Goal**: Create TypeScript/JSDoc interfaces for auth state and API responses

**Steps**:
1. Create file: `specs/005-frontend-auth-integration/data-model.md`
2. Define interfaces:
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
3. Document state transitions:
   ```
   [Unauthenticated]
     → signup/signin → [Authenticated (Profile Incomplete)]
       → profile completion → [Authenticated (Profile Complete)]
         → signout → [Unauthenticated]
   ```

**Output**: `specs/005-frontend-auth-integration/data-model.md`

**Acceptance Criteria**:
- [ ] All interfaces defined with complete property types
- [ ] State transition diagram included
- [ ] Comments explain each field's purpose

---

### P1-T2: Document API Contracts

**Goal**: Create OpenAPI spec for backend auth endpoints

**Steps**:
1. Create file: `specs/005-frontend-auth-integration/contracts/auth-api.yaml`
2. Document 4 endpoints:
   - POST /api/auth/signup → {email, password, name} → 201 {user, tokens}
   - POST /api/auth/signin → {email, password} → 200 {user, tokens, profile}
   - POST /api/auth/refresh → {refreshToken} → 200 {accessToken, expiresIn}
   - POST /api/auth/signout → {refreshToken} + Authorization header → 200 {success}
3. Include request/response schemas
4. Document error responses: 400, 401, 409, 429, 500

**Output**: `specs/005-frontend-auth-integration/contracts/auth-api.yaml`

**Acceptance Criteria**:
- [ ] OpenAPI 3.0 valid YAML file
- [ ] All 4 endpoints fully documented
- [ ] Request body schemas include validation rules (min length, format)
- [ ] Error responses include user-facing error messages

---

### P1-T3: Document JWT Claims Structure

**Goal**: Specify expected JWT payload fields

**Steps**:
1. Create file: `specs/005-frontend-auth-integration/contracts/jwt-claims.md`
2. Document access token claims:
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
3. Explain frontend usage:
   - `sub` → user ID for API calls
   - `email`, `name` → display in navbar
   - `profileComplete` → gate personalization features
   - `softwareBackground[]`, `hardwareBackground[]` → UI hints (first 5 items)
   - `exp` → calculate token expiry, schedule refresh

**Output**: `specs/005-frontend-auth-integration/contracts/jwt-claims.md`

**Acceptance Criteria**:
- [ ] All JWT claims documented with types
- [ ] Frontend usage for each claim explained
- [ ] Expiry calculation example included

---

### P1-T4: Create Quickstart Guide

**Goal**: Developer onboarding document with step-by-step implementation

**Steps**:
1. Create file: `specs/005-frontend-auth-integration/quickstart.md`
2. Write 9-step implementation guide:
   - Step 1: Install dependencies (`axios`, `jwt-decode`)
   - Step 2: Create auth API client (`src/services/authAPI.js`)
   - Step 3: Create token service (`src/services/tokenService.js`)
   - Step 4: Create auth context (`src/context/AuthContext.jsx`)
   - Step 5: Swizzle navbar (`npm run swizzle @docusaurus/theme-classic Navbar`)
   - Step 6: Create auth components (SignupModal, SigninModal, UserDropdown)
   - Step 7: Add axios interceptor
   - Step 8: Protect personalization features
   - Step 9: Test all flows
3. Include time estimates per step (total ~70 minutes)

**Output**: `specs/005-frontend-auth-integration/quickstart.md`

**Acceptance Criteria**:
- [ ] All 9 steps clearly explained
- [ ] Code snippets for key configurations
- [ ] Time estimates provided
- [ ] Prerequisites section lists backend requirements

---

## Phase 2.1: Core Infrastructure (3-4 hours)

### P2.1-T1: Create Auth API Client

**Goal**: Axios instance for authentication endpoints

**File**: `frontend/src/services/authAPI.js`

**Steps**:
1. Install dependencies:
   ```bash
   cd frontend
   npm install axios
   ```
2. Create axios instance:
   ```javascript
   import axios from 'axios';

   const authAPI = axios.create({
     baseURL: process.env.REACT_APP_AUTH_API_URL || 'http://localhost:3001/api/auth',
     timeout: 5000,
     headers: {
       'Content-Type': 'application/json',
     },
   });

   export const signup = async (email, password, name) => {
     const response = await authAPI.post('/signup', { email, password, name });
     return response.data;
   };

   export const signin = async (email, password) => {
     const response = await authAPI.post('/signin', { email, password });
     return response.data;
   };

   export const refresh = async (refreshToken) => {
     const response = await authAPI.post('/refresh', { refreshToken });
     return response.data;
   };

   export const signout = async (accessToken, refreshToken) => {
     const response = await authAPI.post('/signout', { refreshToken }, {
       headers: { Authorization: `Bearer ${accessToken}` },
     });
     return response.data;
   };

   export default authAPI;
   ```
3. Create `.env` file in frontend root:
   ```
   REACT_APP_AUTH_API_URL=http://localhost:3001/api/auth
   ```

**Testing**:
- Import `authAPI` in test file
- Call `signup('test@example.com', 'password123', 'Test User')` with auth backend running
- Verify 201 response with user and tokens

**Acceptance Criteria**:
- [ ] Axios instance configured with base URL from env variable
- [ ] 4 methods exported: `signup`, `signin`, `refresh`, `signout`
- [ ] 5-second timeout configured
- [ ] Error handling preserves axios error structure

---

### P2.1-T2: Create Token Service

**Goal**: Secure token storage and retrieval

**File**: `frontend/src/services/tokenService.js`

**Steps**:
1. Install jwt-decode:
   ```bash
   npm install jwt-decode
   ```
2. Implement token service:
   ```javascript
   import jwtDecode from 'jwt-decode';

   const ACCESS_TOKEN_KEY = 'accessToken';
   const REFRESH_TOKEN_KEY = 'refreshToken';

   export const storeTokens = (tokens) => {
     sessionStorage.setItem(ACCESS_TOKEN_KEY, tokens.accessToken);
     localStorage.setItem(REFRESH_TOKEN_KEY, tokens.refreshToken);
   };

   export const getAccessToken = () => {
     return sessionStorage.getItem(ACCESS_TOKEN_KEY);
   };

   export const getRefreshToken = () => {
     return localStorage.getItem(REFRESH_TOKEN_KEY);
   };

   export const clearTokens = () => {
     sessionStorage.removeItem(ACCESS_TOKEN_KEY);
     localStorage.removeItem(REFRESH_TOKEN_KEY);
   };

   export const isTokenExpired = (token) => {
     if (!token) return true;
     try {
       const decoded = jwtDecode(token);
       const currentTime = Date.now() / 1000;
       return decoded.exp < currentTime;
     } catch (error) {
       return true;
     }
   };

   export const decodeToken = (token) => {
     if (!token) return null;
     try {
       return jwtDecode(token);
     } catch (error) {
       console.error('Failed to decode token:', error);
       return null;
     }
   };
   ```

**Testing**:
- Store mock tokens: `storeTokens({ accessToken: 'eyJhbGc...', refreshToken: 'eyJhbGc...' })`
- Verify `getAccessToken()` retrieves from sessionStorage
- Verify `getRefreshToken()` retrieves from localStorage
- Close tab, reopen, verify access token cleared but refresh token persists
- Test `isTokenExpired()` with expired token (manually set `exp` claim in past)

**Acceptance Criteria**:
- [ ] Access token stored in sessionStorage
- [ ] Refresh token stored in localStorage
- [ ] `isTokenExpired()` correctly checks JWT exp claim
- [ ] `decodeToken()` returns null on invalid token without throwing

---

### P2.1-T3: Create JWT Decoder Utility

**Goal**: Wrapper for jwt-decode with error handling

**File**: `frontend/src/utils/jwtDecoder.js`

**Steps**:
1. Create utility:
   ```javascript
   import jwtDecode from 'jwt-decode';

   export const decodeAccessToken = (token) => {
     if (!token) return null;
     try {
       const decoded = jwtDecode(token);
       return {
         userId: decoded.sub,
         email: decoded.email,
         name: decoded.name,
         profileComplete: decoded.profileComplete,
         softwareBackground: decoded.softwareBackground || [],
         hardwareBackground: decoded.hardwareBackground || [],
         roles: decoded.roles || ['user'],
         expiry: decoded.exp,
       };
     } catch (error) {
       console.error('Invalid JWT token:', error);
       return null;
     }
   };
   ```

**Testing**:
- Generate real JWT token from auth backend
- Decode with `decodeAccessToken(token)`
- Verify all fields extracted correctly
- Test with invalid token string → returns null

**Acceptance Criteria**:
- [ ] Returns structured object with all JWT claims
- [ ] Handles missing optional fields (softwareBackground, hardwareBackground)
- [ ] Returns null on decode failure
- [ ] No uncaught exceptions

---

### P2.1-T4: Create Auth Context Provider

**Goal**: Global auth state with React Context

**File**: `frontend/src/context/AuthContext.jsx`

**Steps**:
1. Create context and reducer:
   ```javascript
   import React, { createContext, useReducer, useEffect } from 'react';
   import { storeTokens, getAccessToken, getRefreshToken, clearTokens, isTokenExpired, decodeToken } from '../services/tokenService';
   import { signup as signupAPI, signin as signinAPI, refresh as refreshAPI, signout as signoutAPI } from '../services/authAPI';

   const AuthContext = createContext();

   const initialState = {
     isAuthenticated: false,
     user: null,
     loading: true,
     error: null,
   };

   const authReducer = (state, action) => {
     switch (action.type) {
       case 'SET_USER':
         return {
           ...state,
           isAuthenticated: true,
           user: action.payload,
           loading: false,
           error: null,
         };
       case 'SET_LOADING':
         return { ...state, loading: action.payload };
       case 'SET_ERROR':
         return { ...state, error: action.payload, loading: false };
       case 'CLEAR_USER':
         return { ...initialState, loading: false };
       default:
         return state;
     }
   };

   export const AuthProvider = ({ children }) => {
     const [state, dispatch] = useReducer(authReducer, initialState);

     // Initialize auth state on mount
     useEffect(() => {
       const initAuth = async () => {
         const accessToken = getAccessToken();
         const refreshToken = getRefreshToken();

         if (!accessToken || !refreshToken) {
           dispatch({ type: 'SET_LOADING', payload: false });
           return;
         }

         if (isTokenExpired(accessToken)) {
           // Try to refresh
           try {
             const data = await refreshAPI(refreshToken);
             storeTokens({ accessToken: data.data.accessToken, refreshToken });
             const user = decodeToken(data.data.accessToken);
             dispatch({ type: 'SET_USER', payload: user });
           } catch (error) {
             clearTokens();
             dispatch({ type: 'CLEAR_USER' });
           }
         } else {
           const user = decodeToken(accessToken);
           dispatch({ type: 'SET_USER', payload: user });
         }
       };

       initAuth();
     }, []);

     const signup = async (email, password, name) => {
       try {
         dispatch({ type: 'SET_LOADING', payload: true });
         const data = await signupAPI(email, password, name);
         storeTokens(data.data.tokens);
         const user = decodeToken(data.data.tokens.accessToken);
         dispatch({ type: 'SET_USER', payload: user });
       } catch (error) {
         dispatch({ type: 'SET_ERROR', payload: error.response?.data?.error || 'Signup failed' });
         throw error;
       }
     };

     const signin = async (email, password) => {
       try {
         dispatch({ type: 'SET_LOADING', payload: true });
         const data = await signinAPI(email, password);
         storeTokens(data.data.tokens);
         const user = decodeToken(data.data.tokens.accessToken);
         dispatch({ type: 'SET_USER', payload: user });
       } catch (error) {
         dispatch({ type: 'SET_ERROR', payload: error.response?.data?.error || 'Signin failed' });
         throw error;
       }
     };

     const logout = async () => {
       try {
         const accessToken = getAccessToken();
         const refreshToken = getRefreshToken();
         await signoutAPI(accessToken, refreshToken);
       } catch (error) {
         console.error('Signout API call failed:', error);
       } finally {
         clearTokens();
         dispatch({ type: 'CLEAR_USER' });
       }
     };

     const value = {
       ...state,
       signup,
       signin,
       logout,
       hasCompleteProfile: () => state.user?.profileComplete || false,
     };

     return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
   };

   export default AuthContext;
   ```

2. Wrap app with AuthProvider in `src/theme/Root.js`:
   ```javascript
   import React from 'react';
   import { AuthProvider } from '../context/AuthContext';

   export default function Root({ children }) {
     return <AuthProvider>{children}</AuthProvider>;
   }
   ```

**Testing**:
- Start frontend with provider
- Open React DevTools, verify AuthContext in component tree
- Store mock tokens in storage, refresh page
- Verify `state.isAuthenticated` becomes true
- Call `logout()`, verify tokens cleared and state reset

**Acceptance Criteria**:
- [ ] Context initialized with `loading: true`
- [ ] Auto-authentication on mount checks stored tokens
- [ ] Expired access token triggers automatic refresh
- [ ] `signup()`, `signin()`, `logout()` methods update state correctly
- [ ] Error state set on API failures

---

### P2.1-T5: Create useAuth Hook

**Goal**: Hook to consume auth context

**File**: `frontend/src/hooks/useAuth.js`

**Steps**:
1. Create hook:
   ```javascript
   import { useContext } from 'react';
   import AuthContext from '../context/AuthContext';

   export const useAuth = () => {
     const context = useContext(AuthContext);
     if (!context) {
       throw new Error('useAuth must be used within AuthProvider');
     }
     return context;
   };
   ```

**Testing**:
- Use in component: `const { isAuthenticated, user, signup } = useAuth();`
- Verify context values accessible
- Test outside provider → error thrown

**Acceptance Criteria**:
- [ ] Returns all context values
- [ ] Throws error if used outside provider
- [ ] TypeScript types inferred correctly

---

## Phase 2.2: Authentication Forms (4-5 hours)

### P2.2-T1: Create Validators

**Goal**: Client-side validation utilities

**File**: `frontend/src/utils/validators.js`

**Steps**:
1. Implement validators:
   ```javascript
   export const validateEmail = (email) => {
     const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
     return emailRegex.test(email);
   };

   export const validatePassword = (password) => {
     if (!password || password.length < 8) {
       return { valid: false, message: 'Password must be at least 8 characters' };
     }
     return { valid: true, message: '' };
   };

   export const validatePasswordMatch = (password, confirmation) => {
     return password === confirmation;
   };

   export const validateName = (name) => {
     if (!name || name.trim().length === 0) {
       return { valid: false, message: 'Name is required' };
     }
     return { valid: true, message: '' };
   };
   ```

**Testing**:
- Test `validateEmail('invalid')` → false
- Test `validateEmail('test@example.com')` → true
- Test `validatePassword('short')` → `{ valid: false, message: '...' }`
- Test `validatePassword('longpassword123')` → `{ valid: true, message: '' }`

**Acceptance Criteria**:
- [ ] Email validator uses standard regex
- [ ] Password validator checks minimum 8 characters
- [ ] Password match validator returns boolean
- [ ] Name validator checks non-empty string

---

### P2.2-T2: Create Signup Modal

**Goal**: Signup form with validation and API integration

**File**: `frontend/src/components/Auth/SignupModal.jsx`

**Steps**:
1. Create modal component:
   ```javascript
   import React, { useState } from 'react';
   import { useAuth } from '../../hooks/useAuth';
   import { validateEmail, validatePassword, validatePasswordMatch, validateName } from '../../utils/validators';
   import Modal from './Modal';

   const SignupModal = ({ isOpen, onClose }) => {
     const { signup, loading, error } = useAuth();
     const [formData, setFormData] = useState({
       email: '',
       password: '',
       passwordConfirmation: '',
       name: '',
     });
     const [errors, setErrors] = useState({});

     const handleChange = (e) => {
       setFormData({ ...formData, [e.target.name]: e.target.value });
       setErrors({ ...errors, [e.target.name]: '' });
     };

     const validateForm = () => {
       const newErrors = {};

       if (!validateEmail(formData.email)) {
         newErrors.email = 'Invalid email format';
       }

       const passwordValidation = validatePassword(formData.password);
       if (!passwordValidation.valid) {
         newErrors.password = passwordValidation.message;
       }

       if (!validatePasswordMatch(formData.password, formData.passwordConfirmation)) {
         newErrors.passwordConfirmation = 'Passwords do not match';
       }

       const nameValidation = validateName(formData.name);
       if (!nameValidation.valid) {
         newErrors.name = nameValidation.message;
       }

       setErrors(newErrors);
       return Object.keys(newErrors).length === 0;
     };

     const handleSubmit = async (e) => {
       e.preventDefault();
       if (!validateForm()) return;

       try {
         await signup(formData.email, formData.password, formData.name);
         onClose();
         // Show success toast (Phase 2.7)
       } catch (err) {
         // Error handled by context
       }
     };

     return (
       <Modal isOpen={isOpen} onClose={onClose} title="Sign Up">
         <form onSubmit={handleSubmit}>
           <div>
             <label htmlFor="name">Name</label>
             <input
               type="text"
               id="name"
               name="name"
               value={formData.name}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.name && <span className="error">{errors.name}</span>}
           </div>

           <div>
             <label htmlFor="email">Email</label>
             <input
               type="email"
               id="email"
               name="email"
               value={formData.email}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.email && <span className="error">{errors.email}</span>}
           </div>

           <div>
             <label htmlFor="password">Password</label>
             <input
               type="password"
               id="password"
               name="password"
               value={formData.password}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.password && <span className="error">{errors.password}</span>}
           </div>

           <div>
             <label htmlFor="passwordConfirmation">Confirm Password</label>
             <input
               type="password"
               id="passwordConfirmation"
               name="passwordConfirmation"
               value={formData.passwordConfirmation}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.passwordConfirmation && <span className="error">{errors.passwordConfirmation}</span>}
           </div>

           {error && <div className="error">{error}</div>}

           <button type="submit" disabled={loading}>
             {loading ? 'Creating Account...' : 'Sign Up'}
           </button>
         </form>
       </Modal>
     );
   };

   export default SignupModal;
   ```

2. Add CSS (inline or separate file):
   ```css
   .error {
     color: red;
     font-size: 0.875rem;
     margin-top: 0.25rem;
   }
   ```

**Testing**:
- Open modal, submit empty form → validation errors appear
- Enter invalid email → email error appears
- Enter short password → password error appears
- Enter mismatched confirmation → confirmation error appears
- Enter valid data → form submits, tokens stored, modal closes

**Acceptance Criteria**:
- [ ] All fields render with labels
- [ ] Client-side validation runs before submit
- [ ] Loading state disables inputs and button
- [ ] Success closes modal and updates auth state
- [ ] Backend errors display in form

---

### P2.2-T3: Create Signin Modal

**Goal**: Signin form with validation

**File**: `frontend/src/components/Auth/SigninModal.jsx`

**Steps**:
1. Create component (similar structure to SignupModal):
   ```javascript
   import React, { useState } from 'react';
   import { useAuth } from '../../hooks/useAuth';
   import { validateEmail } from '../../utils/validators';
   import Modal from './Modal';

   const SigninModal = ({ isOpen, onClose }) => {
     const { signin, loading, error } = useAuth();
     const [formData, setFormData] = useState({ email: '', password: '' });
     const [errors, setErrors] = useState({});

     const handleChange = (e) => {
       setFormData({ ...formData, [e.target.name]: e.target.value });
       setErrors({ ...errors, [e.target.name]: '' });
     };

     const validateForm = () => {
       const newErrors = {};
       if (!validateEmail(formData.email)) {
         newErrors.email = 'Invalid email format';
       }
       if (!formData.password) {
         newErrors.password = 'Password is required';
       }
       setErrors(newErrors);
       return Object.keys(newErrors).length === 0;
     };

     const handleSubmit = async (e) => {
       e.preventDefault();
       if (!validateForm()) return;

       try {
         await signin(formData.email, formData.password);
         onClose();
       } catch (err) {
         // Preserve email, clear password on error
         setFormData({ ...formData, password: '' });
       }
     };

     return (
       <Modal isOpen={isOpen} onClose={onClose} title="Sign In">
         <form onSubmit={handleSubmit}>
           <div>
             <label htmlFor="email">Email</label>
             <input
               type="email"
               id="email"
               name="email"
               value={formData.email}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.email && <span className="error">{errors.email}</span>}
           </div>

           <div>
             <label htmlFor="password">Password</label>
             <input
               type="password"
               id="password"
               name="password"
               value={formData.password}
               onChange={handleChange}
               disabled={loading}
               required
             />
             {errors.password && <span className="error">{errors.password}</span>}
           </div>

           {error && <div className="error">{error}</div>}

           <button type="submit" disabled={loading}>
             {loading ? 'Signing In...' : 'Sign In'}
           </button>

           <a href="#" className="forgot-password" disabled>
             Forgot Password? (Coming soon)
           </a>
         </form>
       </Modal>
     );
   };

   export default SigninModal;
   ```

**Testing**:
- Open modal, submit empty → validation errors
- Enter wrong password → backend error appears, password field cleared
- Enter correct credentials → signin succeeds, modal closes

**Acceptance Criteria**:
- [ ] Email and password fields render
- [ ] Validation runs before submit
- [ ] Wrong credentials show error message
- [ ] Account locked (429) shows specific message
- [ ] Success closes modal and updates navbar

---

### P2.2-T4: Create Base Modal Component

**Goal**: Reusable modal with accessibility

**File**: `frontend/src/components/Auth/Modal.jsx`

**Steps**:
1. Create modal using React portal:
   ```javascript
   import React, { useEffect, useRef } from 'react';
   import { createPortal } from 'react-dom';

   const Modal = ({ isOpen, onClose, title, children }) => {
     const modalRef = useRef();
     const closeButtonRef = useRef();

     useEffect(() => {
       if (isOpen) {
         // Focus first input or close button
         const firstInput = modalRef.current?.querySelector('input');
         if (firstInput) {
           firstInput.focus();
         } else {
           closeButtonRef.current?.focus();
         }

         // Lock body scroll
         document.body.style.overflow = 'hidden';

         // Handle Escape key
         const handleEscape = (e) => {
           if (e.key === 'Escape') onClose();
         };
         document.addEventListener('keydown', handleEscape);

         return () => {
           document.body.style.overflow = '';
           document.removeEventListener('keydown', handleEscape);
         };
       }
     }, [isOpen, onClose]);

     if (!isOpen) return null;

     const handleOverlayClick = (e) => {
       if (e.target === e.currentTarget) onClose();
     };

     return createPortal(
       <div className="modal-overlay" onClick={handleOverlayClick} role="dialog" aria-modal="true" aria-labelledby="modal-title">
         <div className="modal-content" ref={modalRef}>
           <div className="modal-header">
             <h2 id="modal-title">{title}</h2>
             <button ref={closeButtonRef} onClick={onClose} aria-label="Close modal">
               ×
             </button>
           </div>
           <div className="modal-body">{children}</div>
         </div>
       </div>,
       document.body
     );
   };

   export default Modal;
   ```

2. Add CSS:
   ```css
   .modal-overlay {
     position: fixed;
     top: 0;
     left: 0;
     right: 0;
     bottom: 0;
     background: rgba(0, 0, 0, 0.5);
     display: flex;
     align-items: center;
     justify-content: center;
     z-index: 9999;
   }

   .modal-content {
     background: white;
     border-radius: 8px;
     max-width: 500px;
     width: 90%;
     padding: 1.5rem;
     max-height: 90vh;
     overflow-y: auto;
   }

   .modal-header {
     display: flex;
     justify-content: space-between;
     align-items: center;
     margin-bottom: 1rem;
   }

   .modal-header button {
     background: none;
     border: none;
     font-size: 1.5rem;
     cursor: pointer;
   }
   ```

**Testing**:
- Open modal → first input focused
- Press Escape → modal closes
- Click overlay → modal closes
- Click inside modal → modal stays open
- Check with screen reader → dialog role announced

**Acceptance Criteria**:
- [ ] Renders to document.body via portal
- [ ] Focus traps within modal (Tab cycles through form)
- [ ] Escape key closes modal
- [ ] Overlay click closes modal
- [ ] ARIA attributes set correctly

---

## Phase 2.3: Navbar Integration (3-4 hours)

### P2.3-T1: Swizzle Docusaurus Navbar

**Goal**: Override navbar component

**Steps**:
1. Run swizzle command:
   ```bash
   cd frontend
   npm run swizzle @docusaurus/theme-classic Navbar -- --eject
   ```
2. Verify file created at `src/theme/Navbar/index.jsx`
3. Confirm existing navbar features work (logo, links, search, theme toggle)

**Testing**:
- Start dev server: `npm start`
- Verify navbar renders with all original features
- Click logo → navigates home
- Use search → works
- Toggle theme → works

**Acceptance Criteria**:
- [ ] Navbar component copied to `src/theme/Navbar/index.jsx`
- [ ] No visual regressions (navbar looks identical)
- [ ] All existing features functional

---

### P2.3-T2: Create Auth Buttons Component

**Goal**: Sign Up / Sign In buttons for navbar

**File**: `frontend/src/components/Auth/AuthButtons.jsx`

**Steps**:
1. Create component:
   ```javascript
   import React, { useState } from 'react';
   import SignupModal from './SignupModal';
   import SigninModal from './SigninModal';

   const AuthButtons = () => {
     const [signupOpen, setSignupOpen] = useState(false);
     const [signinOpen, setSigninOpen] = useState(false);

     return (
       <>
         <button className="auth-button primary" onClick={() => setSignupOpen(true)}>
           Sign Up
         </button>
         <button className="auth-button secondary" onClick={() => setSigninOpen(true)}>
           Sign In
         </button>

         <SignupModal isOpen={signupOpen} onClose={() => setSignupOpen(false)} />
         <SigninModal isOpen={signinOpen} onClose={() => setSigninOpen(false)} />
       </>
     );
   };

   export default AuthButtons;
   ```

2. Add CSS:
   ```css
   .auth-button {
     padding: 0.5rem 1rem;
     border: none;
     border-radius: 4px;
     cursor: pointer;
     margin-left: 0.5rem;
   }

   .auth-button.primary {
     background: #007bff;
     color: white;
   }

   .auth-button.secondary {
     background: transparent;
     color: #007bff;
     border: 1px solid #007bff;
   }
   ```

**Testing**:
- Render `<AuthButtons />` in isolation
- Click "Sign Up" → modal opens
- Close modal → modal disappears
- Click "Sign In" → modal opens

**Acceptance Criteria**:
- [ ] Two buttons render with correct labels
- [ ] Buttons open respective modals
- [ ] Modals close independently
- [ ] Styles match Docusaurus theme

---

### P2.3-T3: Create User Dropdown Component

**Goal**: Authenticated user menu

**File**: `frontend/src/components/Auth/UserDropdown.jsx`

**Steps**:
1. Create component:
   ```javascript
   import React, { useState, useRef, useEffect } from 'react';
   import { useAuth } from '../../hooks/useAuth';

   const UserDropdown = () => {
     const { user, logout, hasCompleteProfile } = useAuth();
     const [dropdownOpen, setDropdownOpen] = useState(false);
     const dropdownRef = useRef();

     useEffect(() => {
       const handleClickOutside = (e) => {
         if (dropdownRef.current && !dropdownRef.current.contains(e.target)) {
           setDropdownOpen(false);
         }
       };
       document.addEventListener('mousedown', handleClickOutside);
       return () => document.removeEventListener('mousedown', handleClickOutside);
     }, []);

     const handleLogout = async () => {
       await logout();
       setDropdownOpen(false);
     };

     return (
       <div className="user-dropdown" ref={dropdownRef}>
         <button className="user-button" onClick={() => setDropdownOpen(!dropdownOpen)}>
           {user?.name || user?.email}
           {!hasCompleteProfile() && <span className="badge">!</span>}
         </button>

         {dropdownOpen && (
           <div className="dropdown-menu">
             <a href="/profile">Profile</a>
             <button onClick={handleLogout}>Logout</button>
           </div>
         )}
       </div>
     );
   };

   export default UserDropdown;
   ```

2. Add CSS:
   ```css
   .user-dropdown {
     position: relative;
   }

   .user-button {
     background: transparent;
     border: none;
     cursor: pointer;
     display: flex;
     align-items: center;
   }

   .badge {
     background: red;
     color: white;
     border-radius: 50%;
     width: 16px;
     height: 16px;
     display: inline-flex;
     align-items: center;
     justify-content: center;
     margin-left: 0.5rem;
     font-size: 0.75rem;
   }

   .dropdown-menu {
     position: absolute;
     top: 100%;
     right: 0;
     background: white;
     border: 1px solid #ccc;
     border-radius: 4px;
     box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
     min-width: 150px;
     margin-top: 0.5rem;
   }

   .dropdown-menu a,
   .dropdown-menu button {
     display: block;
     width: 100%;
     text-align: left;
     padding: 0.75rem 1rem;
     border: none;
     background: none;
     cursor: pointer;
     text-decoration: none;
     color: inherit;
   }

   .dropdown-menu a:hover,
   .dropdown-menu button:hover {
     background: #f5f5f5;
   }
   ```

**Testing**:
- Render with authenticated user
- Click user name → dropdown opens
- Click outside → dropdown closes
- Click "Logout" → user signed out, navbar updates
- Verify badge appears when profileComplete: false

**Acceptance Criteria**:
- [ ] Displays user name (or email as fallback)
- [ ] Dropdown opens on click
- [ ] Dropdown closes on outside click
- [ ] Logout button calls auth context logout
- [ ] Profile completion badge appears when needed

---

### P2.3-T4: Integrate Auth Components into Navbar

**Goal**: Add auth buttons/dropdown to swizzled navbar

**File**: `frontend/src/theme/Navbar/index.jsx`

**Steps**:
1. Import auth components and hook:
   ```javascript
   import { useAuth } from '../../hooks/useAuth';
   import AuthButtons from '../../components/Auth/AuthButtons';
   import UserDropdown from '../../components/Auth/UserDropdown';
   ```

2. Find navbar right section (usually near theme toggle)

3. Add conditional rendering:
   ```javascript
   function Navbar() {
     const { isAuthenticated, loading } = useAuth();

     // ... existing navbar code

     return (
       <nav className="navbar">
         {/* ... existing logo, links */}

         <div className="navbar__right">
           {/* ... existing search, theme toggle */}

           {!loading && (
             <>
               {isAuthenticated ? <UserDropdown /> : <AuthButtons />}
             </>
           )}
         </div>
       </nav>
     );
   }
   ```

**Testing**:
- Start with no tokens stored → navbar shows Sign Up / Sign In
- Sign up → navbar updates to show user dropdown
- Refresh page → navbar shows dropdown (auto-authenticated)
- Logout → navbar shows Sign Up / Sign In

**Acceptance Criteria**:
- [ ] Unauthenticated state shows AuthButtons
- [ ] Authenticated state shows UserDropdown
- [ ] Loading state doesn't flicker between states
- [ ] Navbar updates immediately on auth state change
- [ ] Existing navbar features not affected

---

## Phase 2.4: Automatic Token Refresh (2-3 hours)

### P2.4-T1: Add Request Interceptor

**Goal**: Inject Authorization header

**File**: `frontend/src/services/authAPI.js` (modify existing)

**Steps**:
1. Add request interceptor to axios instance:
   ```javascript
   import { getAccessToken } from './tokenService';

   authAPI.interceptors.request.use(
     (config) => {
       // Skip auth header for public endpoints
       const publicEndpoints = ['/signup', '/signin'];
       const isPublic = publicEndpoints.some((endpoint) => config.url.includes(endpoint));

       if (!isPublic) {
         const accessToken = getAccessToken();
         if (accessToken) {
           config.headers.Authorization = `Bearer ${accessToken}`;
         }
       }

       return config;
     },
     (error) => Promise.reject(error)
   );
   ```

**Testing**:
- Store access token
- Make authenticated API call (e.g., fetch user profile)
- Inspect request in Network tab → verify `Authorization: Bearer ...` header present
- Call signup endpoint → verify no Authorization header

**Acceptance Criteria**:
- [ ] Authorization header added to all requests except /signup, /signin
- [ ] Header format: `Bearer {accessToken}`
- [ ] No header added if token not found

---

### P2.4-T2: Add Response Interceptor for 401

**Goal**: Auto-refresh on 401 errors

**File**: `frontend/src/services/authAPI.js` (modify existing)

**Steps**:
1. Add response interceptor:
   ```javascript
   import { getRefreshToken, storeTokens, clearTokens } from './tokenService';
   import { refresh as refreshAPI } from './authAPI'; // circular import - handle carefully

   let isRefreshing = false;
   let failedQueue = [];

   const processQueue = (error, token = null) => {
     failedQueue.forEach((prom) => {
       if (error) {
         prom.reject(error);
       } else {
         prom.resolve(token);
       }
     });
     failedQueue = [];
   };

   authAPI.interceptors.response.use(
     (response) => response,
     async (error) => {
       const originalRequest = error.config;

       // Check if 401 and not already retried
       if (error.response?.status === 401 && !originalRequest._retry) {
         if (isRefreshing) {
           // Another request is already refreshing, wait for it
           return new Promise((resolve, reject) => {
             failedQueue.push({ resolve, reject });
           })
             .then((token) => {
               originalRequest.headers.Authorization = `Bearer ${token}`;
               return authAPI(originalRequest);
             })
             .catch((err) => Promise.reject(err));
         }

         originalRequest._retry = true;
         isRefreshing = true;

         const refreshToken = getRefreshToken();
         if (!refreshToken) {
           clearTokens();
           window.location.href = '/'; // Redirect to home
           return Promise.reject(error);
         }

         try {
           const data = await refreshAPI(refreshToken);
           const newAccessToken = data.data.accessToken;
           storeTokens({ accessToken: newAccessToken, refreshToken });

           authAPI.defaults.headers.common.Authorization = `Bearer ${newAccessToken}`;
           originalRequest.headers.Authorization = `Bearer ${newAccessToken}`;

           processQueue(null, newAccessToken);
           isRefreshing = false;

           return authAPI(originalRequest);
         } catch (refreshError) {
           processQueue(refreshError, null);
           isRefreshing = false;
           clearTokens();
           window.location.href = '/';
           return Promise.reject(refreshError);
         }
       }

       return Promise.reject(error);
     }
   );
   ```

**Testing**:
- Manually clear access token from sessionStorage (keep refresh token)
- Make authenticated API call
- Verify interceptor catches 401
- Verify refresh token called
- Verify new access token stored
- Verify original request retried with new token
- Test concurrent requests → only one refresh call made

**Acceptance Criteria**:
- [ ] 401 responses trigger token refresh
- [ ] Original request retried with new access token
- [ ] Refresh failure signs user out and redirects
- [ ] Max 1 retry per request (prevents infinite loops)
- [ ] Concurrent 401s handled with promise queue

---

### P2.4-T3: Implement Refresh Token Function

**Goal**: Call refresh endpoint

**File**: `frontend/src/services/tokenService.js` (modify existing)

**Steps**:
1. Add refresh function (already implemented in P2.4-T2 via authAPI.refresh)

2. Ensure refresh endpoint exists in authAPI.js:
   ```javascript
   export const refresh = async (refreshToken) => {
     const response = await authAPI.post('/refresh', { refreshToken });
     return response.data;
   };
   ```

**Testing**:
- Call `refresh(validRefreshToken)`
- Verify 200 response with new access token
- Call with invalid token → verify 401 error

**Acceptance Criteria**:
- [ ] POST /api/auth/refresh with refresh token
- [ ] Returns new access token on success
- [ ] Throws error on invalid token

---

### P2.4-T4: Handle Concurrent Refresh Requests

**Goal**: Prevent multiple simultaneous refresh calls

**Implementation**: Already included in P2.4-T2 with `isRefreshing` flag and `failedQueue`

**Testing**:
1. Clear access token
2. Make 3 API calls simultaneously
3. Verify only 1 refresh call made (check Network tab)
4. Verify all 3 original requests retried with new token

**Acceptance Criteria**:
- [ ] First 401 triggers refresh
- [ ] Subsequent 401s wait for first refresh
- [ ] All waiting requests retry after refresh completes
- [ ] No duplicate refresh calls

---

## Phase 2.5: Protected Features (2-3 hours)

### P2.5-T1: Create useProtectedFeature Hook

**Goal**: Check auth + profile completion

**File**: `frontend/src/hooks/useProtectedFeature.js`

**Steps**:
1. Create hook:
   ```javascript
   import { useAuth } from './useAuth';

   export const useProtectedFeature = () => {
     const { isAuthenticated, user } = useAuth();

     if (!isAuthenticated) {
       return {
         canAccess: false,
         reason: 'signin_required',
       };
     }

     if (!user?.profileComplete) {
       return {
         canAccess: false,
         reason: 'profile_incomplete',
       };
     }

     return {
       canAccess: true,
       reason: null,
     };
   };
   ```

**Testing**:
- Use hook when not authenticated → returns `{ canAccess: false, reason: 'signin_required' }`
- Use hook when authenticated but profileComplete: false → returns `{ canAccess: false, reason: 'profile_incomplete' }`
- Use hook when authenticated + profileComplete: true → returns `{ canAccess: true, reason: null }`

**Acceptance Criteria**:
- [ ] Returns access status and reason
- [ ] Checks both authentication and profile completion
- [ ] Reason codes: 'signin_required', 'profile_incomplete', null

---

### P2.5-T2: Create ProtectedFeature Wrapper

**Goal**: HOC/wrapper for protected UI elements

**File**: `frontend/src/components/Auth/ProtectedFeature.jsx`

**Steps**:
1. Create component:
   ```javascript
   import React, { useState } from 'react';
   import { useProtectedFeature } from '../../hooks/useProtectedFeature';
   import SigninModal from './SigninModal';

   const ProtectedFeature = ({ children, requireCompleteProfile = false, onUnauthorized }) => {
     const { canAccess, reason } = useProtectedFeature();
     const [signinOpen, setSigninOpen] = useState(false);

     const handleClick = (e) => {
       if (!canAccess) {
         e.preventDefault();
         e.stopPropagation();

         if (reason === 'signin_required') {
           setSigninOpen(true);
         } else if (reason === 'profile_incomplete') {
           alert('Please complete your profile to access this feature.');
           // Or show profile completion modal
         }

         if (onUnauthorized) {
           onUnauthorized(reason);
         }
       }
     };

     return (
       <>
         <div onClick={handleClick}>
           {children}
         </div>
         <SigninModal isOpen={signinOpen} onClose={() => setSigninOpen(false)} />
       </>
     );
   };

   export default ProtectedFeature;
   ```

**Testing**:
- Wrap button with `<ProtectedFeature>`
- Click when not authenticated → signin modal opens
- Click when authenticated but profile incomplete → alert appears
- Click when authenticated + complete profile → button action executes

**Acceptance Criteria**:
- [ ] Wraps children without visual changes
- [ ] Intercepts clicks when canAccess: false
- [ ] Opens signin modal for unauthenticated users
- [ ] Shows profile prompt for incomplete profiles
- [ ] Allows click through when authorized

---

### P2.5-T3: Wrap Personalization Button

**Goal**: Protect personalization feature

**File**: Existing personalization button component (location TBD - likely in chapter pages or ChatKit)

**Steps**:
1. Find personalization button component
2. Import ProtectedFeature:
   ```javascript
   import ProtectedFeature from '../../components/Auth/ProtectedFeature';
   ```

3. Wrap button:
   ```javascript
   <ProtectedFeature requireCompleteProfile={true}>
     <button onClick={handlePersonalization}>
       Personalize This Chapter
     </button>
   </ProtectedFeature>
   ```

**Testing**:
- Click personalization when not signed in → signin modal appears
- Sign in with incomplete profile → profile completion prompt appears
- Complete profile → personalization feature executes

**Acceptance Criteria**:
- [ ] Personalization button wrapped with ProtectedFeature
- [ ] Requires complete profile (requireCompleteProfile={true})
- [ ] Unauthenticated users redirected to signin
- [ ] Incomplete profiles prompted to complete profile

---

## Phase 2.6: Multi-Tab Synchronization (2-3 hours)

### P2.6-T1: Implement Storage Event Listener

**Goal**: Sync logout across tabs

**File**: `frontend/src/context/AuthContext.jsx` (modify existing)

**Steps**:
1. Add storage event listener in AuthProvider:
   ```javascript
   useEffect(() => {
     const handleStorageChange = (e) => {
       if (e.key === 'refreshToken' && e.newValue === null) {
         // Refresh token removed (logout in another tab)
         dispatch({ type: 'CLEAR_USER' });
       } else if (e.key === 'accessToken' && e.newValue) {
         // Access token updated (signin in another tab)
         const user = decodeToken(e.newValue);
         if (user) {
           dispatch({ type: 'SET_USER', payload: user });
         }
       }
     };

     window.addEventListener('storage', handleStorageChange);
     return () => window.removeEventListener('storage', handleStorageChange);
   }, []);
   ```

**Testing**:
- Open app in two tabs (Tab 1, Tab 2)
- Sign in on Tab 1
- Verify Tab 2 navbar updates to authenticated state
- Logout on Tab 2
- Verify Tab 1 navbar updates to unauthenticated state

**Acceptance Criteria**:
- [ ] Storage event listener registered
- [ ] Logout in one tab signs out all tabs
- [ ] Signin in one tab updates all tabs
- [ ] Event only fires for storage changes in OTHER tabs

---

### P2.6-T2: Implement Session Expiry Warning

**Goal**: Warn user 1 min before token expires

**File**: `frontend/src/context/AuthContext.jsx` (modify existing)

**Steps**:
1. Add expiry warning logic:
   ```javascript
   useEffect(() => {
     if (!state.isAuthenticated || !state.user) return;

     const accessToken = getAccessToken();
     if (!accessToken) return;

     const decoded = decodeToken(accessToken);
     if (!decoded) return;

     const expiryTime = decoded.exp * 1000; // Convert to milliseconds
     const currentTime = Date.now();
     const timeUntilExpiry = expiryTime - currentTime;
     const warningTime = timeUntilExpiry - 60000; // 1 minute before

     if (warningTime > 0) {
       const warningTimer = setTimeout(() => {
         alert('Your session will expire in 1 minute. Please save your work.');
         // Or show toast notification (Phase 2.7)
       }, warningTime);

       return () => clearTimeout(warningTimer);
     }
   }, [state.isAuthenticated, state.user]);
   ```

**Testing**:
- Sign in with backend configured for short access token expiry (e.g., 2 minutes)
- Wait 1 minute
- Verify warning appears 1 minute before expiry
- Make API call after warning → token auto-refreshes

**Acceptance Criteria**:
- [ ] Warning appears 1 minute before access token expiry
- [ ] Warning only shows once per session
- [ ] Timer cleared on signout or token refresh

---

### P2.6-T3: Handle Page Visibility for Token Refresh

**Goal**: Refresh expired token when tab regains focus

**File**: `frontend/src/context/AuthContext.jsx` (modify existing)

**Steps**:
1. Add visibility change listener:
   ```javascript
   useEffect(() => {
     const handleVisibilityChange = async () => {
       if (document.visibilityState === 'visible' && state.isAuthenticated) {
         const accessToken = getAccessToken();
         const refreshToken = getRefreshToken();

         if (accessToken && isTokenExpired(accessToken) && refreshToken) {
           try {
             const data = await refreshAPI(refreshToken);
             storeTokens({ accessToken: data.data.accessToken, refreshToken });
             const user = decodeToken(data.data.accessToken);
             dispatch({ type: 'SET_USER', payload: user });
           } catch (error) {
             clearTokens();
             dispatch({ type: 'CLEAR_USER' });
           }
         }
       }
     };

     document.addEventListener('visibilitychange', handleVisibilityChange);
     return () => document.removeEventListener('visibilitychange', handleVisibilityChange);
   }, [state.isAuthenticated]);
   ```

**Testing**:
- Sign in
- Switch to another tab/app for 20 minutes (token expires)
- Switch back to app tab
- Verify token auto-refreshes
- Verify no 401 errors on next API call

**Acceptance Criteria**:
- [ ] Visibility change listener registered
- [ ] Expired token refreshed when tab becomes visible
- [ ] Refresh failure signs user out
- [ ] No refresh if token not expired

---

## Phase 2.7: Error Handling & User Feedback (2-3 hours)

### P2.7-T1: Create Toast Notification Component

**Goal**: User-friendly notifications

**File**: `frontend/src/components/Auth/Toast.jsx`

**Steps**:
1. Create toast component:
   ```javascript
   import React, { useEffect } from 'react';

   const Toast = ({ message, type = 'success', duration = 3000, onClose }) => {
     useEffect(() => {
       const timer = setTimeout(onClose, duration);
       return () => clearTimeout(timer);
     }, [duration, onClose]);

     const typeStyles = {
       success: { background: '#4caf50', color: 'white' },
       error: { background: '#f44336', color: 'white' },
       warning: { background: '#ff9800', color: 'white' },
     };

     return (
       <div className="toast" style={typeStyles[type]}>
         <span>{message}</span>
         <button onClick={onClose}>×</button>
       </div>
     );
   };

   export default Toast;
   ```

2. Add CSS:
   ```css
   .toast {
     position: fixed;
     top: 20px;
     right: 20px;
     padding: 1rem 1.5rem;
     border-radius: 4px;
     box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
     z-index: 10000;
     display: flex;
     align-items: center;
     gap: 1rem;
   }

   .toast button {
     background: none;
     border: none;
     color: inherit;
     font-size: 1.5rem;
     cursor: pointer;
   }
   ```

**Testing**:
- Render `<Toast message="Success!" type="success" onClose={...} />`
- Verify toast appears in top-right corner
- Wait 3 seconds → toast auto-dismisses
- Click X button → toast dismisses immediately

**Acceptance Criteria**:
- [ ] Toast renders with message and type styling
- [ ] Auto-dismisses after duration
- [ ] Manual close via X button
- [ ] Positioned above all other elements (z-index)

---

### P2.7-T2: Integrate Toasts into Auth Context

**Goal**: Show toasts for auth events

**File**: `frontend/src/context/AuthContext.jsx` (modify existing)

**Steps**:
1. Add toast state:
   ```javascript
   const [toast, setToast] = useState(null);

   const showToast = (message, type) => {
     setToast({ message, type });
   };

   const hideToast = () => {
     setToast(null);
   };
   ```

2. Update auth methods to show toasts:
   ```javascript
   const signup = async (email, password, name) => {
     try {
       // ... existing signup logic
       showToast('Account created! Complete your profile for personalization.', 'success');
     } catch (error) {
       showToast(error.response?.data?.error || 'Signup failed', 'error');
       throw error;
     }
   };

   const signin = async (email, password) => {
     try {
       // ... existing signin logic
       showToast(`Welcome back, ${user.name}!`, 'success');
     } catch (error) {
       showToast(error.response?.data?.error || 'Signin failed', 'error');
       throw error;
     }
   };

   const logout = async () => {
     // ... existing logout logic
     showToast('Signed out successfully', 'success');
   };
   ```

3. Render toast in provider:
   ```javascript
   return (
     <AuthContext.Provider value={value}>
       {children}
       {toast && <Toast message={toast.message} type={toast.type} onClose={hideToast} />}
     </AuthContext.Provider>
   );
   ```

**Testing**:
- Sign up → success toast appears
- Sign in → welcome toast appears
- Logout → signout toast appears
- Enter wrong password → error toast appears

**Acceptance Criteria**:
- [ ] Success toasts for signup, signin, logout
- [ ] Error toasts for failures
- [ ] Session expiry warning toast
- [ ] Network error toast

---

### P2.7-T3: Standardize Error Messages

**Goal**: User-friendly error mapping

**File**: `frontend/src/utils/errorMessages.js`

**Steps**:
1. Create error message map:
   ```javascript
   export const getErrorMessage = (error) => {
     if (!error.response) {
       return 'Network error. Please check your internet connection.';
     }

     const status = error.response.status;
     const backendMessage = error.response.data?.error;

     const statusMessages = {
       400: 'Please check your input and try again.',
       401: backendMessage || 'Invalid email or password.',
       409: 'Email already registered. Please sign in or use a different email.',
       429: 'Too many attempts. Please try again in 1 hour.',
       500: 'Service temporarily unavailable. Please try again later.',
     };

     return statusMessages[status] || 'An unexpected error occurred. Please try again.';
   };
   ```

2. Use in auth context:
   ```javascript
   import { getErrorMessage } from '../utils/errorMessages';

   const signup = async (email, password, name) => {
     try {
       // ... signup logic
     } catch (error) {
       const message = getErrorMessage(error);
       showToast(message, 'error');
       throw error;
     }
   };
   ```

**Testing**:
- Trigger each error status (400, 401, 409, 429, 500)
- Verify correct user-facing message appears
- Test with network offline → network error message

**Acceptance Criteria**:
- [ ] All HTTP status codes mapped to messages
- [ ] Network errors handled separately
- [ ] Messages are actionable and user-friendly
- [ ] Backend error messages preserved when available

---

### P2.7-T4: Add Loading States to Forms

**Goal**: Visual feedback during API calls

**File**: `frontend/src/components/Auth/SignupModal.jsx`, `SigninModal.jsx` (modify existing)

**Steps**:
1. Already implemented in P2.2-T2 and P2.2-T3 (loading state from useAuth hook)

2. Enhance with spinner:
   ```javascript
   <button type="submit" disabled={loading}>
     {loading ? (
       <>
         <span className="spinner"></span>
         Creating Account...
       </>
     ) : (
       'Sign Up'
     )}
   </button>
   ```

3. Add spinner CSS:
   ```css
   .spinner {
     display: inline-block;
     width: 14px;
     height: 14px;
     border: 2px solid rgba(255, 255, 255, 0.3);
     border-top-color: white;
     border-radius: 50%;
     animation: spin 0.6s linear infinite;
     margin-right: 0.5rem;
   }

   @keyframes spin {
     to { transform: rotate(360deg); }
   }
   ```

**Testing**:
- Submit signup form
- Verify button shows spinner and "Creating Account..." text
- Verify all inputs disabled during loading
- Verify loading state clears on success or error

**Acceptance Criteria**:
- [ ] Loading spinner appears during API calls
- [ ] Submit button disabled during loading
- [ ] Input fields disabled during loading
- [ ] Loading text describes current action

---

## Phase 2.8: Testing & Integration Verification (3-4 hours)

### P2.8-T1: Manual Test Checklist

**Goal**: Comprehensive end-to-end testing

**Steps**:
1. Test Signup Flow:
   - [ ] Open signup modal from navbar
   - [ ] Submit empty form → validation errors appear
   - [ ] Enter invalid email → email error
   - [ ] Enter short password → password error
   - [ ] Enter mismatched confirmation → confirmation error
   - [ ] Enter valid data → form submits
   - [ ] Verify loading state during submission
   - [ ] Verify success toast appears
   - [ ] Verify tokens stored (check sessionStorage, localStorage)
   - [ ] Verify navbar updates to show user dropdown
   - [ ] Try duplicate email → 409 error message

2. Test Signin Flow:
   - [ ] Open signin modal from navbar
   - [ ] Enter wrong password → 401 error, password cleared
   - [ ] Enter correct credentials → signin succeeds
   - [ ] Verify welcome toast with user name
   - [ ] Verify navbar shows user dropdown
   - [ ] Test account lock (5 failed attempts) → 429 error

3. Test Token Refresh:
   - [ ] Sign in successfully
   - [ ] Manually clear access token from sessionStorage
   - [ ] Make authenticated API call (e.g., fetch profile)
   - [ ] Verify 401 caught by interceptor
   - [ ] Verify refresh token called
   - [ ] Verify new access token stored
   - [ ] Verify original request retried
   - [ ] Make multiple concurrent calls → only 1 refresh

4. Test Logout:
   - [ ] Click logout in dropdown
   - [ ] Verify signout API called
   - [ ] Verify tokens cleared from storage
   - [ ] Verify navbar shows Sign Up / Sign In
   - [ ] Verify toast "Signed out successfully"

5. Test Multi-Tab Sync:
   - [ ] Open two tabs (Tab 1, Tab 2)
   - [ ] Sign in on Tab 1
   - [ ] Verify Tab 2 updates to authenticated state
   - [ ] Logout on Tab 2
   - [ ] Verify Tab 1 updates to unauthenticated state

6. Test Protected Features:
   - [ ] Not authenticated → click personalization → signin modal
   - [ ] Sign in with incomplete profile → click personalization → profile prompt
   - [ ] Complete profile → click personalization → feature executes

7. Test Error Scenarios:
   - [ ] Disconnect network → submit signup → network error
   - [ ] Stop auth backend → submit signin → 500 error
   - [ ] Invalid refresh token → auto-signout

8. Test Session Persistence:
   - [ ] Sign in
   - [ ] Close browser (all tabs)
   - [ ] Reopen site
   - [ ] Verify still authenticated (auto-login via refresh token)

**Acceptance Criteria**:
- [ ] All 8 test categories pass
- [ ] No console errors
- [ ] No visual bugs
- [ ] Toasts appear for all user actions

---

### P2.8-T2: Verify Backend Integration

**Goal**: Confirm frontend-backend contract

**Steps**:
1. Start auth backend: `cd auth-backend && npm start`
2. Verify backend on http://localhost:3001
3. Test all 4 endpoints via frontend:
   - Signup → POST /api/auth/signup
   - Signin → POST /api/auth/signin
   - Token refresh → POST /api/auth/refresh (via interceptor)
   - Signout → POST /api/auth/signout
4. Verify CORS allows frontend origin (http://localhost:3000)
5. Verify JWT tokens:
   - Access token stored in sessionStorage
   - Refresh token stored in localStorage
   - Decoded access token has all expected claims (sub, email, name, profileComplete, backgrounds, roles)
6. Verify token expiry:
   - Access token expires in 15 minutes
   - Refresh token expires in 7 days

**Acceptance Criteria**:
- [ ] All 4 backend endpoints respond correctly
- [ ] CORS configured properly (no preflight errors)
- [ ] JWT claims match expected structure
- [ ] Token expiry times match backend config

---

### P2.8-T3: Browser Compatibility Test

**Goal**: Cross-browser testing

**Steps**:
1. Test in Chrome (latest):
   - [ ] All auth flows work
   - [ ] Modals render correctly
   - [ ] Storage events fire (multi-tab sync)
2. Test in Firefox (latest):
   - [ ] All auth flows work
   - [ ] Modals render correctly
   - [ ] Storage events fire
3. Test in Safari (latest):
   - [ ] All auth flows work
   - [ ] Modals render correctly
   - [ ] Storage events fire
4. Test in Edge (latest):
   - [ ] All auth flows work
   - [ ] Modals render correctly
   - [ ] Storage events fire

**Acceptance Criteria**:
- [ ] All browsers render UI correctly
- [ ] No browser-specific console errors
- [ ] sessionStorage and localStorage work in all browsers
- [ ] Storage events work across tabs in all browsers

---

### P2.8-T4: Accessibility Check

**Goal**: Keyboard and screen reader support

**Steps**:
1. Keyboard Navigation:
   - [ ] Tab through navbar → auth buttons receive focus
   - [ ] Press Enter on Sign Up → modal opens
   - [ ] Tab through form fields → focus moves correctly
   - [ ] Press Escape → modal closes
   - [ ] Focus returns to trigger button after modal close
2. Screen Reader (NVDA/JAWS/VoiceOver):
   - [ ] Modal announced as dialog
   - [ ] Form labels associated with inputs
   - [ ] Error messages announced on validation
   - [ ] Success toasts announced
3. Focus Trap:
   - [ ] Tab cycles within modal (doesn't escape to page behind)
   - [ ] Shift+Tab cycles backward
4. ARIA Attributes:
   - [ ] Modal has `role="dialog"`, `aria-modal="true"`, `aria-labelledby`
   - [ ] Form inputs have `aria-describedby` for error messages
   - [ ] Buttons have `aria-label` where needed

**Acceptance Criteria**:
- [ ] Full keyboard navigation support
- [ ] Screen reader announces all UI changes
- [ ] Focus managed correctly (trap in modal, restore on close)
- [ ] ARIA attributes present and correct

---

## Summary

**Total Tasks**: 37 implementation tasks
**Phases**: 10 (Research → Design → 8 Implementation Phases)
**Files Created**: 16 new files (services, components, hooks, utils, theme)
**Files Modified**: 3 (AuthContext, authAPI, Navbar)

**Dependencies**:
- Auth backend running on localhost:3001
- Database migrations completed
- CORS configured for localhost:3000

**Critical Path**: Phase 0 → Phase 1 → Phase 2.1 → Phase 2.2 → Phase 2.3 → Phase 2.8

**Parallel Work**: Phases 2.4-2.7 can overlap after Phase 2.1 complete

**Next Steps After Completion**:
1. User acceptance testing
2. Collect feedback on error messages and UX
3. Plan profile completion UI (background collection deferred from signup)
4. Consider security enhancements (httpOnly cookies for refresh token)

---

**Tasks Ready for Implementation** ✅
