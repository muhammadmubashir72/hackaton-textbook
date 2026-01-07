---
id: "008"
title: "Fix Google OAuth URL in SignupModal"
stage: "fix"
date: "2025-12-30"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-urdu-translation-toggle"
user: "User"
command: "Fix Google Signup URL"
labels: ["auth","google","oauth","bugfix"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "frontend/src/components/Auth/SignupModal.jsx"
  - "frontend/package.json"
tests:
  - "None"
---

## Prompt

sign up with google me ye arha GET http://localhost:8000/favicon.ico 404 (Not Found) {
  "auth_url": "https://accounts.google.com/o/oauth2/auth?client_id=...&redirect_uri=http://localhost:3000/auth-callback&response_type=code&scope=openid email profile&access_type=offline&prompt=consent"
}
http://localhost:8000/api/auth/oauth/google/url

## Response snapshot

**Problem:** SignupModal was calling wrong backend URL (3001 instead of 8000)

**Issues found:**
1. SignupModal.jsx line 249 had fallback `http://localhost:3001` instead of `8000`
2. Environment variables might not be generating env.js file

**Fixes applied:**
1. Updated SignupModal to use correct fallback URL and better code structure
2. Added `postinstall` script to package.json to generate env.js automatically

## Outcome

- ✅ Impact: Google OAuth will now call correct backend URL (8000)
- 🧪 Tests: None run
- 📁 Files: SignupModal.jsx, package.json modified
- 🔁 Next prompts: Run `node generate-env.js` to create env.js file
- 🧠 Reflection: Issue was frontend hardcoded fallback URL, not backend

## Evaluation notes (flywheel)

- Failure modes observed: Wrong port 3001 used for backend API calls
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run frontend with env.js properly generated
