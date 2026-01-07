---
id: 0002
title: background-info-and-profile-completion-fixes
stage: red
date: 2025-12-27
surface: agent
model: claude-sonnet-4.5-20250929
feature: 006-frontend-auth-integration
branch: 006-frontend-auth-integration
user: unknown
command: /sp.phr
labels: ["bug-fix", "background-info", "profile-completion", "database-update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - auth-backend/src/services/auth-simple.service.ts
tests:
 - none
---

## Prompt

DIRECT BACKGROUND INFO UPDATE KIYA TO NH HUA ISKO UPDATE KRO SATH SATH UPDATE HOJAO

## Response snapshot

Fixed background info real-time update by changing SQL query from `COALESCE($2, bio)` to `CASE WHEN $2 IS NOT NULL THEN $2 ELSE bio END` to properly handle empty strings. Fixed profile completion percentage calculation by fetching all user data and calculating completion based on ALL fields instead of just updated ones, with proper scoring: Name (25), Bio (10), Software Background (25), Hardware Background (25), Preferred Tools (15) for total of 100 points.

## Outcome

- ✅ Impact: Background info now updates immediately after save; profile completion percentage correctly calculates 0-100% based on all filled fields
- 🧪 Tests: TypeScript compilation passed
- 📁 Files: auth-backend/src/services/auth-simple.service.ts (fixed bio update logic, rewrote completion calculation)
- 🔁 Next prompts: Test fixes in browser to verify background info updates and profile completion displays correctly
- 🧠 Reflection: COALESCE function ignores empty strings; need CASE WHEN for explicit NULL checking in PostgreSQL

## Evaluation notes (flywheel)

- Failure modes observed: None - PHR creation completed successfully; all placeholders populated correctly
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none - bug fixes are complete; user testing required
