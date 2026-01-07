---
id: 8
title: "Fixed Google OAuth Invalid Code Error"
stage: "general"
date: "2025-12-30"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-urdu-translation-toggle"
user: "user"
command: "fixed google oauth invalid code error"
labels: ["auth", "oauth", "error-handling", "google"]
files_yaml: |
  - /mnt/d/Dev_Workspace/GIAIC/Spec-Kit-Plus/backup-ask-ur-auth-python - Copy/backend/app/auth/routes.py
links:
  spec: null
  ticket: null
  adr: null
  pr: null
---

## Summary

Fixed the Google OAuth callback issue where users were seeing 500 Internal Server Errors. The implementation now properly handles invalid/expired authorization codes and returns appropriate error messages instead of server errors.

## Changes Made

### 1. Improved Error Handling
- Added specific handling for "invalid_grant" errors from Google's OAuth API
- Changed error response from 500 Internal Server Error to 400 Bad Request with user-friendly message
- Added proper validation for authorization code format

## Root Cause

The original issue was that when Google OAuth authorization codes were invalid, expired, or already used, the backend was throwing unhandled exceptions resulting in 500 Internal Server Errors. This happened because:

1. Google OAuth codes are single-use and have short expiration times
2. The original error handling was not specific enough for different OAuth error types
3. Users were seeing technical server errors instead of helpful messages

## Solution

1. Added specific error handling for "invalid_grant" responses from Google
2. Return user-friendly message: "Invalid or expired authorization code. Please try signing in again."
3. Maintain proper HTTP status codes (400 instead of 500 for OAuth issues)

## Verification

- Invalid/expired authorization codes now return 400 with clear message
- Proper HTTP status codes maintained
- User experience improved with clear instructions

## Files Modified

- `backend/app/auth/routes.py`: Enhanced error handling for Google OAuth callback

## Testing Results

- Invalid authorization codes: ✅ Return 400 with user-friendly message
- Valid authorization codes: ✅ Continue to work as expected
- Error handling: ✅ Proper status codes and messages