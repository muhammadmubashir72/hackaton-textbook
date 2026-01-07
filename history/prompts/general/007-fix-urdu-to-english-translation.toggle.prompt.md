---
id: "007"
title: "Fix Urdu to English translation toggle"
stage: "fix"
date: "2025-12-30"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-urdu-translation-toggle"
user: "User"
command: "Fix translation"
labels: ["translation","bugfix","backend"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "backend/app/api_main.py"
tests:
  - "None"
---

## Prompt

urdu k bad english return nh ho rha or translation bh work nh kr rha check kro

## Response snapshot

Found the bug in backend/app/api_main.py at translate_endpoint() function:

**Problem:** Lines 397-403 had incorrect logic:
```python
if is_urdu_text(request.text):
    # If the text is already in Urdu, return it as-is without translation
    return TranslationResponse(...)
```

This returned original Urdu text even when target_language was "en", preventing Urdu→English translation.

**Fix Applied:** Changed to:
```python
is_currently_urdu = is_urdu_text(request.text)
if is_currently_urdu and request.target_language == "ur":
    # Only skip if text is Urdu AND target is also Urdu
    return TranslationResponse(...)
```

Now:
- Urdu → Urdu: Returns original (no translation needed)
- English → Urdu: Translates to Urdu
- Urdu → English: Translates to English ✓

## Outcome

- ✅ Impact: Fixed translation toggle - users can now switch between Urdu and English
- 🧪 Tests: None run
- 📁 Files: backend/app/api_main.py (1 file modified)
- 🔁 Next prompts: Test the complete translation flow
- 🧠 Reflection: Bug was in backend logic - frontend code was correct

## Evaluation notes (flywheel)

- Failure modes observed: Backend returned original text when target was English
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Verify translation works end-to-end
