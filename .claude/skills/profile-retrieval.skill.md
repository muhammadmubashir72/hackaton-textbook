---
name: profile-retrieval
description: Retrieve user profile and background data for personalization
trigger: Authenticated request requiring user context
inputs:
  - userId: string (required)
  - includeBackgrounds: boolean (default: true)
outputs:
  - userProfile: object
  - softwareBackground: object
  - hardwareBackground: object
  - profileComplete: boolean
access_control:
  - User can only access their own profile
  - Admin roles can access any profile
  - Service-to-service calls require API key
---

# Profile Retrieval Skill

Retrieves complete user profile including background data for content personalization.

## Purpose
Fetch user profile and structured background data with proper access control for personalization engines.

## Workflow
1. Validate userId from request context
2. Check access permissions (self-access or admin)
3. Retrieve user profile from database
4. If includeBackgrounds=true, fetch software and hardware backgrounds
5. Aggregate data into unified response
6. Return structured profile data

## Response Structure
```json
{
  "userProfile": {
    "id": "user_123",
    "email": "user@example.com",
    "name": "John Doe",
    "profileComplete": true,
    "createdAt": "2025-01-01T00:00:00Z"
  },
  "softwareBackground": {
    "programmingLanguages": ["Python", "JavaScript"],
    "frameworks": ["ROS2", "React"],
    "experienceLevel": "intermediate",
    "specializations": ["Robotics", "AI"],
    "yearsOfExperience": 3
  },
  "hardwareBackground": {
    "familiarPlatforms": ["Raspberry Pi", "Arduino"],
    "roboticsExperience": "hobbyist",
    "electronicsKnowledge": "intermediate",
    "preferredTools": ["Gazebo", "Unity"]
  }
}
```

## Integration Points
- Neon DB for profile queries (users, profiles, software_backgrounds, hardware_backgrounds tables with JOINs)
- Authorization guard for access control
- Redis for profile caching (profile:{userId} with 1h TTL)
- Neon DB authz_logs table for access tracking
