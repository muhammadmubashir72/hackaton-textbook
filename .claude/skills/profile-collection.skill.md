---
name: profile-collection
description: Collect and structure user software and hardware background information
trigger: Post-signup profile completion or profile update
inputs:
  - userId: string (required)
  - softwareBackground: object (required)
  - hardwareBackground: object (required)
outputs:
  - profileId: string
  - profileComplete: boolean
  - updatedAt: timestamp
software_background_schema:
  - programmingLanguages: array of strings
  - frameworks: array of strings
  - experienceLevel: enum (beginner, intermediate, advanced, expert)
  - specializations: array of strings
  - yearsOfExperience: number
hardware_background_schema:
  - familiarPlatforms: array of strings (Arduino, Raspberry Pi, ESP32, etc.)
  - roboticsExperience: enum (none, hobbyist, professional)
  - electronicsKnowledge: enum (none, basic, intermediate, advanced)
  - preferredTools: array of strings
validations:
  - Required fields present
  - Enum values within allowed ranges
  - Array items non-empty
---

# Profile Collection Skill

Structures and validates user background information for personalization purposes.

## Purpose
Collect, validate, and store structured software and hardware background data to enable content personalization.

## Software Background Schema
```json
{
  "programmingLanguages": ["Python", "JavaScript", "C++"],
  "frameworks": ["ROS2", "TensorFlow", "React"],
  "experienceLevel": "intermediate",
  "specializations": ["Robotics", "Computer Vision", "Web Development"],
  "yearsOfExperience": 3
}
```

## Hardware Background Schema
```json
{
  "familiarPlatforms": ["Raspberry Pi", "Arduino", "ESP32"],
  "roboticsExperience": "hobbyist",
  "electronicsKnowledge": "intermediate",
  "preferredTools": ["Gazebo", "Unity", "Fusion 360"]
}
```

## Workflow
1. Receive profile data from user
2. Validate against defined schemas
3. Check required fields present
4. Validate enum values
5. Sanitize input data
6. Store in database
7. Mark profile as complete
8. Trigger JWT refresh to include new claims
9. Return profile ID and completion status

## Integration Points
- Neon DB for persistent storage (profiles, software_backgrounds, hardware_backgrounds, user_preferences, audit_logs tables)
- Validation library (Zod/Joi) for schema validation
- JWT service to refresh tokens with new claims
- Redis for cache invalidation (profile:{userId}, personalization_context:{userId})
