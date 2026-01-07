---
name: profile-manager
purpose: Manage user profile and background data collection and storage
responsibilities:
  - Collect structured software and hardware background data
  - Validate profile data against schema
  - Store profile data securely
  - Retrieve profile data for personalization
  - Update profile completion status
dependencies:
  - Database service (PostgreSQL/MongoDB)
  - Validation library
  - Encryption service for sensitive data
storage_schema:
  - users table: id, email, name, passwordHash, createdAt, updatedAt
  - profiles table: userId, profileComplete, createdAt, updatedAt
  - software_backgrounds table: profileId, languages, frameworks, experienceLevel, specializations, yearsOfExperience
  - hardware_backgrounds table: profileId, platforms, roboticsExperience, electronicsKnowledge, tools
interfaces:
  - createProfile(userId, data)
  - updateProfile(userId, data)
  - getProfile(userId)
  - markProfileComplete(userId)
---

# Profile Manager Agent

Manages user profile lifecycle including structured background data collection and retrieval.

## Purpose
Central authority for user profile and background data management with validation, storage, and retrieval capabilities.

## Responsibilities

### Profile Creation
1. Receive userId after user signup
2. Create empty profile record
3. Set profileComplete to false
4. Initialize empty software_background record
5. Initialize empty hardware_background record
6. Return profileId

### Profile Update
1. Receive userId and profile data
2. Validate data against schema
3. Update profile fields
4. If backgrounds provided, validate and update
5. Check if profile meets completion criteria
6. Update profileComplete status
7. Emit profile updated event
8. Return updated profile

### Profile Retrieval
1. Receive userId
2. Query user record
3. Join profile data
4. Join software_background data
5. Join hardware_background data
6. Aggregate into unified response
7. Cache result with TTL
8. Return complete profile

### Profile Completion Check
1. Check user record exists
2. Check profile record exists
3. Validate software background has required fields
4. Validate hardware background has required fields
5. Mark profileComplete = true if all criteria met
6. Trigger JWT refresh event
7. Return completion status

## Database Schema

### users table
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255),
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);
```

### profiles table
```sql
CREATE TABLE profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  profile_complete BOOLEAN DEFAULT FALSE,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(user_id)
);
```

### software_backgrounds table
```sql
CREATE TABLE software_backgrounds (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  profile_id UUID REFERENCES profiles(id) ON DELETE CASCADE,
  programming_languages TEXT[] DEFAULT '{}',
  frameworks TEXT[] DEFAULT '{}',
  experience_level VARCHAR(50) CHECK (experience_level IN ('beginner', 'intermediate', 'advanced', 'expert')),
  specializations TEXT[] DEFAULT '{}',
  years_of_experience INTEGER,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(profile_id)
);
```

### hardware_backgrounds table
```sql
CREATE TABLE hardware_backgrounds (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  profile_id UUID REFERENCES profiles(id) ON DELETE CASCADE,
  familiar_platforms TEXT[] DEFAULT '{}',
  robotics_experience VARCHAR(50) CHECK (robotics_experience IN ('none', 'hobbyist', 'professional')),
  electronics_knowledge VARCHAR(50) CHECK (electronics_knowledge IN ('none', 'basic', 'intermediate', 'advanced')),
  preferred_tools TEXT[] DEFAULT '{}',
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(profile_id)
);
```

## Validation Schema

### Software Background Validation
```javascript
{
  programmingLanguages: {
    type: 'array',
    items: { type: 'string' },
    minItems: 1,
    required: true
  },
  frameworks: {
    type: 'array',
    items: { type: 'string' }
  },
  experienceLevel: {
    type: 'string',
    enum: ['beginner', 'intermediate', 'advanced', 'expert'],
    required: true
  },
  specializations: {
    type: 'array',
    items: { type: 'string' }
  },
  yearsOfExperience: {
    type: 'number',
    min: 0,
    max: 50
  }
}
```

### Hardware Background Validation
```javascript
{
  familiarPlatforms: {
    type: 'array',
    items: { type: 'string' },
    minItems: 1,
    required: true
  },
  roboticsExperience: {
    type: 'string',
    enum: ['none', 'hobbyist', 'professional'],
    required: true
  },
  electronicsKnowledge: {
    type: 'string',
    enum: ['none', 'basic', 'intermediate', 'advanced'],
    required: true
  },
  preferredTools: {
    type: 'array',
    items: { type: 'string' }
  }
}
```

## Profile Completion Criteria
- User record exists
- Profile record exists
- Software background has at least:
  - 1 programming language
  - Experience level set
- Hardware background has:
  - At least 1 familiar platform
  - Robotics experience set
  - Electronics knowledge set

## Integration Points
- Neon DB (PostgreSQL-compatible) for data persistence (users, profiles, software_backgrounds, hardware_backgrounds, user_preferences, audit_logs tables)
- @neondatabase/serverless driver for connection pooling and serverless optimization
- Validation library (Zod/Joi) for schema validation
- Redis for profile caching (profile:{userId} with 1h TTL)
- Event bus for profile lifecycle events
- JWT service for token refresh triggers
- Neon DB audit_logs table for comprehensive data change tracking

## Caching Strategy
- Cache complete profiles for 1 hour
- Invalidate on profile update
- Cache key: `profile:{userId}`
- Include all joined data in cache
