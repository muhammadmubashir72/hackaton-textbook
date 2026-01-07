---
name: personalization-context
purpose: Build user context for content personalization based on profile data
responsibilities:
  - Retrieve user profile and backgrounds
  - Build personalization context object
  - Cache user context for performance
  - Invalidate cache on profile updates
dependencies:
  - Profile manager agent
  - Cache service (Redis)
  - JWT service for user identification
context_structure:
  - userId: string
  - softwareExpertise: object
  - hardwareExpertise: object
  - recommendedContent: array
  - learningPath: object
interfaces:
  - getUserContext(userId)
  - refreshUserContext(userId)
  - invalidateContext(userId)
caching_strategy:
  - TTL: 1 hour
  - Invalidate on profile update
  - Lazy refresh on token refresh
---

# Personalization Context Agent

Builds and maintains user personalization context derived from profile and background data.

## Purpose
Transform user profile and background data into actionable personalization context for content recommendation, learning path customization, and adaptive UI.

## Responsibilities

### Context Building
1. Receive userId from authenticated request
2. Retrieve user profile from profile manager
3. Retrieve software background data
4. Retrieve hardware background data
5. Analyze expertise levels
6. Build personalization context object
7. Cache context with TTL
8. Return context

### Context Refresh
1. Receive userId
2. Invalidate existing cache
3. Rebuild context from latest profile data
4. Update cache
5. Emit context updated event
6. Return fresh context

### Cache Invalidation
1. Receive userId or profile update event
2. Delete cached context for userId
3. Log invalidation
4. Next request will rebuild context

### Context Analysis
1. Parse software background data
2. Determine programming language proficiency
3. Identify framework familiarity
4. Assess experience level
5. Parse hardware background data
6. Determine platform expertise
7. Assess robotics and electronics knowledge
8. Build expertise matrix

## Personalization Context Structure

### Complete Context Object
```json
{
  "userId": "user_123",
  "profileComplete": true,
  "expertise": {
    "software": {
      "languages": {
        "Python": "advanced",
        "JavaScript": "intermediate",
        "C++": "beginner"
      },
      "frameworks": {
        "ROS2": "intermediate",
        "TensorFlow": "beginner"
      },
      "overallLevel": "intermediate",
      "specializations": ["Robotics", "Computer Vision"]
    },
    "hardware": {
      "platforms": {
        "Raspberry Pi": "proficient",
        "Arduino": "proficient",
        "ESP32": "learning"
      },
      "roboticsExperience": "hobbyist",
      "electronicsKnowledge": "intermediate",
      "preferredTools": ["Gazebo", "Unity"]
    }
  },
  "learningPath": {
    "currentLevel": "intermediate",
    "suggestedNextTopics": [
      "Advanced ROS2 Navigation",
      "Computer Vision with OpenCV",
      "SLAM Algorithms"
    ],
    "skillGaps": [
      "Reinforcement Learning",
      "Advanced Electronics"
    ]
  },
  "contentPreferences": {
    "difficultySetting": "intermediate",
    "focusAreas": ["Robotics", "Computer Vision"],
    "preferredPlatforms": ["Raspberry Pi", "Arduino"],
    "showAdvancedTopics": false,
    "showBeginnerTopics": false
  },
  "recommendations": {
    "chapters": [
      {
        "id": "ros2-navigation",
        "title": "ROS2 Navigation Stack",
        "reason": "Matches intermediate level and ROS2 framework"
      },
      {
        "id": "opencv-basics",
        "title": "Computer Vision with OpenCV",
        "reason": "Aligns with Computer Vision specialization"
      }
    ],
    "projects": [
      {
        "id": "line-following-robot",
        "title": "Line Following Robot with ROS2",
        "reason": "Uses familiar platforms (Raspberry Pi) and frameworks (ROS2)"
      }
    ]
  },
  "uiAdaptations": {
    "showCodeExamples": true,
    "codeLanguage": "Python",
    "showHardwareSetup": true,
    "preferredPlatform": "Raspberry Pi",
    "hideBeginnerTips": false,
    "showAdvancedNotes": false
  },
  "metadata": {
    "createdAt": "2025-12-24T10:00:00Z",
    "lastUpdated": "2025-12-24T10:00:00Z",
    "cacheExpiry": "2025-12-24T11:00:00Z"
  }
}
```

## Context Building Algorithm

### Software Expertise Mapping
```javascript
function mapSoftwareExpertise(softwareBackground) {
  const languageProficiency = {};
  softwareBackground.programmingLanguages.forEach(lang => {
    languageProficiency[lang] = inferProficiency(
      softwareBackground.experienceLevel,
      softwareBackground.specializations
    );
  });

  return {
    languages: languageProficiency,
    frameworks: mapFrameworks(softwareBackground.frameworks),
    overallLevel: softwareBackground.experienceLevel,
    specializations: softwareBackground.specializations
  };
}
```

### Hardware Expertise Mapping
```javascript
function mapHardwareExpertise(hardwareBackground) {
  const platformProficiency = {};
  hardwareBackground.familiarPlatforms.forEach(platform => {
    platformProficiency[platform] = inferPlatformProficiency(
      hardwareBackground.roboticsExperience,
      hardwareBackground.electronicsKnowledge
    );
  });

  return {
    platforms: platformProficiency,
    roboticsExperience: hardwareBackground.roboticsExperience,
    electronicsKnowledge: hardwareBackground.electronicsKnowledge,
    preferredTools: hardwareBackground.preferredTools
  };
}
```

### Learning Path Generation
```javascript
function generateLearningPath(expertise) {
  const currentLevel = expertise.software.overallLevel;
  const specializations = expertise.software.specializations;

  // Identify skill gaps
  const allSkills = getRequiredSkills();
  const userSkills = [
    ...expertise.software.languages,
    ...expertise.software.frameworks
  ];
  const skillGaps = allSkills.filter(skill => !userSkills.includes(skill));

  // Suggest next topics based on current level and specializations
  const suggestedTopics = getRecommendedTopics(currentLevel, specializations);

  return {
    currentLevel,
    suggestedNextTopics: suggestedTopics,
    skillGaps
  };
}
```

### Content Recommendations
```javascript
function generateRecommendations(context) {
  // Find chapters matching user level and interests
  const matchingChapters = filterChapters({
    level: context.expertise.software.overallLevel,
    specializations: context.expertise.software.specializations,
    platforms: Object.keys(context.expertise.hardware.platforms)
  });

  // Find projects matching user expertise
  const matchingProjects = filterProjects({
    frameworks: context.expertise.software.frameworks,
    platforms: context.expertise.hardware.platforms
  });

  return {
    chapters: matchingChapters,
    projects: matchingProjects
  };
}
```

## Caching Strategy

### Cache Key Structure
- Key: `personalization_context:{userId}`
- TTL: 3600 seconds (1 hour)
- Storage: Redis

### Cache Invalidation Triggers
1. Profile update event
2. Background data update
3. Manual refresh request
4. Token refresh (lazy refresh)

### Cache Warming
- Pre-build context on profile completion
- Rebuild on first authenticated request
- Background refresh on token refresh

## Integration Points
- Profile manager for user data retrieval
- Redis for context caching (personalization_context:{userId} with 1h TTL)
- Neon DB for preference persistence (user_preferences table)
- Content service for recommendation data
- Event bus for profile update events
- Analytics service for usage tracking

## Performance Optimization
- Cache complete context object
- Use Redis pipeline for multi-key operations
- Implement lazy loading for recommendations
- Batch process context updates
- Monitor cache hit rate (target >80%)
