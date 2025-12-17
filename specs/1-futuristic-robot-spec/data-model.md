# Data Model: Futuristic Robot Visualization System

**Feature**: 1-futuristic-robot-spec
**Created**: 2025-12-14
**Status**: Complete

## Entity: RobotConfiguration

**Description**: Configuration parameters for the futuristic robot visualization

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| id | string | Yes | UUID format | Unique identifier for this configuration |
| appearance | RobotAppearance | Yes | See RobotAppearance entity | Visual appearance settings |
| animationParams | AnimationParameters | Yes | See AnimationParameters entity | Animation behavior settings |
| accessibilityOptions | AccessibilitySettings | No | See AccessibilitySettings entity | User preference settings |
| createdAt | Date | Yes | ISO 8601 format | Timestamp of configuration creation |
| updatedAt | Date | No | ISO 8601 format | Timestamp of last modification |

## Entity: RobotAppearance

**Description**: Visual appearance properties of the robot

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| bodyColor | string | Yes | Hex color code (#RRGGBB) | Primary body color (default: #00FFFF for cyan blue) |
| coreColor | string | Yes | Hex color code (#RRGGBB) | Core/eye color (default: #FFA500 for orange) |
| particleColors | ParticleColors | Yes | See ParticleColors entity | Colors for different particle types |
| size | number | Yes | Min: 100, Max: 800 | Size of the robot visualization in pixels |
| style | string | No | "cute", "realistic", "minimal" | Visual style preference |

## Entity: ParticleColors

**Description**: Color definitions for different particle types

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| innerOrbit | string | Yes | Hex color code | Color for inner orbital particles |
| outerOrbit | string | Yes | Hex color code | Color for outer orbital particles |
| trailEffect | string | No | Hex color code | Color for particle trail effects |

## Entity: AnimationParameters

**Description**: Parameters controlling the animation behavior

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| speed | number | Yes | Min: 0.1, Max: 5.0, Default: 1.0 | Animation speed multiplier |
| innerParticleCount | number | Yes | Min: 4, Max: 30, Default: 12 | Number of inner orbital particles |
| outerParticleCount | number | Yes | Min: 8, Max: 60, Default: 24 | Number of outer orbital particles |
| innerOrbitRadius | number | Yes | Min: 10, Max: 100, Default: 45 | Radius for inner particle orbits |
| outerOrbitRadius | number | Yes | Min: 60, Max: 200, Default: 90 | Radius for outer particle orbits |
| clockwiseParticles | number | No | Min: 0, Max: 100, Default: 50 | Percentage of particles moving clockwise |

## Entity: AccessibilitySettings

**Description**: Settings for accessibility and user preferences

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| reducedMotion | boolean | No | Boolean | Whether to reduce motion effects |
| animationSpeed | number | No | Min: 0, Max: 1.0 | Override for animation speed (0 = paused) |
| highContrast | boolean | No | Boolean | Whether to use high contrast mode |
| particleVisibility | string | No | "all", "core-only", "none" | Level of particle visibility |

## Entity: ParticleState

**Description**: Runtime state of individual particles (transient)

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| id | string | Yes | UUID format | Unique identifier for this particle |
| angle | number | Yes | 0 to 2π radians | Current angular position in orbit |
| orbitRadius | number | Yes | Positive number | Radius of this particle's orbit |
| direction | number | Yes | -1 or 1 | Direction of movement (-1 = counterclockwise, 1 = clockwise) |
| speed | number | Yes | Positive number | Movement speed factor |
| size | number | Yes | Min: 1, Max: 10 | Visual size of the particle |
| color | string | Yes | Hex color code | Color of this particle |
| opacity | number | Yes | 0 to 1 | Opacity level |

## Entity: OrbitalSystem

**Description**: State of the entire orbital particle system

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| robotId | string | Yes | UUID format | Reference to the parent robot |
| particles | ParticleState[] | Yes | Array of ParticleState objects | All active particles in the system |
| lastUpdated | Date | Yes | ISO 8601 format | Timestamp of last state update |
| frameRate | number | No | Min: 1, Max: 60 | Current animation frame rate |
| isActive | boolean | No | Boolean, Default: true | Whether animation is running |

## Relationships

```
RobotConfiguration
    ├── appearance → RobotAppearance
    ├── animationParams → AnimationParameters
    ├── accessibilityOptions → AccessibilitySettings
    └── (generates) → OrbitalSystem
            └── particles → ParticleState[]
```

## Validation Rules

1. **Color Validation**: All color values must be valid hex codes (#RRGGBB format)
2. **Size Validation**: Robot size must be between 100px and 800px
3. **Particle Count**: Total particles (inner + outer) must not exceed 100 for performance
4. **Animation Speed**: Speed values must be positive numbers
5. **Orbit Radius**: Outer orbit radius must be greater than inner orbit radius
6. **Accessibility Consistency**: If reducedMotion is true, animationSpeed should be 0.1 or lower

## State Transitions

### RobotConfiguration States
- `CREATED` → `ACTIVE` → `MODIFIED` → `ARCHIVED`

### OrbitalSystem States
- `INITIALIZING` → `ACTIVE` → `PAUSED` → `STOPPED`

## Indexes

1. **RobotConfiguration.id**: Primary index for configuration lookups
2. **RobotConfiguration.createdAt**: For chronological ordering
3. **OrbitalSystem.robotId**: For linking orbital systems to robots