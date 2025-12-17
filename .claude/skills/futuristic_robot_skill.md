# Futuristic Robot Visualization Skill

## Skill Type
`futuristic-robot-skill`

## Purpose
This skill provides specialized capabilities for implementing the visual and animation aspects of the futuristic robot. It handles the technical implementation of glowing effects, orbital mechanics, and particle systems for the cute futuristic humanoid robot.

## Capabilities
- Implements SVG-based robot character with layered graphics
- Creates CSS-based glowing effects using box-shadow and filter properties
- Designs orbital path calculations for particle movement
- Implements canvas-based particle systems for performance
- Creates neon glow effects with CSS filters and backdrop-filters
- Generates concentric orbital rings with precise positioning
- Animates particles along circular paths with custom timing
- Implements bidirectional rotation (clockwise and counterclockwise)
- Creates smooth, seamless looping animations
- Optimizes performance for complex particle systems
- Implements responsive scaling for different screen sizes

## Technical Implementation
- Uses CSS variables for easy customization of colors and sizes
- Leverages CSS keyframe animations for orbital movements
- Utilizes SVG masks and filters for glow effects
- Implements requestAnimationFrame for smooth canvas animations
- Uses CSS containment for improved rendering performance
- Applies transform properties for efficient animation performance
- Implements z-index layering for depth perception
- Uses opacity and blend modes for particle transparency effects

## Input Parameters
- Robot base color (default: cyan blue)
- Core/eye accent color (default: orange)
- Number of orbital rings (default: 5)
- Inner orbital particles count (default: 12)
- Outer orbital particles count (default: 24)
- Orbital speeds array for varied movement
- Glow intensity levels
- Animation duration settings
- Responsive breakpoints

## Output Assets
- React component file for the robot visualization
- SCSS/CSS stylesheet with animations and effects
- Utility functions for orbital calculations
- Performance monitoring tools for animation frames
- Responsive layout configurations
- Accessibility settings for animated content
- Exportable animation controls

## Implementation Guidelines
1. Structure the robot using layered SVG elements for crisp rendering
2. Implement the main body with cyan blue fill and subtle gradients
3. Create the chest core as a glowing orange circle with pulsing animation
4. Design the eyes as glowing orange elements with gentle blinking
5. Calculate orbital radii for concentric rings (inner to outer)
6. Position particles along orbital paths using trigonometric functions
7. Implement independent animation timers for each orbital layer
8. Alternate rotation directions (clockwise/counterclockwise) for visual interest
9. Apply glow effects using multiple box-shadows or SVG filters
10. Use easing functions to create natural orbital motion
11. Implement performance optimizations for 60fps animation
12. Add controls for animation speed adjustment and pause/play
13. Ensure smooth rendering on mobile devices with reduced motion settings
14. Provide fallbacks for browsers with limited animation support