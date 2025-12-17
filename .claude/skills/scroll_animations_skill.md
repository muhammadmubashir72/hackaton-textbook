# Scroll-Based Animations Skill

## Skill Type
`scroll-animations`

## Purpose
Implements scroll-triggered animations throughout the Physical AI & Humanoid Robotics course page, creating dynamic content reveals as users scroll through the interface.

## Capabilities
- Creates scroll-triggered animations for content reveals
- Implements parallax effects for visual interest
- Designs staggered animations for related elements
- Optimizes animations for smooth scrolling performance
- Creates entrance animations based on viewport visibility
- Implements scroll-progress based animations

## Required Inputs
- Elements that should animate on scroll
- Animation preferences (delay, duration, easing)
- Trigger thresholds for animations
- Performance constraints
- Accessibility requirements for motion

## Expected Output
- React components with scroll-triggered animations
- Intersection Observer implementations
- Performance-optimized animation code
- Accessibility-compliant animation controls
- Smooth scrolling behavior
- Animation timing and sequence management

## Implementation Steps
1. Identify elements that should animate on scroll
2. Set up Intersection Observer for viewport detection
3. Create entrance animations for content as it enters view
4. Implement staggered animations for related elements
5. Optimize for smooth scrolling performance
6. Add controls for users to reduce motion if needed
7. Test animations across different scroll speeds
8. Ensure animations don't interfere with page performance

## Best Practices
- Use Intersection Observer API for efficient scroll detection
- Implement animations that enhance rather than distract from content
- Optimize for 60fps performance during scrolling
- Provide controls for users with motion sensitivity
- Use appropriate easing functions for natural motion
- Implement throttling or debouncing where necessary
- Test on different devices and scroll methods (mouse, touch, keyboard)
- Avoid animations that cause layout thrashing
- Consider scroll direction in animation logic
- Make animations feel natural and purposeful