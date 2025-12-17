# Data Model: Hero Section Elements

## Hero Section
- **Name**: Hero Section
- **Description**: The main hero area containing "PHYSICAL AI · HUMANOID ROBOTICS", "Interactive Curriculum", "Master These Skills", and "Join the Future of Robotics Education" content
- **Fields**:
  - title: string (e.g., "PHYSICAL AI · HUMANOID ROBOTICS")
  - subtitle: string (e.g., "Interactive Curriculum")
  - description: string (e.g., "Master These Skills")
  - cta: string (e.g., "Join the Future of Robotics Education")
  - styling: object (current CSS properties that define box/card appearance)
- **Validation**: All text fields must be non-empty
- **Relationships**: N/A (standalone UI component)

## Performance Metrics
- **Name**: Performance Metrics
- **Description**: Measures of page loading speed and rendering efficiency for the hero section
- **Fields**:
  - loadTime: number (time in milliseconds to load hero section)
  - cssSize: number (size of CSS rules for hero section in bytes)
  - renderTime: number (time in milliseconds to render hero section)
- **Validation**: All values must be positive numbers
- **Relationships**: Associated with Hero Section to track optimization impact