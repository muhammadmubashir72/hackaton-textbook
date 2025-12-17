# Data Model: Theme System

## Theme State
- **Name**: Theme State
- **Description**: Represents the current color mode and persists user preference
- **Fields**:
  - mode: string (light | dark)
  - lastUpdated: timestamp
  - persistenceMethod: string (localStorage, system preference, user setting)
- **Validation**: Mode must be one of the allowed values
- **State transitions**: light ↔ dark (toggle action)

## Color Palette
- **Name**: Color Palette
- **Description**: Collection of color definitions for different visual elements
- **Fields**:
  - backgroundColor: string (CSS color value)
  - textColor: string (CSS color value)
  - navbarColor: string (CSS color value)
  - contentAreaColor: string (CSS color value)
  - theme: string (light | dark)
- **Validation**: All color values must be valid CSS color formats
- **Relationships**: Associated with Theme State to determine active colors

## Theme Preference
- **Name**: Theme Preference
- **Description**: User's theme selection preference
- **Fields**:
  - userId: string (optional, for authenticated users)
  - themeMode: string (light | dark)
  - autoDetect: boolean (whether to follow system preference)
  - updatedAt: timestamp
- **Validation**: If autoDetect is true, system preference should be followed