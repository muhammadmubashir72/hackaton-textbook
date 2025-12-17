# Website Redesign Specification

## Project Overview
Redesign the entire book website with a single, consistent theme and one modern, technical font used across all pages. Implement clearly defined dark and light modes with solid colors only, no gradients, and a serious, focused mood suitable for robotics and Physical AI content.

## Requirements

### 1. Theme & Typography
- **Single, consistent theme** across all pages
- **One modern, technical font** used throughout the site
- Font should be professional and suitable for technical documentation
- Typography should be clean and readable

### 2. Color Schemes
- **Dark mode**: Deep dark tones with soft, readable text
- **Light mode**: Off-white backgrounds with dark, comfortable text
- **No gradients** - solid colors only
- Professional color palette suitable for robotics content
- Consistent color usage across all components

### 3. Mood & Design
- **Serious, focused mood** suitable for robotics and Physical AI content
- Clean, minimal, and professional design
- No visual clutter or playful elements
- Technical and academic aesthetic

### 4. Navigation Updates
- Remove the "Docs" item from the navbar
- Replace with a GitHub link
- Add a clean light/dark mode toggle in the navbar
- Toggle should remember user preference and respect system settings

### 5. Layout & Styling
- Calm, minimal, and professional layout
- No visual clutter or decorative elements
- Consistent styling across all pages
- Proper spacing and typography hierarchy

## Technical Requirements

### 1. CSS Framework
- Use Docusaurus CSS variables system
- Implement theme customization via CSS variables
- Ensure responsive design is maintained

### 2. Fonts
- Primary font: Technical/programming-friendly font (e.g., Inter, JetBrains Mono)
- Fallback fonts for cross-browser compatibility
- Consistent font sizing and line heights

### 3. Color Variables
- Define CSS variables for both light and dark themes
- Ensure proper contrast ratios for accessibility
- Use consistent color naming conventions

### 4. Components
- Update all Docusaurus components to match new theme
- Ensure all buttons, cards, and UI elements follow new design
- Maintain existing functionality while updating appearance

## Success Criteria

### 1. Visual Consistency
- [ ] All pages use the same theme and typography
- [ ] Consistent color usage across all components
- [ ] Professional appearance maintained throughout

### 2. Theme Implementation
- [ ] Dark mode works correctly with deep dark tones
- [ ] Light mode uses off-white backgrounds with dark text
- [ ] No gradients used anywhere in the design
- [ ] Theme toggle functions properly

### 3. Navigation Updates
- [ ] "Docs" item removed from navbar
- [ ] GitHub link added to navbar
- [ ] Theme toggle added to navbar
- [ ] All navbar elements properly positioned

### 4. Accessibility
- [ ] Proper color contrast ratios maintained
- [ ] Font sizes remain readable
- [ ] Focus states properly styled
- [ ] Theme preference persistence works

## Constraints

### 1. Technology
- Must work within Docusaurus framework
- Use CSS modules where appropriate
- Maintain existing site functionality
- Don't break existing links or navigation

### 2. Performance
- Keep CSS bundle size reasonable
- Don't add unnecessary dependencies
- Maintain fast loading times
- Optimize for performance

### 3. Compatibility
- Support modern browsers
- Maintain responsive design
- Ensure mobile compatibility
- Cross-platform consistency

## Dependencies

### 1. Existing Infrastructure
- Docusaurus site structure
- Current documentation content
- Existing navigation structure
- Current deployment setup

### 2. External Resources
- Google Fonts for technical font
- Docusaurus theme system
- CSS variables support

## Risks

### 1. Visual
- Risk of making the site too minimal and losing important visual hierarchy
- Potential contrast issues with new color scheme
- Risk of breaking existing visual elements

### 2. Functional
- Risk of breaking existing navigation or functionality
- Theme toggle may not work properly across all browsers
- CSS changes might affect responsive behavior

### 3. User Experience
- Users may prefer the old theme
- Theme preference persistence might not work correctly
- Font changes might affect readability for some users

## Assumptions

### 1. Technical
- Docusaurus supports the required theme customizations
- CSS variables work as expected across target browsers
- The technical font is available via Google Fonts

### 2. Content
- Existing content structure will remain unchanged
- Navigation structure is flexible enough for changes
- Images and media will adapt to new color scheme

## Out of Scope

### 1. Content Changes
- No changes to documentation content
- No changes to site structure or information architecture
- No addition of new content or pages

### 2. Advanced Features
- No new JavaScript functionality
- No complex animations or interactions
- No integration with external APIs

## Acceptance Criteria

### 1. Theme Implementation
- [ ] All pages display with new consistent theme
- [ ] Both light and dark modes function correctly
- [ ] No gradients present anywhere in the design
- [ ] Typography is consistent across all pages

### 2. Navigation
- [ ] "Docs" link is removed from navbar
- [ ] GitHub link is present and functional
- [ ] Theme toggle is present and functional
- [ ] All links continue to work as expected

### 3. Design Quality
- [ ] Design is clean, minimal, and professional
- [ ] No visual clutter or unnecessary elements
- [ ] Color scheme is appropriate for technical content
- [ ] Typography is readable and well-structured

### 4. Functionality
- [ ] Theme preference is remembered between sessions
- [ ] System color scheme preference is respected
- [ ] All existing functionality remains intact
- [ ] Responsive design continues to work properly