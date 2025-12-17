# Frontend Customization Prompts

This document contains detailed prompts for customizing the Docusaurus frontend of the "Physical AI & Humanoid Robotics Textbook" project, covering specification, planning, task breakdown, and initial implementation.

---

## Prompt for `sp.specify`

**User Request:** Fully customize the frontend of the Docusaurus project to create a modern, engaging, and branded experience for the "Physical AI & Humanoid Robotics Textbook". This includes a complete overhaul of the landing page, incorporating specific visual and interactive elements.

**Agent Goal:** Create a comprehensive specification for the Docusaurus frontend customization.

---

**Detailed Specification:**

1.  **Overall Theme & Aesthetic:**
    *   **Goal:** A clean, modern, high-tech, and professional aesthetic suitable for an AI and Robotics educational platform. Emphasize clarity, readability, and a visually appealing user experience.
    *   **Inspiration:** Look at leading tech education platforms or AI/robotics research websites for design cues.

2.  **Typography:**
    *   **Primary Font (Headings):** A bold, modern, sans-serif font for all headings (H1-H6) that conveys innovation and technology.
    *   **Secondary Font (Body Text):** A highly readable sans-serif font for all body text, paragraphs, and general UI elements.
    *   **Monospace Font (Code Blocks):** A clean, distinct monospace font for code examples.
    *   **Integration:** Fonts should be imported from Google Fonts.

3.  **Color Palette:**
    *   **Base Colors:**
        *   **Primary Accent Color:** A vibrant, tech-inspired color (e.g., a shade of blue, teal, or purple) for buttons, links, and key highlights.
        *   **Secondary Accent Color:** A complementary color for secondary actions or contrasting elements.
    *   **Text Colors:**
        *   **Heading Text:** Darker shade for light mode, lighter shade for dark mode.
        *   **Body Text:** Standard dark/light for readability.
        *   **Muted Text:** For secondary information, captions.
    *   **Background Colors:**
        *   **Light Mode Background:** Clean, light background (e.g., off-white, very light grey).
        *   **Dark Mode Background:** Deep, comfortable dark background (e.g., dark grey, deep blue/purple).
    *   **Neutral Colors:** Various shades of grey for borders, dividers, and subtle UI elements.
    *   **Consistency:** Define colors as CSS variables to ensure easy theme switching and maintainability.

4.  **Landing Page (Home Page - `src/pages/index.js`) Overhaul:**
    *   **Goal:** Transform the default Docusaurus landing page into a custom, engaging entry point for the textbook.
    *   **Structure:**
        *   **Hero Section:**
            *   **Layout:** Left-aligned content block (Title, Tagline, Call to Action) and a right-aligned visual element (robotics animation).
            *   **Title:** "Physical AI & Humanoid Robotics Textbook" (from `siteConfig.title`). Prominently displayed.
            *   **Tagline:** "Bridging the gap between digital AI and embodied intelligence in physical systems" (from `siteConfig.tagline`). Clear and concise.
            *   **Call to Action (CTA):** A prominent button (e.g., "Start Learning" or "Explore Textbook") linking to `/docs/intro`.
            *   **Visual Element (Right Side):** An engaging, looping robotics-themed animation (e.g., a LottieFiles animation of a humanoid robot, AI interface, or abstract data flow). If a specific animation cannot be found/integrated initially, use a high-quality placeholder image of a modern robot.
            *   **Background:** A subtle, abstract background pattern or gradient that complements the theme.
        *   **Feature/Overview Section:** Highlight 3-4 key aspects or benefits of the textbook (e.g., "Comprehensive Coverage," "Practical Applications," "Cutting-Edge Research"). Each feature should have an icon/image, a short title, and a brief description.
        *   **Target Audience/Benefits Section:** Clearly articulate who the textbook is for and what learners will gain.
        *   **Call to Action (Secondary):** A less prominent CTA to encourage deeper exploration, possibly linking to specific advanced topics or a table of contents.
        *   **Footer:** Leverage the existing Docusaurus footer but ensure its styling aligns with the new theme.

5.  **Images & Icons:**
    *   **Usage:** Strategic placement of high-quality, relevant images and icons throughout the landing page and potentially other key areas.
    *   **Style:** Modern, consistent, and illustrative of AI/robotics concepts.
    *   **Placeholders:** Use placeholder images initially if specific assets are not available, ensuring they fit the described style.
    *   **Favicon:** Ensure a custom, branded favicon (currently `img/robotics-favicon.ico`).

6.  **Animations & Interactivity:**
    *   **Framer Motion:** Integrate Framer Motion for subtle, elegant animations.
    *   **Hover Effects:** Apply subtle hover animations to buttons, navigation links, and feature cards/elements.
    *   **Scroll Animations:** Implement light, scroll-triggered reveal animations for sections as they come into view (e.g., fade-in, slide-up).
    *   **Robotics Animation:** Ensure the chosen hero animation is smooth and high-quality.

7.  **Light/Dark Theme Support:**
    *   **Mandatory:** Fully support both light and dark themes, ensuring all specified colors, text, and UI elements adapt seamlessly.
    *   **Docusaurus Integration:** Utilize Docusaurus's built-in color mode toggle.

8.  **Responsiveness:**
    *   **Mandatory:** The entire customized frontend must be fully responsive and optimized for various screen sizes (desktop, tablet, mobile).

---

## Prompt for `sp.plan`

**User Request:** Based on the detailed specification for the Docusaurus frontend customization, create a high-level plan for implementation.

**Agent Goal:** Develop a coherent, step-by-step plan that covers the technical approach and major milestones for the frontend overhaul.

---

**High-Level Implementation Plan:**

1.  **Project Setup & Initial Styling (Foundation):**
    *   **Objective:** Establish the core styling elements: fonts, global CSS variables for colors (light/dark themes), and basic layout adjustments.
    *   **Steps:**
        *   Select and integrate chosen Google Fonts into `src/css/custom.css`.
        *   Define a comprehensive set of CSS variables for the color palette (primary, secondary, text, background, etc.) within `src/css/custom.css`, ensuring distinct values for both light and dark modes using Docusaurus's theme system.
        *   Apply initial global styles for `body`, headings, and links using the new fonts and color variables.
        *   Ensure `docusaurus.config.js` correctly points to `custom.css` and `colorMode` is configured.

2.  **Landing Page Component Restructuring (Hero Section First):**
    *   **Objective:** Redesign the main landing page (`src/pages/index.js`) by replacing default Docusaurus components with custom, highly styled ones, starting with the Hero Section.
    *   **Steps:**
        *   Create a new React component file (e.g., `src/components/HeroSection/index.js`) to encapsulate the hero section logic and styling.
        *   Implement the left-aligned content (Title, Tagline, CTA button) within `HeroSection`.
        *   Integrate a placeholder for the right-aligned robotics animation/image in `HeroSection`.
        *   Update `src/pages/index.js` to import and render the new `HeroSection` component, removing the existing `HomepageHeader` and `HomepageFeatures` components.
        *   Develop dedicated CSS Modules (e.g., `src/components/HeroSection/styles.module.css`) for component-specific styling.

3.  **Animation Integration (Framer Motion):**
    *   **Objective:** Introduce dynamic and engaging animations using Framer Motion.
    *   **Steps:**
        *   Install Framer Motion package (`npm install framer-motion`).
        *   Apply subtle hover effects to the CTA button in the `HeroSection` using Framer Motion's `whileHover` prop.
        *   Plan for scroll-triggered animations for subsequent sections of the landing page.

4.  **Developing Additional Landing Page Sections:**
    *   **Objective:** Create the remaining sections of the landing page as per the specification (Feature/Overview, Target Audience, Secondary CTA).
    *   **Steps:**
        *   Create new React components for each section (e.g., `src/components/FeatureSection/index.js`, `src/components/CallToActionSection/index.js`).
        *   Populate these components with placeholder content, images, and icons.
        *   Apply styling using dedicated CSS Modules.
        *   Integrate these new components into `src/pages/index.js`.
        *   Implement scroll-triggered animations for these sections using Framer Motion.

5.  **Responsiveness & Refinement:**
    *   **Objective:** Ensure the entire landing page is fully responsive and polished across devices, and conduct a final aesthetic review.
    *   **Steps:**
        *   Test responsiveness using browser developer tools across various breakpoints.
        *   Adjust CSS as needed for optimal display on mobile, tablet, and desktop.
        *   Review typography, color contrast, spacing, and animation timing.
        *   Address any visual inconsistencies between light and dark modes.

---

## Prompt for `sp.task`

**User Request:** Break down the high-level plan for frontend customization into concrete, actionable tasks for the "Physical AI & Humanoid Robotics Textbook" Docusaurus project.

**Agent Goal:** Generate a detailed list of tasks to guide the implementation process, ensuring all aspects of the specification are covered.

---

**Detailed Task List:**

1.  **Typography & Global Styles:**
    *   **Task 1.1:** Choose specific Google Fonts for headings (e.g., Montserrat) and body text (e.g., Roboto).
    *   **Task 1.2:** Modify `frontend/textbook-physical-ai/src/css/custom.css` to import the selected Google Fonts.
    *   **Task 1.3:** Define CSS variables for the full color palette (primary, secondary, accent, text, background, muted, border) within `custom.css` for both light and dark themes, using Docusaurus's `--ifm-color-primary`, `--ifm-background-color`, `--ifm-text-color` and custom variables as needed.
    *   **Task 1.4:** Apply the new font families and color variables to `html`, `body`, and heading elements (h1-h6) in `custom.css`.
    *   **Task 1.5:** Ensure the existing `docusaurus.config.js` correctly links `custom.css` and has `colorMode: { respectPrefersColorScheme: true }` enabled.

2.  **Hero Section Development:**
    *   **Task 2.1:** Create a new directory and files for the Hero Section component: `frontend/textbook-physical-ai/src/components/HeroSection/index.js` and `frontend/textbook-physical-ai/src/components/HeroSection/styles.module.css`.
    *   **Task 2.2:** Implement the basic React structure for `HeroSection` in `index.js`, including `Heading` for the title, `p` for the tagline, and a `Link` for the CTA button.
    *   **Task 2.3:** Add styling to `HeroSection/styles.module.css` to achieve the left-aligned content layout and appropriate spacing.
    *   **Task 2.4:** Integrate the `siteConfig.title` and `siteConfig.tagline` from `useDocusaurusContext()` into the `HeroSection`.
    *   **Task 2.5:** Add a placeholder `div` or `img` tag on the right side of the `HeroSection` for the robotics animation/image.
    *   **Task 2.6:** Replace the existing `HomepageHeader` component in `frontend/textbook-physical-ai/src/pages/index.js` with the new `HeroSection` component. Remove `HomepageFeatures` for now.

3.  **Framer Motion Integration & Animations:**
    *   **Task 3.1:** Run `npm install framer-motion` in `frontend/textbook-physical-ai/` to add the dependency.
    *   **Task 3.2:** Import `motion` from `framer-motion` into `HeroSection/index.js`.
    *   **Task 3.3:** Apply a `whileHover` animation to the CTA button in `HeroSection` (e.g., subtle scale or shadow effect).

4.  **Additional Landing Page Sections:**
    *   **Task 4.1:** Plan and create `FeatureSection` component with icons/images, titles, and descriptions.
    *   **Task 4.2:** Plan and create `TargetAudienceSection` component.
    *   **Task 4.3:** Plan and create a `SecondaryCallToAction` component.
    *   **Task 4.4:** Integrate these new components into `frontend/textbook-physical-ai/src/pages/index.js`.
    *   **Task 4.5:** Implement scroll-triggered animations for these new sections using Framer Motion.

5.  **Content, Responsiveness & Refinement:**
    *   **Task 5.1:** Populate all new landing page sections with actual content from the textbook or relevant placeholders.
    *   **Task 5.2:** Test the entire landing page on various screen sizes (desktop, tablet, mobile) and adjust CSS in component-specific modules or `custom.css` as needed.
    *   **Task 5.3:** Replace the robotics animation placeholder with an actual LottieFiles animation or a high-quality SVG/image.
    *   **Task 5.4:** Conduct a final visual review of the entire landing page, ensuring consistency in fonts, colors, spacing, and animations across both light and dark themes.

---

## Prompt for `sp.implement`

**User Request:** Implement the initial styling foundation for the "Physical AI & Humanoid Robotics Textbook" Docusaurus frontend. This involves integrating new fonts and defining a comprehensive color palette using CSS variables for both light and dark themes.

**Agent Goal:** Modify `frontend/textbook-physical-ai/src/css/custom.css` to import selected fonts and establish CSS variables for light/dark theme colors.

---

**Implementation Details:**

**File to Modify:** `frontend/textbook-physical-ai/src/css/custom.css`

**Specific Changes:**

1.  **Font Imports:**
    *   Import Google Fonts:
        *   `Montserrat` (for headings, weights 700, 900)
        *   `Roboto` (for body text, weights 400, 700)
        *   (Optional) `Fira Code` (for monospace/code blocks, weight 400)
    *   Add `@import` statements at the very top of `custom.css`.

2.  **Global CSS Variables (Color Palette & Fonts):**
    *   **Root Variables:** Define the base font families for the entire application.
        ```css
        :root {
          --ifm-font-family-base: 'Roboto', sans-serif;
          --ifm-font-family-monospace: 'Fira Code', monospace;
          --ifm-h1-font-family: 'Montserrat', sans-serif;
          --ifm-h2-font-family: 'Montserrat', sans-serif;
          --ifm-h3-font-family: 'Montserrat', sans-serif;
          --ifm-h4-font-family: 'Montserrat', sans-serif;
          --ifm-h5-font-family: 'Montserrat', sans-serif;
          --ifm-h6-font-family: 'Montserrat', sans-serif;
        }
        ```
    *   **Light Theme Variables (default Docusaurus theme):**
        *   Define Docusaurus's `--ifm-color-primary` and other relevant color variables.
        *   Define custom variables like `--custom-color-secondary`, `--custom-color-accent`, etc.
        *   Example (adjust hex codes as per chosen palette):
            ```css
            :root {
              --ifm-color-primary: #3f51b5; /* A shade of blue */
              --ifm-color-primary-dark: #303f9f;
              --ifm-color-primary-darker: #283593;
              --ifm-color-primary-light: #7986cb;
              --ifm-color-primary-lighter: #9fa8da;

              --ifm-background-color: #f0f2f5; /* Light grey */
              --ifm-code-background-color: #e0e0e0;

              --ifm-font-color-base: #333333; /* Dark text */
              --ifm-font-color-secondary: #555555;
              --ifm-font-color-headings: #222222;

              --custom-color-accent: #00bcd4; /* Teal accent */
              --custom-color-border: #cccccc;
            }
            ```
    *   **Dark Theme Variables:**
        *   Define variables within the `.dark` class to override light theme values.
        *   Example (adjust hex codes for dark mode compatibility):
            ```css
            html[data-theme='dark'] {
              --ifm-color-primary: #7986cb; /* Lighter blue for dark mode */
              --ifm-color-primary-dark: #5c6bc0;
              --ifm-color-primary-darker: #5363b4;
              --ifm-color-primary-light: #9fa8da;
              --ifm-color-primary-lighter: #c5cae9;

              --ifm-background-color: #1a202c; /* Deep dark background */
              --ifm-code-background-color: #2d3748;

              --ifm-font-color-base: #e2e8f0; /* Light text */
              --ifm-font-color-secondary: #a0aec0;
              --ifm-font-color-headings: #ffffff;

              --custom-color-accent: #26c6da; /* Lighter teal for dark mode */
              --custom-color-border: #4a5568;
            }
            ```

3.  **Global Font Application:**
    *   Apply the new font families to the `body` and heading tags to ensure consistency throughout the site.
    *   Example:
        ```css
        body {
          font-family: var(--ifm-font-family-base);
          color: var(--ifm-font-color-base);
        }

        h1, h2, h3, h4, h5, h6 {
          font-family: var(--ifm-h1-font-family); /* Or respective heading fonts */
          color: var(--ifm-font-color-headings);
        }
        ```
    *   Ensure proper line heights and font sizes for readability.

**Expected Outcome:** The Docusaurus site should render with the new specified fonts and a visually distinct light and dark theme based on the defined color palette. This lays the groundwork for the custom landing page.
