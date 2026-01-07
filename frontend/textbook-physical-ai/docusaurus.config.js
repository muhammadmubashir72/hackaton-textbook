// @ts-check
import { themes as prismThemes } from "prism-react-renderer";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics Textbook",
  tagline:
    "Bridging the gap between digital AI and embodied intelligence in physical systems",
  favicon: "img/robotics-favicon.jpg",

  url: "https://textbook-physical-ai.vercel.app", // Updated for Vercel deployment
  baseUrl: "/",

  organizationName: "muhammad-mubashir-saeedi",
  projectName: "textbook-physical-ai",

  onBrokenLinks: "throw",

  // Performance optimizations
  trailingSlash: false,
  noIndex: false,

  markdown: {
    format: 'mdx',
    mermaid: true,
  },

  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.js",
          // Options for better performance
          routeBasePath: "/docs",
          editUrl: "https://github.com/muhammad-mubashir-saeedi/textbook-physical-ai/edit/main/",
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false, // Disable blog for better performance if not needed
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      },
    ],
  ],

  themeConfig: {
    image: "img/physical-ai-social-card.jpg",
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    navbar: {
      hideOnScroll: false,
      items: [
        {
          type: "html",
          value:
            '<a href="/" class="navbar__logo-text"><span class="navbar__title-main">PHYSICAL AI & HUMANOID ROBOTS</span></a>',
          position: "left",
        },
        {
          type: "doc",
          docId: "introduction/intro",
          position: "left",
          label: "TextBook",
          className: "textbook-link",
        },
        {
          type: "html",
          value: '<div id="custom-theme-toggle-mount"></div>',
          position: "right",
        },
        {
          type: "html",
          value: `
        <a href="https://github.com/muhammad-mubashir-saeedi/textbook-physical-ai"
           target="_blank" rel="noopener noreferrer"
           class="header-github-link" style="margin-right: 1rem;">
          <svg width="24" height="24" viewBox="0 0 24 24">
            <path d="M12 2A10 10 0 0 0 2 12c0 4.42 2.87 8.17 6.84 9.5.5.08.66-.23.66-.5v-1.69c-2.77.6-3.36-1.34-3.36-1.34-.46-1.16-1.11-1.47-1.11-1.47-.91-.62.07-.6.07-.6 1 .07 1.53 1.03 1.53 1.03.87 1.52 2.34 1.07 2.91.83.09-.65.35-1.09.63-1.34-2.22-.25-4.55-1.11-4.55-4.92 0-1.11.38-2 1.03-2.71-.1-.25-.45-1.29.1-2.64 0 0 .84-.27 2.75 1.02.79-.22 1.65-.33 2.5-.33.85 0 1.71.11 2.5.33 1.91-1.29 2.75-1.02 2.75-1.02.55 1.35.2 2.39.1 2.64.65.71 1.03 1.6 1.03 2.71 0 3.82-2.34 4.66-4.57 4.91.36.31.69.92.69 1.85V21c0 .27.16.59.67.5C19.14 20.16 22 16.42 22 12A10 10 0 0 0 12 2z"/>
          </svg>
          <span style="margin-left: 0.5rem;">GitHub</span>
        </a>`,
          position: "right",
        },
        {
          type: 'custom-auth',
          position: 'right',
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Resources",
          items: [
            { label: "About Me", href: "#" },
            {
              label: "My Projects",
              href: "#",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/muhammad-mubashir-saeedi/textbook-physical-ai",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Created by Personal Development. Built with Docusaurus for Physical AI Education.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },

  scripts: [
    {
      src: '/js/custom-theme-toggle.js',
      async: true,
      defer: true,
    },
    // Expose environment variables to the client
    {
      src: '/js/env.js',
      async: true,
      defer: true,
    },
  ],

  // Additional performance optimizations
  plugins: [
    // Client-side redirection plugin for faster navigations
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/introduction/intro', // /docs/
            from: ['/docs'], // /docs/index.html
          },
        ],
      },
    ],
    // Sitemap plugin - Note: The sitemap plugin is already included in the classic preset,
    // but we're configuring it separately to override default settings
  ],
};

export default config;
