// @ts-check
const lightCodeTheme = require('prism-react-renderer').themes.github; // Better light theme
const darkCodeTheme = require('prism-react-renderer').themes.dracula;   // Better dark theme

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital AI to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://ayesha-muttalib123.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  organizationName: 'ayesha-muttalib123',
  projectName: 'Physical-AI-Humanoid-Robotics',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/ayesha-muttalib123/Physical-AI-Humanoid-Robotics/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig: ({
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Course Content',
        },
        {
          href: 'https://github.com/ayesha-muttalib123/Physical-AI-Humanoid-Robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        // ... keep your Content links ...
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/ayesha-muttalib123/Physical-AI-Humanoid-Robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © 2025 Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  }),
};

module.exports = config;