import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Embodied Intelligence: Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to AI-native physical AI and humanoid robotics using Claude Code, Spec-Kit Plus, and Docusaurus.',
  favicon: 'img/robo.png',
  future: { v4: true },
  url: 'https://laibakamrankhan.github.io',
  baseUrl: '/Physical_AI_Humanoid_Robotics_Book/',
  organizationName: 'Laibakamrankhan',
  projectName: 'Physical_AI_Humanoid_Robotics_Book',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
   i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },


  presets: [
    ['classic', {
      docs: {
        path: '1-physical-ai-robotics/',
        routeBasePath: '/',
        sidebarPath: require.resolve('./sidebars.ts'),
        editUrl: 'https://github.com/Laibakamrankhan/Physical_AI_Humanoid_Robotics_Course/edit/main/',
      },
      blog: {
        showReadingTime: true,
        feedOptions: { type: ['rss','atom'], xslt: true },
        editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        onInlineTags: 'warn',
        onInlineAuthors: 'warn',
        onUntruncatedBlogPosts: 'warn',
      },
      theme: { customCss: './src/css/custom.css' },
    } satisfies Preset.Options],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: 'Physical AI & Humanoid Robotics Course Book',
      logo: { alt: 'Book Logo', src: 'img/robo.png' }, 
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Start Learning' },
        {
          label: 'Modules',
          position: 'left',
          items: [
            { label: 'Module 1 — ROS 2', to: 'Modules/module1-contract' },
            { label: 'Module 2 — Digital Twin', to: 'Modules/module2-contract' },
            { label: 'Module 3 — AI-Robot Brain', to: 'Modules/module3-contract' },
            { label: 'Module 4 — VLA', to: 'Modules/module4-contract' },
          ],
        },
        { href: 'https://github.com/your-github-username/Physical_AI_Humanoid_Robotics_Course', label: 'GitHub', position: 'right' },
     ],
    },
    footer: {
      style: 'dark',
      links: [
        { title: 'Community', items: [
            { label: 'Stack Overflow', href: 'https://stackoverflow.com/questions/tagged/docusaurus' },
            { label: 'Discord', href: 'https://discordapp.com/invite/docusaurus' },
          ]
        },
        { title: 'More', items: [
            { label: 'GitHub', href: 'https://github.com/your-github-username/Physical_AI_Humanoid_Robotics_Course' },
          ]
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Embodied Intelligence Project. Built with Docusaurus.`,
    },
  },
};

export default config;
