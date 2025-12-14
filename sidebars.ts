import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
      'CH01-ros2-nervous-system',
      'CH02-digital-twin',
      'CH03-isaac-ai-robot-brain',
      'CH04-vla-system',
      'Capstone-Project',
       {
      type: 'category',
      label: 'Modules',
      items: [
        'Modules/module1-contract',
        'Modules/module2-contract',
        'Modules/module3-contract',
        'Modules/module4-contract',
      ],
    },
  ],
};
 


export default sidebars;
