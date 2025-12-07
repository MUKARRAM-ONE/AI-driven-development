import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/AI_Robotics_book/__docusaurus/debug',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug', 'a7e'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/config',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/config', '80c'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/content',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/content', 'fb7'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/globalData',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/globalData', '358'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/metadata',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/metadata', '31b'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/registry',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/registry', '8a2'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/__docusaurus/debug/routes',
    component: ComponentCreator('/AI_Robotics_book/__docusaurus/debug/routes', '9ea'),
    exact: true
  },
  {
    path: '/AI_Robotics_book/docs',
    component: ComponentCreator('/AI_Robotics_book/docs', '78a'),
    routes: [
      {
        path: '/AI_Robotics_book/docs/01-intro-to-ros2',
        component: ComponentCreator('/AI_Robotics_book/docs/01-intro-to-ros2', 'd53'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/02-ros2-nodes-topics-services',
        component: ComponentCreator('/AI_Robotics_book/docs/02-ros2-nodes-topics-services', '517'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/03-python-rclpy-integration',
        component: ComponentCreator('/AI_Robotics_book/docs/03-python-rclpy-integration', 'e5d'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/04-urdf-for-humanoids',
        component: ComponentCreator('/AI_Robotics_book/docs/04-urdf-for-humanoids', '98f'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/05-gazebo-fundamentals',
        component: ComponentCreator('/AI_Robotics_book/docs/05-gazebo-fundamentals', 'ad3'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/06-unity-integration',
        component: ComponentCreator('/AI_Robotics_book/docs/06-unity-integration', '869'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/07-sensor-simulation',
        component: ComponentCreator('/AI_Robotics_book/docs/07-sensor-simulation', 'a79'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/08-isaac-sim-intro',
        component: ComponentCreator('/AI_Robotics_book/docs/08-isaac-sim-intro', '132'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/09-isaac-ros-vslam',
        component: ComponentCreator('/AI_Robotics_book/docs/09-isaac-ros-vslam', 'a0c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/10-nav2-path-planning',
        component: ComponentCreator('/AI_Robotics_book/docs/10-nav2-path-planning', '1c6'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/11-vla-whisper',
        component: ComponentCreator('/AI_Robotics_book/docs/11-vla-whisper', '668'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/12-vla-llm-planning',
        component: ComponentCreator('/AI_Robotics_book/docs/12-vla-llm-planning', '7ce'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/AI_Robotics_book/docs/13-capstone-project',
        component: ComponentCreator('/AI_Robotics_book/docs/13-capstone-project', '4cc'),
        exact: true,
        sidebar: "tutorialSidebar"
      }
    ]
  },
  {
    path: '/AI_Robotics_book/',
    component: ComponentCreator('/AI_Robotics_book/', 'a1e'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
