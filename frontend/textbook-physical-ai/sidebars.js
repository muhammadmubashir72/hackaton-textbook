// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar for the Physical AI & Humanoid Robotics textbook
  textbookSidebar: [
    'index',
    {
      type: 'category',
      label: 'Chapter 1: Physical AI Introduction',
      items: [
        'introduction/intro',
        'introduction/embodied-intelligence',
        'introduction/sensors-robotics',
        'introduction/hardware-requirements'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 - The Robotic Nervous System',
      items: [
        'ros2/intro',
        'ros2/architecture-concepts',
        'ros2/python-implementation',
        'ros2/urdf-description'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Gazebo/Unity - The Digital Twin',
      items: [
        'gazebo-unity/intro',
        'gazebo-unity/physics-simulation',
        'gazebo-unity/sensor-simulation',
        'gazebo-unity/unity-integration'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac - The AI-Robot Brain',
      items: [
        'nvidia-isaac/intro',
        'nvidia-isaac/isaac-sim',
        'nvidia-isaac/vslam-nav2',
        'nvidia-isaac/reinforcement-learning'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Humanoid Robot Development',
      items: [
        'humanoid-development/intro',
        'humanoid-development/kinematics',
        'humanoid-development/locomotion',
        'humanoid-development/manipulation'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6: VLA/Conversational Robotics',
      items: [
        'vla-conversational/intro',
        'vla-conversational/voice-action',
        'vla-conversational/llm-planning',
        'vla-conversational/capstone-project'
      ],
    },
  ],
};

export default sidebars;
