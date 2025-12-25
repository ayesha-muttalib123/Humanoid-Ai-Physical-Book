// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro/theme',
        'intro/goal',
        'intro/overview',
        'intro/why-physical-ai-matters',
        'intro/learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: Foundations of Physical AI',
          items: [
            'modules/module-1/index',
            'modules/module-1/foundations-physical-ai',
            'modules/module-1/embodied-cognition',
            'modules/module-1/sensorimotor-integration'
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Hardware Systems',
          items: [
            'modules/module-2/index'
          ],
        },
        {
          type: 'category',
          label: 'Module 3: Software Frameworks',
          items: [
            'modules/module-3/index'
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Integration and Deployment',
          items: [
            'modules/module-4/index'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Weekly Curriculum',
      items: [
        {
          type: 'category',
          label: 'Week 1: Introduction to Physical AI',
          items: [
            'weeks/week-1/index',
            'weeks/week-1/1-1-foundations-physical-ai',
            'weeks/week-1/1-2-embodied-intelligence-principles',
            'weeks/week-1/1-3-digital-ai-physical-laws',
            'weeks/week-1/1-4-humanoid-robotics-landscape'
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Robot Hardware Fundamentals',
          items: [
            'weeks/week-2/index',
            'weeks/week-2/2-1-lidar-sensors',
            'weeks/week-2/2-2-depth-cameras',
            'weeks/week-2/2-3-imus-balance',
            'weeks/week-2/2-4-force-torque-sensors'
          ],
        },
        {
          type: 'category',
          label: 'Week 3: ROS 2 Architecture',
          items: [
            'weeks/week-3/index',
            'weeks/week-3/3-1-ros2-architecture',
            'weeks/week-3/3-2-nodes-topics-services-actions',
            'weeks/week-3/3-3-ros2-packages',
            'weeks/week-3/3-4-urdf-overview'
          ],
        },
        {
          type: 'category',
          label: 'Week 4: URDF and ROS 2 Tools',
          items: [
            'weeks/week-4/index',
            'weeks/week-4/4-1-urdf-detailed',
            'weeks/week-4/4-2-ros2-launch-system',
            'weeks/week-4/4-3-ros2-parameter-system',
            'weeks/week-4/4-4-ros2-tools'
          ],
        },
        {
          type: 'category',
          label: 'Week 5: SLAM and Navigation',
          items: [
            'weeks/week-5/index',
            'weeks/week-5/5-1-slam-mapping',
            'weeks/week-5/5-2-navigation-stacks',
            'weeks/week-5/5-3-perception-pipelines',
            'weeks/week-5/5-4-sensor-fusion'
          ],
        },
        {
          type: 'category',
          label: 'Week 6: Control Systems',
          items: [
            'weeks/week-6/index',
            'weeks/week-6/6-1-pid-control-theory',
            'weeks/week-6/6-2-trajectory-generation-execution',
            'weeks/week-6/6-3-motion-planning-algorithms',
            'weeks/week-6/6-4-whole-body-control'
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Simulation Tools',
          items: [
            'weeks/week-7/index',
            'weeks/week-7/7-1-gazebo-simulation',
            'weeks/week-7/7-4-ros2-tools-integration'
          ],
        },
        {
          type: 'category',
          label: 'Week 8: AI in Robotics',
          items: [
            'weeks/week-8/index',
            'weeks/week-8/8-1-machine-learning-robotics',
            'weeks/week-8/8-2-computer-vision-physical-ai',
            'weeks/week-8/8-3-natural-language-hri',
            'weeks/week-8/8-4-ai-model-deployment'
          ],
        },
        {
          type: 'category',
          label: 'Week 9: Advanced Topics',
          items: [
            'weeks/week-9/index',
            'weeks/week-9/9-1-swarm-robotics',
            'weeks/week-9/9-2-advanced-perception-control',
            'weeks/week-9/9-3-safety-ethics',
            'weeks/week-9/9-4-future-trends'
          ],
        },
        {
          type: 'category',
          label: 'Week 10: Advanced RL and Planning',
          items: [
            'weeks/week-10/index',
            'weeks/week-10/10-1-advanced-rl-techniques',
            'weeks/week-10/10-2-sim-to-real-optimization',
            'weeks/week-10/10-3-embodied-ai-research',
            'weeks/week-10/10-4-project-planning'
          ],
        },
        {
          type: 'category',
          label: 'Week 11: Humanoid Kinematics and Locomotion',
          items: [
            'weeks/week-11/index',
            'weeks/week-11/11-1-humanoid-kinematics',
            'weeks/week-11/11-2-humanoid-locomotion',
            'weeks/week-11/11-3-humanoid-manipulation',
            'weeks/week-11/11-4-humanoid-interaction'
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Humanoid Development and Projects',
          items: [
            'weeks/week-12/index',
            'weeks/week-12/12-1-humanoid-control-systems',
            'weeks/week-12/12-2-humanoid-safety-protocols',
            'weeks/week-12/12-3-humanoid-project-implementation',
            'weeks/week-12/12-4-project-refinement'
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Conversational Robotics and Capstone',
          items: [
            'weeks/week-13/index',
            'weeks/week-13/13-1-gpt-integration',
            'weeks/week-13/13-2-speech-processing',
            'weeks/week-13/13-3-multi-modal-integration',
            'weeks/week-13/13-4-capstone-overview'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware/index',
        'hardware/jetson-orin-nano-super',
        'hardware/realsense-d435i',
        'hardware/respeaker',
        'hardware/unitree-go2',
        'hardware/unitree-g1',
        'hardware/hardware-comparison-table'
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/glossary',
        'appendix/references',
        'appendix/further-reading'
      ],
    },
  ],
};

module.exports = sidebars;