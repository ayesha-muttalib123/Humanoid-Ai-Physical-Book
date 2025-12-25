---
sidebar_label: Hardware Comparison Table
title: Hardware Comparison Table
description: Comparison of hardware options for Physical AI and Humanoid Robotics
keywords: [hardware, comparison, robotics, computing, sensors, platforms]
---

# Hardware Comparison Table

## Overview

This table provides a comprehensive comparison of hardware options for Physical AI and Humanoid Robotics applications. The prices are current as of 2025 and may vary based on configuration, region, and availability.

## Computing Platforms

| Platform | Price (2025) | CPU | GPU | AI Performance | Power | Best For |
|----------|--------------|-----|-----|----------------|-------|----------|
| Jetson Orin Nano Super | $249 | 6-core ARM Cortex-A78AE | Ada Lovelace GPU (4096 CUDA cores) | 20 TOPS INT8 | 15-70W | Mobile robotics, computer vision, AI inference |
| Jetson Orin NX | $749 | 6-core ARM Cortex-A78AE | Ada Lovelace GPU (1024 CUDA cores) | 77 TOPS INT8 | 15-25W | Advanced robotics, multi-sensor processing |
| Jetson AGX Orin | $1,299 | 12-core ARM Cortex-X2 | Ada Lovelace GPU (2048 CUDA cores) | 275 TOPS INT8 | 30-60W | Heavy-duty AI, complex robotics |
| Raspberry Pi 4 | $75 | 4-core ARM Cortex-A72 | VideoCore VI | Limited | 5-8W | Educational, simple applications |

## Depth Cameras

| Platform | Price (2025) | Technology | Range | Accuracy | Best For |
|----------|--------------|------------|-------|----------|----------|
| Intel RealSense D435i | $349 | Stereo Vision + IMU | 0.2m-10m | ±2% | Robotics, mapping, SLAM |
| Intel RealSense D455 | $449 | Stereo Vision + IMU | 0.1m-20m | ±1% | High-precision applications |
| Intel RealSense L515 | $499 | LiDAR | 0.25m-9m | ±1% | Indoor mapping, low-texture scenes |
| ZED 2i | $459 | Stereo Vision | 0.2m-20m | ±1% | Outdoor applications, higher accuracy |

## Robot Platforms

| Platform | Price (2025) | Type | DOF | Speed | Best For |
|----------|--------------|------|-----|-------|----------|
| Unitree Go2 | $1,800-$3,000 | Quadruped | 12 | 1.6 m/s | Locomotion research, dynamic control |
| Unitree G1 | ~$16,000 | Humanoid | 23 | 1.2 m/s | Humanoid robotics, bipedal control |
| TurtleBot 4 | $1,495 | Mobile Base | N/A | 1.5 m/s | Educational, navigation |
| Clearpath Husky | $10,995 | Mobile Platform | N/A | 1.0 m/s | Research, outdoor applications |

## Audio Processing

| Platform | Price (2025) | Microphones | Features | Best For |
|----------|--------------|-------------|----------|----------|
| ReSpeaker 6-mic | $69 | 6 microphones | VAD, AEC, DOA | Voice interaction, basic audio |
| ReSpeaker 4-mic | $49 | 4 microphones | VAD, AEC | Voice commands, audio capture |
| Matrix Voice | $79 | 8 microphones | Far-field, beamforming | Advanced audio processing |
| Respeaker Mic Array v2.0 | $89 | 6 microphones | ROS integration | Robotics applications |

## Selection Guidelines

### For Beginners
- **Computing**: Jetson Orin Nano Super ($249) - good balance of performance and cost
- **Depth Camera**: Intel RealSense D435i ($349) - reliable performance and integration
- **Audio**: ReSpeaker 6-mic ($69) - affordable and functional

### For Advanced Research
- **Computing**: Jetson AGX Orin ($1,299) - maximum performance for complex AI
- **Depth Camera**: Intel RealSense L515 ($499) - LiDAR technology for precision
- **Robot Platform**: Unitree Go2 ($1,800+) for locomotion research or Unitree G1 (~$16k) for humanoid research

### For Educational Institutions
- **Mobile Base**: TurtleBot 4 ($1,495) - comprehensive educational platform
- **Computing**: Jetson Orin NX ($749) - good performance for classroom use
- **Sensors**: Intel RealSense D435i ($349) - robust and well-documented

## Budget Considerations

When planning hardware purchases, consider:
- **Total Cost of Ownership**: Including accessories, cables, power supplies
- **Development Time**: More capable platforms may reduce development time
- **Future Upgrades**: Expandable platforms may offer better long-term value
- **Support and Documentation**: Well-supported platforms reduce development challenges

## Availability and Lead Times

- **Computing Platforms**: Generally available with short lead times
- **Sensors**: Usually in stock, but specialized models may have longer lead times
- **Robot Platforms**: May have lead times of several weeks to months depending on configuration and demand

## Conclusion

Selecting the right hardware for your Physical AI and Humanoid Robotics project depends on your specific requirements, budget, and technical expertise. The platforms listed above represent some of the most popular and well-supported options available as of 2025.

For most applications, starting with a Jetson Orin Nano Super for computing, Intel RealSense D435i for depth perception, and ReSpeaker for audio provides a solid foundation that can be expanded as needed.