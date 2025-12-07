# Physical AI & Humanoid Robotics Textbook

A comprehensive, open-source learning resource for AI-driven humanoid robot development.

## 📚 What is This?

This textbook is designed to teach you everything you need to know about developing intelligent humanoid robots, from foundational robotics concepts to cutting-edge AI integration techniques.

## 🎓 Course Structure

The textbook is organized into **4 main modules** with **13 detailed chapters**:

### Module 1: ROS 2 Fundamentals
- Chapter 1: Introduction to ROS 2
- Chapter 2: Nodes, Topics, and Services
- Chapter 3: Python Integration with rclpy

### Module 2: Digital Twin & Simulation
- Chapter 4: URDF for Humanoid Robots
- Chapter 5: Gazebo Fundamentals
- Chapter 6: Unity Integration
- Chapter 7: Sensor Simulation
- Chapter 8: Isaac Sim Introduction

### Module 3: AI-Robot Brain
- Chapter 9: Isaac ROS Visual SLAM
- Chapter 10: Nav2 Path Planning

### Module 4: Vision Language Models (VLMs)
- Chapter 11: VLM with Whisper
- Chapter 12: LLM Planning
- Chapter 13: Capstone Project

## 🚀 Getting Started

### View the Textbook

Navigate to the `docs/` directory and run:

```bash
npm install
npm run start
```

Then open `http://localhost:3000` in your browser.

### Build the Static Site

```bash
cd docs
npm run build
```

The built site will be in the `docs/build/` directory.

## 📁 Directory Structure

```
├── docs/                      # Docusaurus documentation site
│   ├── docs/                  # Chapter markdown files
│   ├── static/                # Images and assets
│   ├── src/                   # Custom pages and styling
│   ├── package.json
│   └── docusaurus.config.js
├── assessments/               # Quizzes and assignments
├── contracts/                 # Project specifications
├── planning/                  # Course curriculum
├── research/                  # Reference papers
├── specs/                     # Technical specifications
├── src/                       # Example source code
└── README.md                  # This file
```

## 🛠️ Technology Stack

- **ROS 2** - Robot Operating System
- **Gazebo** - Physics-based simulator
- **URDF** - Unified Robot Description Format
- **Isaac Sim** - Advanced NVIDIA simulation
- **Python (rclpy)** - ROS Python client
- **LLMs & VLMs** - Artificial intelligence models
- **Docusaurus** - Documentation platform

## 📖 Key Features

✅ **Comprehensive Content** - From basics to advanced topics
✅ **Hands-on Examples** - Real code snippets and tutorials
✅ **Visual Diagrams** - Architecture and workflow diagrams
✅ **Progressive Difficulty** - Beginner-friendly to expert-level
✅ **Open Source** - Community-driven development

## 🎯 Learning Outcomes

By the end of this course, you will:
- Understand ROS 2 architecture and development
- Create digital twins of robotic systems
- Implement AI-driven behaviors
- Work with simulation tools (Gazebo, Isaac Sim)
- Deploy VLM-based robot controllers
- Build complete autonomous systems

## 🤝 Contributing

Contributions are welcome! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Make your changes
4. Submit a pull request

## 📄 License

MIT License - See LICENSE file for details

## 📞 Contact & Support

- **GitHub Issues** - Report bugs and request features
- **Discussions** - Ask questions and discuss topics
- **Wiki** - Additional resources and FAQs

---

**Start your robotics journey today! 🤖**

For more information, visit: https://ai-driven-development.github.io/AI_Robotics_book/
