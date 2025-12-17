
<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles: All principles updated with more specific requirements
Added sections: None
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Embodied Intelligence Focus
The textbook emphasizes the integration of AI with physical systems, focusing on embodied intelligence that bridges the gap between digital brains and physical bodies. Content must cover sensors, robotics, and real-world applications of AI in physical environments.

### II. AI-Native Educational Experience
The textbook is built using AI-native tools (Docusaurus + FastAPI + Neon Postgres + Qdrant + Cohere + Gemini + better-auth) and methodologies, incorporating interactive elements including text selection popup with "Ask AI" and "Translate to Urdu" buttons, RAG-based chatbot, and personalized learning experiences.

### III. Modular Learning Progression
Content follows a clear progression through 6 chapters (2000+ words each): 1) Physical AI Intro: Embodied intelligence, sensors, robotics, 2) ROS 2: Nodes, topics, URDF, Python, 3) Gazebo/Unity: Physics sim, sensors, digital twins, 4) NVIDIA Isaac: Isaac Sim, VSLAM, Nav2, RL, 5) Humanoid Dev: Kinematics, locomotion, manipulation, 6) VLA/Conversational: Voice-to-action, LLM planning, capstone, ensuring students build knowledge systematically.

### IV. Practical Application Emphasis
Each concept is paired with practical examples, simulations, and real-world applications to reinforce theoretical knowledge with tangible experiences. All content must be technically accurate and implementable.

### V. Advanced Technology Integration
Implementation must utilize the specified technology stack: Docusaurus (Vercel) + FastAPI (Hugging Face) + Neon Postgres + Qdrant + Cohere + Gemini + better-auth, with RAG functionality using Cohere embeddings + Qdrant + Cohere Rerank + Gemini responses.

### VI. Accessibility and Inclusion
The textbook provides multilingual support (Urdu translation using googletrans/HF model - NO GPT) and personalized content based on user background (software/hardware experience) to make advanced robotics education accessible to diverse learners.

### VII. Authentication and Personalization

The system implements better-auth with signup questions about user's software/hardware background, and uses Gemini to adjust content based on user level for personalized learning experience.

## Technical Standards

- Use Docusaurus (Vercel) for responsive, modern textbook presentation
- Implement RAG system using Cohere embeddings + Qdrant + Cohere Rerank + Gemini responses
- Deploy via GitHub Pages/Vercel for public accessibility
- Include authentication with better-auth for personalized experiences
- Support content personalization based on user background
- Implement Urdu translation using googletrans/HF model (NO GPT)
- Include text selection → popup with "Ask AI" + "Translate to Urdu" buttons

## Development Workflow

- Follow Spec-Kit Plus methodology for structured development
- Implement 6 complete chapters with 2000+ words each
- Implement all required features: text selection popup, RAG system, Urdu translation, authentication, personalization
- Test all interactive features and simulations
- Validate deployment on GitHub Pages/Vercel
- Conduct user experience testing with sample audience

## Governance

All project decisions must align with these constitutional principles. Changes to the constitution require explicit justification and approval. Content, features, and technical implementations must comply with the established standards and principles. The project must implement all specified features including the complete technology stack and 6 chapters with 2000+ words each.

**Version**: 1.1.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13
