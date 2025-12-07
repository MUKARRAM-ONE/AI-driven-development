<!--
---
- Sync Impact Report ---
- Version change: 1.0.0 (no change)
- Templates requiring updates:
-  - ✅ .specify/templates/plan-template.md
-  - ✅ .specify/templates/tasks-template.md
-  - ✅ .gemini/commands/sp.constitution.toml
- Follow-up TODOs: none
-->
# Physical AI & Humanoid Robotics Textbook — Constitution

## Core Principles

### I. Research-First, Evidence-Based Content
All material in the textbook must be:
- Grounded in peer-reviewed academic research  
- Verified against official documentation (ROS 2, Gazebo, Isaac Sim, Whisper, REP, URDF)  
- Free from hallucinated or unverifiable claims  

All claims must map to entries in `/research/sources.md`.  
Every chapter must include IEEE-formatted references.

---

### II. Spec-First, AI-Driven Development
The entire textbook is produced using:
- Spec-Kit Plus (feature specs, tasks, plans, ADRs)  
- Gemini CLI (generation engine)  
- A structured “spec → research → generation → verification” workflow  

No chapter is created without a spec.  
No spec is approved without a checklist.  
All content is generated from reproducible tasks.

---

### III. Test-First & Citation-First Validation
A chapter is valid only when:
- It passes citation verification (≥3 academic papers + official docs)  
- References are properly formatted  
- The chapter survives hallucination checks  
- Code examples are runnable  
- Diagrams exist in `/static/img`  

Verification Script:  
`scripts/verify_sources.py`

---

### IV. Modular, Library-Style Architecture
The book is divided into four standalone learning modules:
1. **ROS 2 — The Robotic Nervous System**  
2. **Digital Twin Simulation — Gazebo & Unity**  
3. **NVIDIA Isaac — AI Robotics Stack**  
4. **Vision-Language-Action Systems**  

Each module is treated as a library with:
- Its own spec  
- Its own research requirements  
- Independently reviewable content  
- Its own diagrams & exercises  

---

### V. Observability, Logs, and Reproducibility
All generated content must include:
- AI generation logs  
- Research source logs  
- Metadata for reproducibility  
- Traceability to the spec and task that produced it  

All research artifacts go into `/research/`.

---

### VI. Integration, Continuity & Real-World Application
The system must show:
- How robotics, simulation, AI, and VLA interact  
- How ROS 2 integrates with Gazebo and Isaac  
- How voice, vision, and action map into robotics pipelines  
- How each module supports the final capstone project  

Every chapter must contain:
- Real-world examples  
- Small practical tasks  
- Industry context  

---

### VII. Simplicity, Clarity & Pedagogy
Content must be:
- Beginner-friendly  
- Free of unnecessary jargon  
- Diagram-supported  
- Code-driven  
- Step-by-step  

The target audience is new robotics learners with basic Python.

---

## Project Constraints
- Must follow Spec-Kit Plus workflow  
- Must be generated primarily with Gemini CLI  
- Must function on Windows environments  
- Must produce:
  - A textbook  
  - A Docusaurus website  
  - A RAG search backend  
  - A complete capstone project  

All generated chapters must be Markdown structured and placed under `/docs`.

---

## Development Workflow

### 1. Specification Phase
- Create module specs under `/specs`  
- Use Spec-Kit templates  
- Define research requirements  
- Create tasks + plan files  

### 2. Research Phase
- Populate `/research/sources.md`  
- Collect papers + documentation  
- Run research gathering tools  

### 3. Generation Phase
- Use Gemini CLI to generate chapters  
- Produce diagrams via scripts  
- Store outputs in `/docs`  

### 4. Verification Phase
- Run citation validator  
- Run style and hallucination checks  
- Validate diagrams and code  

### 5. Integration Phase
- Insert chapters into Docusaurus  
- Build RAG backend  
- Publish to web  

---

## Quality Gates
Before merging any chapter, ensure:

- [ ] Minimum 2500 words  
- [ ] ≥3 academic citations  
- [ ] ≥1 official documentation link  
- [ ] IEEE reference formatting  
- [ ] Diagram included  
- [ ] Code examples runnable  
- [ ] Follows module spec  
- [ ] Reviewed under Spec-Kit workflow  
- [ ] Reproducible generation steps logged  

---

## Governance

- This Constitution supersedes all other rules.  
- All features must follow:
  - A spec (`/specs/...`)  
  - A plan  
  - Corresponding tasks  
  - ADR entries for major decisions  

- Amendments require:
  - Clear motivation  
  - Description of change  
  - Updated version number  
  - Migration instructions  

**Version:** 1.0.0  
**Ratified:** 2025-12-07  
**Last Amended:** 2025-12-07