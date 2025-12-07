# Tasks: Physical AI & Humanoid Robotics Textbook Module

**Input**: Design documents from `/specs/1-physical-ai-robotics/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the Docusaurus website.

- [X] T001 Initialize Docusaurus project in `docs/`.
- [X] T002 Configure Docusaurus sidebar and navigation in `docs/docusaurus.config.js`.
- [X] T003 Create `static/img` directory for diagrams.
- [X] T004 [P] Populate `research/sources.md` with initial links from `plan.md`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core content structure that MUST be complete before ANY chapter generation can begin.

- [X] T005 Create detailed chapter outlines for all 13 chapters in a temporary `planning/` directory.

---

## Phase 3: User Story 1 - Learning Core Robotics Middleware (ROS 2) (Priority: P1) 🎯 MVP

**Goal**: Students can understand and use ROS 2 for basic robot control.
**Independent Test**: Student can create a ROS 2 package with a publisher and subscriber.

### Implementation for User Story 1

- [X] T006 [US1] Research ROS 2 best practices for humanoid robot control and rclpy, update `research/research.md`.
- [X] T007 [P] [US1] Generate content for Chapter 1: Introduction to ROS 2 in `docs/01-intro-to-ros2.md`.
- [X] T008 [P] [US1] Generate content for Chapter 2: Nodes, Topics, and Services in `docs/02-ros2-nodes-topics-services.md`.
- [X] T009 [P] [US1] Generate content for Chapter 3: Python Integration with rclpy in `docs/03-python-rclpy-integration.md`.
- [X] T010 [P] [US1] Generate content for Chapter 4: URDF for Humanoids in `docs/04-urdf-for-humanoids.md`.
- [X] T011 [P] [US1] Create architecture diagram for Module 1 in `static/img/module1-architecture.svg`.
- [X] T012 [P] [US1] Create sequence diagram for ROS 2 pub/sub interaction in `static/img/module1-pub-sub-sequence.svg`.
- [X] T013 [P] [US1] Develop code examples for Module 1 in `src/module1/`.
- [X] T014 [P] [US1] Develop auto-gradable assessment project for Module 1 in `assessments/module1/`.
- [X] T015 [US1] Verify all code examples for Module 1 are runnable and have basic logging.
- [X] T016 [US1] Verify all chapters for Module 1 meet citation and quality gates.

---

## Phase 4: User Story 2 - Simulating a Digital Twin (Gazebo/Unity) (Priority: P2)

**Goal**: Students can simulate a robot and its environment.
**Independent Test**: Student can load a robot in Gazebo and visualize it in Unity.

### Implementation for User Story 2

- [ ] T017 [US2] Research Gazebo best practices and Unity visualization techniques, update `research/research.md`.
- [X] T018 [P] [US2] Generate content for Chapter 5: Gazebo Simulation Fundamentals in `docs/05-gazebo-fundamentals.md`.
- [X] T019 [P] [US2] Generate content for Chapter 6: Unity Integration for Robotics in `docs/06-unity-integration.md`.
- [X] T020 [P] [US2] Generate content for Chapter 7: Sensor Simulation in `docs/07-sensor-simulation.md`.
- [X] T021 [P] [US2] Create architecture diagram for Module 2 in `static/img/module2-architecture.svg`.
- [X] T022 [P] [US2] Create data flow diagram for Gazebo-to-Unity streaming in `static/img/module2-gazebo-unity-flow.svg`.
- [ ] T023 [P] [US2] Develop code examples for Module 2 in `src/module2/`.
- [X] T024 [P] [US2] Develop auto-gradable assessment project for Module 2 in `assessments/module2/`.
- [X] T025 [US2] Verify all code examples for Module 2 are runnable.
- [X] T026 [US2] Verify all chapters for Module 2 meet citation and quality gates.

---

## Phase 5: User Story 3 - Developing an AI Brain for a Robot (NVIDIA Isaac) (Priority: P3)

**Goal**: Students can use an AI robotics platform for perception and navigation.
**Independent Test**: Student can use Isaac Sim for object detection and navigation.

### Implementation for User Story 3

- [ ] T027 [US3] Research NVIDIA Isaac best practices, update `research/research.md`.
- [X] T028 [P] [US3] Generate content for Chapter 8: NVIDIA Isaac Sim Introduction in `docs/08-isaac-sim-intro.md`.
- [X] T029 [P] [US3] Generate content for Chapter 9: Isaac ROS and VSLAM in `docs/09-isaac-ros-vslam.md`.
- [X] T030 [P] [US3] Generate content for Chapter 10: Nav2 Path Planning in `docs/10-nav2-path-planning.md`.
- [X] T031 [P] [US3] Create architecture diagram for Module 3 in `static/img/module3-architecture.svg`.
- [X] T032 [P] [US3] Develop code examples for Module 3 in `src/module3/`.
- [X] T033 [P] [US3] Develop auto-gradable assessment project for Module 3 in `assessments/module3/`.
- [X] T034 [US3] Verify all code examples for Module 3 are runnable.
- [X] T035 [US3] Verify all chapters for Module 3 meet citation and quality gates.

---

## Phase 6: User Story 4 - Creating a Voice-Controlled Robot (VLA) (Priority: P4)

**Goal**: Students can integrate a VLA model to control a robot with voice commands.
**Independent Test**: Student can issue a voice command and see the robot plan and execute the task.

### Implementation for User Story 4

- [ ] T036 [US4] Research Whisper and LLM planning techniques, update `research/research.md`.
- [X] T037 [P] [US4] Generate content for Chapter 11: Voice-to-Action with Whisper in `docs/11-vla-whisper.md`.
- [X] T038 [P] [US4] Generate content for Chapter 12: Cognitive Planning with LLMs in `docs/12-vla-llm-planning.md`.
- [X] T039 [P] [US4] Generate content for Chapter 13: Capstone: Autonomous Humanoid Robot in `docs/13-capstone-project.md`.
- [X] T040 [P] [US4] Create architecture diagram for Module 4 in `static/img/module4-architecture.svg`.
- [X] T041 [P] [US4] Create sequence diagram for the full VLA voice-to-action pipeline in `static/img/module4-vla-sequence.svg`.
- [X] T042 [P] [US4] Develop code examples for Module 4 in `src/module4/`.
- [X] T043 [P] [US4] Develop final capstone project code and assets in `src/capstone/`.
- [X] T044 [US4] Verify all code examples for Module 4 and the capstone are runnable.
- [X] T045 [US4] Verify all chapters for Module 4 meet citation and quality gates.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, validation, and deployment.

- [X] T046 [P] Update all documentation based on implementation in `docs/`.
- [X] T047 [P] Code cleanup and refactoring across all `src/` directories.
- [X] T048 [P] Validate `quickstart.md` against the final codebase.
- [X] T049 Build and test the full Docusaurus website.
- [X] T050 Develop and test the RAG search backend.
- [X] T051 Final validation against all Quality Gates from `plan.md`.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must complete before all other phases.
- **Phase 2 (Foundational)** must complete before Phases 3-6.
- **Phases 3-6 (User Stories)** can be worked on in parallel after Phase 2 is complete.
- **Phase 7 (Polish)** depends on the completion of all user story phases.

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently.

### Incremental Delivery

1. Complete Setup + Foundational.
2. Add User Story 1 → Test independently.
3. Add User Story 2 → Test independently.
4. Add User Story 3 → Test independently.
5. Add User Story 4 → Test independently.
6. Complete Polish phase.
