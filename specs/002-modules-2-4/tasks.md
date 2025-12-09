# Tasks: Complete Textbook (Modules 1-4) + Platform

**Input**: Design documents from `/specs/002-modules-2-4/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md (complete)

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are excluded. Focus on implementation and deployment.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing. User stories map to modules: US1 = Module 2 (Gazebo/Unity), US2 = Module 3 (Isaac), US3 = Module 4 (VLA).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/module this task belongs to (e.g., US1=Module2, US2=Module3, US3=Module4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `my-website/docs/`, `my-website/src/`
- **Backend**: `backend/src/`, `backend/tests/`
- **Content Source**: `content-source/module-{1,2,3,4}/`
- **CI/CD**: `.github/workflows/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure (already partially complete from previous work)

**Status**: ✅ Most setup tasks already completed in previous session. Remaining tasks below.

- [ ] T001 [P] Create Module 2 directory structure in my-website/docs/module-2/
- [ ] T002 [P] Create Module 3 directory structure in my-website/docs/module-3/
- [ ] T003 [P] Create Module 4 directory structure in my-website/docs/module-4/
- [ ] T004 [P] Create content-source directories for modules 2-4 (diagrams, exercises)
- [ ] T005 Update sidebars.ts to include Module 2, 3, 4 navigation structure
- [ ] T006 [P] Create backend database models in backend/src/models/user.py
- [ ] T007 [P] Create backend database models in backend/src/models/chat.py
- [ ] T008 [P] Create backend database models in backend/src/models/content.py
- [ ] T009 [P] Create backend database models in backend/src/models/subagent.py
- [ ] T010 Create backend database connection utilities in backend/src/db/postgres.py
- [ ] T011 [P] Create backend vector database connection in backend/src/db/qdrant.py
- [ ] T012 Create database schema initialization script in backend/scripts/setup_db.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core backend infrastructure that MUST be complete before user stories (modules) can be fully functional

**⚠️ CRITICAL**: Backend foundation enables RAG, auth, personalization, translation for ALL modules

- [ ] T013 Implement OpenAI embedding service in backend/src/services/embedding.py
- [ ] T014 Implement Qdrant vector store service in backend/src/services/vector_store.py
- [ ] T015 Implement text chunking utility in backend/src/utils/chunking.py (512 tokens, 50 overlap, Markdown-aware)
- [ ] T016 Create content embedding script in backend/scripts/embed_content.py
- [ ] T017 Implement Better-Auth integration in backend/src/routers/auth.py
- [ ] T018 [P] Implement Better-Auth utilities in backend/src/utils/auth.py
- [ ] T019 Implement OpenAI ChatKit integration in backend/src/services/chat.py
- [ ] T020 Implement RAG chatbot router in backend/src/routers/rag.py
- [ ] T021 Implement Google Translate service in backend/src/services/translator.py
- [ ] T022 Implement translation router in backend/src/routers/translation.py
- [ ] T023 Create pre-translation script in backend/scripts/translate_content.py
- [ ] T024 Implement personalization engine in backend/src/services/personalization_engine.py
- [ ] T025 Implement personalization router in backend/src/routers/personalization.py
- [ ] T026 [P] Create Claude Subagent: Content Generator in backend/src/subagents/content_generator.py
- [ ] T027 [P] Create Claude Subagent: Code Explainer in backend/src/subagents/code_explainer.py
- [ ] T028 [P] Create Claude Subagent: Diagram Generator in backend/src/subagents/diagram_generator.py
- [ ] T029 [P] Create Claude Subagent: Exercise Generator in backend/src/subagents/exercise_generator.py
- [ ] T030 [P] Create Claude Subagent: Learning Path Recommender in backend/src/subagents/learning_path.py
- [ ] T031 Implement Subagent orchestration router in backend/src/routers/subagents.py
- [ ] T032 [P] Create ChatWidget React component in my-website/src/components/ChatWidget.tsx
- [ ] T033 [P] Create LanguageToggle React component in my-website/src/components/LanguageToggle.tsx
- [ ] T034 [P] Create ProgressTracker React component in my-website/src/components/ProgressTracker.tsx
- [ ] T035 Integrate ChatWidget into Docusaurus layout in my-website/src/theme/Layout/index.tsx

**Checkpoint**: Backend platform ready - modules can now be authored with full RAG/auth/translation support

---

## Phase 3: User Story 1 - Understanding Simulation and Digital Twins (Priority: P1 - Module 2) 🎯 MVP

**Goal**: Complete Module 2 (Gazebo/Unity) with 3 chapters covering simulation basics, environment building, and Unity visualization

**Independent Test**: Student can launch Gazebo simulation, modify physics parameters, build custom world, and connect Unity to ROS 2 for humanoid visualization

### Module 2 Chapter 1: Gazebo Simulation Basics

- [ ] T036 [P] [US1] Write Module 2 intro.md in my-website/docs/module-2/intro.md
- [ ] T037 [P] [US1] Write Chapter 1 overview.md in my-website/docs/module-2/chapter-1/overview.md
- [ ] T038 [P] [US1] Write physics-engines.md explaining ODE/Bullet/Simbody in my-website/docs/module-2/chapter-1/physics-engines.md
- [ ] T039 [P] [US1] Write collision-detection.md in my-website/docs/module-2/chapter-1/collision-detection.md
- [ ] T040 [P] [US1] Write sensor-simulation.md in my-website/docs/module-2/chapter-1/sensor-simulation.md
- [ ] T041 [US1] Create runnable Gazebo launch examples for Chapter 1 in content-source/module-2/exercises/chapter-1/
- [ ] T042 [US1] Create physics parameter diagrams in content-source/module-2/diagrams/chapter-1/ (export to static/diagrams/)

### Module 2 Chapter 2: Building Environments + Digital Twin

- [ ] T043 [P] [US1] Write Chapter 2 overview.md in my-website/docs/module-2/chapter-2/overview.md
- [ ] T044 [P] [US1] Write sdf-world-files.md explaining SDF syntax in my-website/docs/module-2/chapter-2/sdf-world-files.md
- [ ] T045 [P] [US1] Write model-composition.md in my-website/docs/module-2/chapter-2/model-composition.md
- [ ] T046 [P] [US1] Write digital-twin-workflow.md in my-website/docs/module-2/chapter-2/digital-twin-workflow.md
- [ ] T047 [US1] Create custom Gazebo world examples (SDF files) in content-source/module-2/exercises/chapter-2/
- [ ] T048 [US1] Create digital twin workflow diagrams in content-source/module-2/diagrams/chapter-2/

### Module 2 Chapter 3: Unity Visualization

- [ ] T049 [P] [US1] Write Chapter 3 overview.md in my-website/docs/module-2/chapter-3/overview.md
- [ ] T050 [P] [US1] Write unity-ros2-bridge.md explaining ROS-TCP-Connector in my-website/docs/module-2/chapter-3/unity-ros2-bridge.md
- [ ] T051 [P] [US1] Write humanoid-visualization.md in my-website/docs/module-2/chapter-3/humanoid-visualization.md
- [ ] T052 [P] [US1] Write interactive-scenes.md in my-website/docs/module-2/chapter-3/interactive-scenes.md
- [ ] T053 [US1] Create Unity ROS 2 integration examples (setup scripts, URDF import) in content-source/module-2/exercises/chapter-3/
- [ ] T054 [US1] Create Unity-ROS 2 architecture diagrams in content-source/module-2/diagrams/chapter-3/

### Module 2 Wrap-up

- [ ] T055 [US1] Write Module 2 exercises.md with 3 hands-on exercises in my-website/docs/module-2/exercises.md
- [ ] T056 [US1] Write Module 2 resources.md in my-website/docs/module-2/resources.md
- [ ] T057 [US1] Validate all Module 2 code examples in ROS 2 Humble + Gazebo 11 environment
- [ ] T058 [US1] Generate embeddings for Module 2 content and upload to Qdrant (run backend/scripts/embed_content.py)
- [ ] T059 [US1] Pre-translate Module 2 to Urdu (run backend/scripts/translate_content.py)

**Checkpoint**: Module 2 complete, students can understand simulation and digital twins. RAG chatbot can answer Module 2 questions.

---

## Phase 4: User Story 2 - Building Perception Pipelines with Isaac (Priority: P2 - Module 3)

**Goal**: Complete Module 3 (Isaac Sim) with 3 chapters covering Isaac setup, VSLAM, and Nav2 path planning

**Independent Test**: Student can set up Isaac Sim, generate synthetic data, run Isaac ROS VSLAM, and configure Nav2 for autonomous navigation

### Module 3 Chapter 1: Isaac Sim Setup + Synthetic Data

- [ ] T060 [P] [US2] Write Module 3 intro.md in my-website/docs/module-3/intro.md
- [ ] T061 [P] [US2] Write Chapter 1 overview.md in my-website/docs/module-3/chapter-1/overview.md
- [ ] T062 [P] [US2] Write installation.md with Isaac Sim setup instructions in my-website/docs/module-3/chapter-1/installation.md
- [ ] T063 [P] [US2] Write gpu-requirements.md with RTX 3060 specs + AWS G4dn cloud guide in my-website/docs/module-3/chapter-1/gpu-requirements.md
- [ ] T064 [P] [US2] Write synthetic-data-generation.md in my-website/docs/module-3/chapter-1/synthetic-data-generation.md
- [ ] T065 [US2] Create Isaac Sim synthetic data generation examples in content-source/module-3/exercises/chapter-1/
- [ ] T066 [US2] Create Isaac Sim setup diagrams in content-source/module-3/diagrams/chapter-1/

### Module 3 Chapter 2: Isaac ROS for VSLAM + Navigation

- [ ] T067 [P] [US2] Write Chapter 2 overview.md in my-website/docs/module-3/chapter-2/overview.md
- [ ] T068 [P] [US2] Write vslam-concepts.md explaining feature extraction/loop closure in my-website/docs/module-3/chapter-2/vslam-concepts.md
- [ ] T069 [P] [US2] Write isaac-ros-integration.md for cuVSLAM setup in my-website/docs/module-3/chapter-2/isaac-ros-integration.md
- [ ] T070 [P] [US2] Write map-building.md in my-website/docs/module-3/chapter-2/map-building.md
- [ ] T071 [US2] Create Isaac ROS VSLAM runnable examples in content-source/module-3/exercises/chapter-2/
- [ ] T072 [US2] Create VSLAM workflow diagrams in content-source/module-3/diagrams/chapter-2/

### Module 3 Chapter 3: Nav2 Path Planning

- [ ] T073 [P] [US2] Write Chapter 3 overview.md in my-website/docs/module-3/chapter-3/overview.md
- [ ] T074 [P] [US2] Write costmap-configuration.md in my-website/docs/module-3/chapter-3/costmap-configuration.md
- [ ] T075 [P] [US2] Write planner-algorithms.md explaining DWB/TEB in my-website/docs/module-3/chapter-3/planner-algorithms.md
- [ ] T076 [P] [US2] Write controller-tuning.md in my-website/docs/module-3/chapter-3/controller-tuning.md
- [ ] T077 [US2] Create Nav2 configuration examples for humanoid robots in content-source/module-3/exercises/chapter-3/
- [ ] T078 [US2] Create Nav2 path planning diagrams in content-source/module-3/diagrams/chapter-3/

### Module 3 Wrap-up

- [ ] T079 [US2] Write Module 3 exercises.md with 3 hands-on exercises in my-website/docs/module-3/exercises.md
- [ ] T080 [US2] Write Module 3 resources.md in my-website/docs/module-3/resources.md
- [ ] T081 [US2] Validate all Module 3 code examples in Isaac Sim 2023.1+ environment
- [ ] T082 [US2] Generate embeddings for Module 3 content and upload to Qdrant
- [ ] T083 [US2] Pre-translate Module 3 to Urdu

**Checkpoint**: Module 3 complete, students can build perception pipelines with Isaac. RAG chatbot can answer Module 3 questions.

---

## Phase 5: User Story 3 - Integrating Vision-Language-Action Models (Priority: P3 - Module 4)

**Goal**: Complete Module 4 (VLA) with 3 chapters covering Whisper integration, LLM action planning, and capstone pipeline

**Independent Test**: Student can integrate Whisper for voice transcription, use LLM to generate ROS 2 actions, and demonstrate end-to-end autonomous humanoid pipeline

### Module 4 Chapter 1: Voice-to-Action with Whisper

- [ ] T084 [P] [US3] Write Module 4 intro.md in my-website/docs/module-4/intro.md
- [ ] T085 [P] [US3] Write Chapter 1 overview.md in my-website/docs/module-4/chapter-1/overview.md
- [ ] T086 [P] [US3] Write whisper-integration.md explaining Whisper Small model in my-website/docs/module-4/chapter-1/whisper-integration.md
- [ ] T087 [P] [US3] Write ros2-audio-pipeline.md in my-website/docs/module-4/chapter-1/ros2-audio-pipeline.md
- [ ] T088 [P] [US3] Write transcription-accuracy.md in my-website/docs/module-4/chapter-1/transcription-accuracy.md
- [ ] T089 [US3] Create Whisper + ROS 2 integration examples in content-source/module-4/exercises/chapter-1/
- [ ] T090 [US3] Create audio pipeline diagrams in content-source/module-4/diagrams/chapter-1/

### Module 4 Chapter 2: LLM → ROS 2 Action Planning

- [ ] T091 [P] [US3] Write Chapter 2 overview.md in my-website/docs/module-4/chapter-2/overview.md
- [ ] T092 [P] [US3] Write llm-integration.md for GPT-4/Claude API in my-website/docs/module-4/chapter-2/llm-integration.md
- [ ] T093 [P] [US3] Write action-schema.md with JSON schema definition in my-website/docs/module-4/chapter-2/action-schema.md
- [ ] T094 [P] [US3] Write prompt-engineering.md with safety constraints in my-website/docs/module-4/chapter-2/prompt-engineering.md
- [ ] T095 [US3] Create LLM action planning examples with JSON schema in content-source/module-4/exercises/chapter-2/
- [ ] T096 [US3] Create LLM action schema diagrams in content-source/module-4/diagrams/chapter-2/

### Module 4 Chapter 3: Capstone - Autonomous Humanoid Pipeline

- [ ] T097 [P] [US3] Write Chapter 3 overview.md in my-website/docs/module-4/chapter-3/overview.md
- [ ] T098 [P] [US3] Write pipeline-architecture.md in my-website/docs/module-4/chapter-3/pipeline-architecture.md
- [ ] T099 [P] [US3] Write task-examples.md with 5 demonstration tasks in my-website/docs/module-4/chapter-3/task-examples.md
- [ ] T100 [P] [US3] Write error-handling.md in my-website/docs/module-4/chapter-3/error-handling.md
- [ ] T101 [US3] Create end-to-end capstone pipeline code in content-source/module-4/exercises/chapter-3/
- [ ] T102 [US3] Create capstone architecture diagrams in content-source/module-4/diagrams/chapter-3/

### Module 4 Wrap-up

- [ ] T103 [US3] Write Module 4 exercises.md with hands-on VLA exercises in my-website/docs/module-4/exercises.md
- [ ] T104 [US3] Write Module 4 resources.md in my-website/docs/module-4/resources.md
- [ ] T105 [US3] Validate all Module 4 code examples with Whisper Small + LLM API
- [ ] T106 [US3] Generate embeddings for Module 4 content and upload to Qdrant
- [ ] T107 [US3] Pre-translate Module 4 to Urdu

**Checkpoint**: Module 4 complete, students can integrate VLA models. RAG chatbot can answer Module 4 questions. All modules (1-4) now complete.

---

## Phase 6: Capstone & Cross-Cutting Content

**Purpose**: Tie all 4 modules together and add cross-cutting documentation

- [ ] T108 Write textbook intro.md in my-website/docs/intro.md
- [ ] T109 Write capstone.md with final integrated project in my-website/docs/capstone.md
- [ ] T110 [P] Create high-level architecture diagrams for full textbook in static/diagrams/
- [ ] T111 [P] Create module progression flowchart (ROS 2 → Sim → Isaac → VLA) in static/diagrams/
- [ ] T112 Update README.md with project overview and setup instructions

---

## Phase 7: Backend Testing & Validation

**Purpose**: Validate backend platform functionality

- [ ] T113 [P] Write unit tests for embedding service in backend/tests/unit/test_embedding.py
- [ ] T114 [P] Write unit tests for vector store service in backend/tests/unit/test_vector_store.py
- [ ] T115 [P] Write unit tests for chat service in backend/tests/unit/test_chat.py
- [ ] T116 [P] Write integration test for RAG pipeline in backend/tests/integration/test_rag_pipeline.py
- [ ] T117 [P] Write integration test for auth flow in backend/tests/integration/test_auth_flow.py
- [ ] T118 [P] Write contract tests for Subagent APIs in backend/tests/contract/test_subagents.py
- [ ] T119 Run full backend test suite (pytest backend/tests/)
- [ ] T120 Validate RAG chatbot responses for accuracy across all modules

---

## Phase 8: Deployment & CI/CD

**Purpose**: Deploy to production and ensure CI/CD pipelines work

- [ ] T121 Update GitHub username placeholders in my-website/docusaurus.config.ts (replace YOUR_USERNAME)
- [ ] T122 Configure Better-Auth environment variables for production in backend/.env
- [ ] T123 Configure Qdrant Cloud collection and API key in backend/.env
- [ ] T124 Configure Neon Postgres connection string in backend/.env
- [ ] T125 Deploy backend to Vercel/Railway with environment variables configured
- [ ] T126 Verify backend health endpoints (/ and /health) are accessible
- [ ] T127 Run content embedding script against production Qdrant instance
- [ ] T128 Run content translation script for Urdu pre-translation
- [ ] T129 Test frontend deployment to GitHub Pages (push to main branch)
- [ ] T130 Verify Docusaurus build succeeds in GitHub Actions
- [ ] T131 [P] Create backend testing workflow in .github/workflows/test-backend.yml
- [ ] T132 [P] Create linting workflow in .github/workflows/lint.yml
- [ ] T133 Test end-to-end RAG query from frontend → backend → Qdrant → GPT-4 → frontend
- [ ] T134 Test language toggle (English ↔ Urdu) functionality
- [ ] T135 Test authentication flow (signup, onboarding, login)
- [ ] T136 Test personalization recommendations based on user profile

---

## Phase 9: Polish & Documentation

**Purpose**: Final polish, documentation, and deployment readiness

- [ ] T137 Run frontend linter and fix issues (npm run lint:fix)
- [ ] T138 Run backend formatter and linter (black backend/src/ && flake8 backend/src/)
- [ ] T139 [P] Write quickstart.md developer guide in specs/002-modules-2-4/quickstart.md
- [ ] T140 [P] Write data-model.md documentation in specs/002-modules-2-4/data-model.md
- [ ] T141 [P] Generate OpenAPI contracts in specs/002-modules-2-4/contracts/ (rag-api.yaml, auth-api.yaml, subagents-api.yaml, translation-api.yaml)
- [ ] T142 Create deployment verification checklist
- [ ] T143 Write backend/README.md with setup and deployment instructions
- [ ] T144 Write my-website/README.md with content contribution guidelines
- [ ] T145 Validate all 12 chapters are accessible and render correctly
- [ ] T146 Validate all diagrams are exported to static/diagrams/ and display correctly
- [ ] T147 Validate all code examples are tested and include copy-paste instructions
- [ ] T148 Run accessibility audit on Docusaurus site (Lighthouse)
- [ ] T149 Run performance audit (target: <3s initial load, <500ms navigation)
- [ ] T150 Create hackathon demo script with example queries and features to showcase

---

## Summary

**Total Tasks**: 150
**Parallelizable Tasks**: 78 marked with [P]

### Tasks by Phase

| Phase | Tasks | Description |
|-------|-------|-------------|
| 1. Setup | 12 | Project initialization and directory structure |
| 2. Foundational | 23 | Backend platform (RAG, auth, translation, subagents, frontend components) |
| 3. US1 (Module 2) | 24 | Gazebo/Unity simulation content + validation |
| 4. US2 (Module 3) | 24 | Isaac perception content + validation |
| 5. US3 (Module 4) | 24 | VLA integration content + validation |
| 6. Capstone | 5 | Cross-cutting content and diagrams |
| 7. Backend Testing | 8 | Unit, integration, contract tests |
| 8. Deployment | 16 | Production deployment and CI/CD |
| 9. Polish | 14 | Documentation, validation, demo prep |

### Tasks by User Story

- **Setup + Foundational**: 35 tasks (blocking prerequisites)
- **US1 (Module 2 - Gazebo/Unity)**: 24 tasks ✅ MVP
- **US2 (Module 3 - Isaac)**: 24 tasks
- **US3 (Module 4 - VLA)**: 24 tasks
- **Cross-Cutting**: 43 tasks (capstone, testing, deployment, polish)

### Independent Test Criteria

**User Story 1 (Module 2)**:
- Can launch Gazebo world with physics enabled
- Can modify physics parameters and observe effects
- Can create custom Gazebo world with SDF files
- Can connect Unity to ROS 2 and visualize humanoid URDF

**User Story 2 (Module 3)**:
- Can install Isaac Sim and verify GPU requirements
- Can generate synthetic camera/lidar datasets
- Can run Isaac ROS cuVSLAM and build 3D map
- Can configure Nav2 for humanoid path planning

**User Story 3 (Module 4)**:
- Can integrate Whisper and transcribe voice to ROS 2 topic
- Can use LLM to generate JSON action sequences
- Can demonstrate end-to-end pipeline (voice → plan → execute)
- Can handle 5 different task types (navigate, grasp, search, follow, report)

---

## Dependency Graph (User Story Completion Order)

```
Setup (T001-T012)
    ↓
Foundational (T013-T035) ← BLOCKS all user stories
    ↓
    ├─→ US1: Module 2 (T036-T059) ✅ MVP - Independent
    ├─→ US2: Module 3 (T060-T083) - Independent (can run parallel with US1)
    └─→ US3: Module 4 (T084-T107) - Independent (can run parallel with US1/US2)
    ↓
Capstone (T108-T112) - Requires all modules complete
    ↓
Backend Testing (T113-T120) - Can run parallel with deployment
    ↓
Deployment (T121-T136)
    ↓
Polish (T137-T150)
```

---

## Parallel Execution Examples

### Phase 2 (Foundational) - High Parallelism
Can run in parallel (different services):
- T013, T014, T017, T021, T024 (backend services)
- T026, T027, T028, T029, T030 (Claude subagents - 5 parallel)
- T032, T033, T034 (frontend React components - 3 parallel)

### Phase 3 (Module 2) - Content Writing
Can run in parallel (different chapters/files):
- T036, T037, T038, T039, T040 (Chapter 1 - 5 parallel)
- T043, T044, T045, T046 (Chapter 2 - 4 parallel)
- T049, T050, T051, T052 (Chapter 3 - 4 parallel)

### Phase 4 (Module 3) - Content Writing
Same parallelism as Module 2 (different chapters/files)

### Phase 5 (Module 4) - Content Writing
Same parallelism as Module 2 (different chapters/files)

---

## Implementation Strategy

### Recommended MVP Scope (Minimum Viable Product)
To deliver a functional demo quickly:
1. **Phase 1-2**: Setup + Foundational (T001-T035) - 35 tasks
2. **Phase 3**: US1 (Module 2 only) (T036-T059) - 24 tasks
3. **Phase 8**: Minimal Deployment (T121-T133) - 13 tasks

**MVP Total**: 72 tasks
**Deliverable**: Working textbook with Module 2 + RAG chatbot + Auth + Translation

### Full Implementation Scope
All 150 tasks for complete 4-module textbook with all features

### Incremental Delivery Plan
1. **Sprint 1**: MVP (Module 2 + Platform) - 72 tasks
2. **Sprint 2**: Add Module 3 (Isaac) - 24 tasks
3. **Sprint 3**: Add Module 4 (VLA) - 24 tasks
4. **Sprint 4**: Capstone + Testing + Polish - 30 tasks

---

## Format Validation

✅ All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
✅ Task IDs sequential (T001-T150)
✅ [P] markers for parallelizable tasks (78 tasks)
✅ [Story] labels for user story tasks (US1, US2, US3)
✅ File paths included in descriptions
✅ Phases organized by user story for independent implementation
✅ Independent test criteria defined for each user story
✅ Dependency graph shows completion order
✅ Parallel execution examples provided
