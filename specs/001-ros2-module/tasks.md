---
description: "Task list for Module 1 - The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-module/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are OPTIONAL - not included in this task list as they were not explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend (Docusaurus)**: `my-website/` at repository root
- **Backend (FastAPI)**: `backend/` at repository root
- **Content source**: `content-source/module-1/` for drafts
- **Infrastructure**: `.github/workflows/` for CI/CD

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Docusaurus project structure in my-website/
- [ ] T002 Create FastAPI backend structure in backend/
- [ ] T003 [P] Configure package.json dependencies for Docusaurus in my-website/package.json
- [ ] T004 [P] Configure requirements.txt for backend in backend/requirements.txt
- [ ] T005 [P] Create .env.example file in backend/.env.example with all required environment variables
- [ ] T006 [P] Configure ESLint and Prettier for frontend in my-website/.eslintrc.js
- [ ] T007 [P] Configure Black and Flake8 for backend in backend/pyproject.toml
- [ ] T008 Create GitHub Actions workflow for frontend deployment in .github/workflows/deploy-frontend.yml
- [ ] T009 Create content-source directory structure in content-source/module-1/
- [ ] T010 [P] Configure Docusaurus config in my-website/docusaurus.config.ts (site title, GitHub Pages URL, navbar)
- [ ] T011 [P] Configure sidebar structure in my-website/sidebars.ts with Module 1 category

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T012 Create FastAPI app entry point in backend/src/main.py
- [ ] T013 Create config module in backend/src/config.py for environment variable loading
- [ ] T014 [P] Implement Qdrant client initialization in backend/src/db/qdrant_client.py
- [ ] T015 [P] Implement Neon Postgres client initialization in backend/src/db/postgres_client.py
- [ ] T016 Create Postgres schema for user profiles in backend/src/db/schema.sql
- [ ] T017 Create Postgres schema for content metadata in backend/src/db/schema.sql
- [ ] T018 Create Postgres schema for chat history in backend/src/db/schema.sql
- [ ] T019 Create Postgres schema for user progress in backend/src/db/schema.sql
- [ ] T020 [P] Implement text embedding utilities in backend/src/utils/embedding.py
- [ ] T021 [P] Implement content chunking utilities in backend/src/utils/chunking.py (512 tokens, semantic boundaries, 50-token overlap)
- [ ] T022 [P] Implement structured logging in backend/src/utils/logging.py

### Authentication & User Management

- [ ] T023 Create User Pydantic schema in backend/src/models/user.py
- [ ] T024 Implement Better-Auth integration in backend/src/routers/auth.py (signup, login, logout endpoints)
- [ ] T025 Implement auth service in backend/src/services/auth_service.py
- [ ] T026 Create onboarding page UI in my-website/src/pages/onboarding.tsx
- [ ] T027 Implement onboarding form logic (hardware/software background questions) in my-website/src/pages/onboarding.tsx

### RAG Chatbot Foundation

- [ ] T028 Create Chat Pydantic schema in backend/src/models/chat.py
- [ ] T029 Create Content metadata Pydantic schema in backend/src/models/content.py
- [ ] T030 Implement RAG query endpoint in backend/src/routers/rag.py (POST /api/rag/query)
- [ ] T031 Implement RAG service in backend/src/services/rag_service.py (embedding, retrieval, re-ranking logic)
- [ ] T032 Implement selected text feature in backend/src/services/rag_service.py
- [ ] T033 Create RAGChatbot React component in my-website/src/components/RAGChatbot.tsx
- [ ] T034 Integrate RAGChatbot component with backend API in my-website/src/components/RAGChatbot.tsx

### Personalization Foundation

- [ ] T035 Create Recommendation Pydantic schema in backend/src/models/recommendation.py
- [ ] T036 Implement personalization endpoints in backend/src/routers/personalization.py (GET /api/personalization/recommendations)
- [ ] T037 Implement personalization service in backend/src/services/personalization_service.py (beginner/intermediate/advanced path logic)
- [ ] T038 Create PersonalizationBanner React component in my-website/src/components/PersonalizationBanner.tsx
- [ ] T039 Integrate PersonalizationBanner with backend API in my-website/src/components/PersonalizationBanner.tsx

### Translation Foundation

- [ ] T040 Implement translation endpoint in backend/src/routers/translation.py (POST /api/translation/translate)
- [ ] T041 Implement translation service in backend/src/services/translation_service.py (Google Translate API wrapper)
- [ ] T042 Create TranslationToggle React component in my-website/src/components/TranslationToggle.tsx
- [ ] T043 Integrate TranslationToggle with backend API in my-website/src/components/TranslationToggle.tsx

### Reusable UI Components

- [ ] T044 [P] Create CodeBlock React component with copy button in my-website/src/components/CodeBlock.tsx
- [ ] T045 [P] Create DiagramViewer React component for Mermaid diagrams in my-website/src/components/DiagramViewer.tsx
- [ ] T046 [P] Create base QuizComponent React component in my-website/src/components/QuizComponent.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understanding ROS 2 Communication Model (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 1 (Middleware) and Chapter 2 (Nodes, Topics, Services) content to teach ROS 2 communication concepts

**Independent Test**: Learner can correctly distinguish between topics and services when presented with 5 robot communication scenarios

### Content Drafting (Chapter 1: Introduction to ROS 2 Middleware)

- [ ] T047 [US1] Draft Chapter 1 introduction section in content-source/module-1/chapter-1-draft.md
- [ ] T048 [US1] Draft "What is Middleware?" section in content-source/module-1/chapter-1-draft.md
- [ ] T049 [US1] Draft "ROS 2 Architecture Overview" section in content-source/module-1/chapter-1-draft.md
- [ ] T050 [US1] Draft "Why ROS 2 for Robotics?" section in content-source/module-1/chapter-1-draft.md
- [ ] T051 [US1] Draft "Key Takeaways" section in content-source/module-1/chapter-1-draft.md
- [ ] T052 [US1] Create middleware architecture diagram (Mermaid syntax) in content-source/module-1/diagrams/middleware-arch.mmd
- [ ] T053 [US1] Convert Chapter 1 draft to MDX in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T054 [US1] Add frontmatter and sidebar position to my-website/docs/module-1-ros2/chapter-1-middleware.mdx

### Content Drafting (Chapter 2: Nodes, Topics, and Services)

- [ ] T055 [US1] Draft Chapter 2 introduction section in content-source/module-1/chapter-2-draft.md
- [ ] T056 [US1] Draft "Nodes: The Building Blocks" section in content-source/module-1/chapter-2-draft.md
- [ ] T057 [US1] Draft "Topics: Publish-Subscribe Pattern" section in content-source/module-1/chapter-2-draft.md
- [ ] T058 [US1] Draft "Services: Request-Response Pattern" section in content-source/module-1/chapter-2-draft.md
- [ ] T059 [US1] Draft "When to Use Topics vs Services" comparison section in content-source/module-1/chapter-2-draft.md
- [ ] T060 [US1] Draft "Key Takeaways" section in content-source/module-1/chapter-2-draft.md
- [ ] T061 [P] [US1] Create topic flow diagram (Mermaid syntax) in content-source/module-1/diagrams/topic-flow.mmd
- [ ] T062 [P] [US1] Create service flow diagram (Mermaid syntax) in content-source/module-1/diagrams/service-flow.mmd
- [ ] T063 [US1] Convert Chapter 2 draft to MDX in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx
- [ ] T064 [US1] Add frontmatter and sidebar position to my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx

### Diagrams & Visuals

- [ ] T065 [P] [US1] Export middleware architecture diagram as SVG in my-website/static/diagrams/module-1/middleware-arch.svg
- [ ] T066 [P] [US1] Export topic flow diagram as SVG in my-website/static/diagrams/module-1/topic-flow.svg
- [ ] T067 [P] [US1] Export service flow diagram as SVG in my-website/static/diagrams/module-1/service-flow.svg
- [ ] T068 [P] [US1] Add alt text to all diagrams in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T069 [P] [US1] Add alt text to all diagrams in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx

### Interactive Exercises

- [ ] T070 [US1] Create quiz questions for Chapter 1 (middleware concepts) in my-website/docs/module-1-ros2/exercises/quiz-ch1.tsx
- [ ] T071 [US1] Create quiz questions for Chapter 2 (topics vs services scenarios) in my-website/docs/module-1-ros2/exercises/quiz-ch2.tsx
- [ ] T072 [P] [US1] Embed quiz component in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T073 [P] [US1] Embed quiz component in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx

### Integration & Polish

- [ ] T074 [P] [US1] Integrate PersonalizationBanner in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T075 [P] [US1] Integrate PersonalizationBanner in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx
- [ ] T076 [P] [US1] Integrate TranslationToggle in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T077 [P] [US1] Integrate TranslationToggle in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx
- [ ] T078 [P] [US1] Integrate RAGChatbot in my-website/docs/module-1-ros2/chapter-1-middleware.mdx
- [ ] T079 [P] [US1] Integrate RAGChatbot in my-website/docs/module-1-ros2/chapter-2-nodes-topics-services.mdx
- [ ] T080 [US1] Create Module 1 category config in my-website/docs/module-1-ros2/_category_.json
- [ ] T081 [US1] Proofread Chapter 1 for clarity and technical accuracy
- [ ] T082 [US1] Proofread Chapter 2 for clarity and technical accuracy

**Checkpoint**: At this point, User Story 1 (Understanding ROS 2 Communication) should be fully functional and testable independently. Learners can read Chapters 1-2 and understand topics vs services.

---

## Phase 4: User Story 2 - Running Basic rclpy Examples (Priority: P2)

**Goal**: Create Chapter 3 hands-on section with runnable rclpy code examples (publisher, subscriber, service client, service server)

**Independent Test**: Learner can execute provided rclpy example scripts on their local machine, modify parameters, and verify changes take effect

### Content Drafting (Chapter 3: Part 1 - rclpy Examples)

- [ ] T083 [US2] Draft Chapter 3 introduction section in content-source/module-1/chapter-3-draft.md
- [ ] T084 [US2] Draft "Setting Up ROS 2 Workspace" section in content-source/module-1/chapter-3-draft.md
- [ ] T085 [US2] Draft "Creating a Publisher" section in content-source/module-1/chapter-3-draft.md
- [ ] T086 [US2] Draft "Creating a Subscriber" section in content-source/module-1/chapter-3-draft.md
- [ ] T087 [US2] Draft "Creating a Service Server" section in content-source/module-1/chapter-3-draft.md
- [ ] T088 [US2] Draft "Creating a Service Client" section in content-source/module-1/chapter-3-draft.md
- [ ] T089 [US2] Draft "Running and Testing Examples" section in content-source/module-1/chapter-3-draft.md

### rclpy Code Examples

- [ ] T090 [P] [US2] Create minimal_publisher.py in my-website/static/code-examples/module-1/minimal_publisher.py with inline comments
- [ ] T091 [P] [US2] Create minimal_subscriber.py in my-website/static/code-examples/module-1/minimal_subscriber.py with inline comments
- [ ] T092 [P] [US2] Create service_server.py in my-website/static/code-examples/module-1/service_server.py with inline comments
- [ ] T093 [P] [US2] Create service_client.py in my-website/static/code-examples/module-1/service_client.py with inline comments

### Code Example Validation

- [ ] T094 [US2] Set up ROS 2 Humble Docker container for testing in backend/tests/validation/Dockerfile
- [ ] T095 [US2] Test minimal_publisher.py execution in ROS 2 Humble environment
- [ ] T096 [US2] Test minimal_subscriber.py execution in ROS 2 Humble environment
- [ ] T097 [US2] Test service_server.py execution in ROS 2 Humble environment
- [ ] T098 [US2] Test service_client.py execution in ROS 2 Humble environment
- [ ] T099 [US2] Create validation script for all code examples in backend/tests/validation/validate_ros2_examples.sh

### MDX Integration

- [ ] T100 [US2] Convert Chapter 3 rclpy section to MDX in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T101 [US2] Embed minimal_publisher.py with CodeBlock component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T102 [US2] Embed minimal_subscriber.py with CodeBlock component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T103 [US2] Embed service_server.py with CodeBlock component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T104 [US2] Embed service_client.py with CodeBlock component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T105 [US2] Add download links for all code examples in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx

### Interactive Features

- [ ] T106 [P] [US2] Integrate PersonalizationBanner in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T107 [P] [US2] Integrate TranslationToggle in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T108 [P] [US2] Integrate RAGChatbot in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx

**Checkpoint**: At this point, User Story 2 (Running rclpy Examples) should be fully functional. Learners can download and run all code examples successfully.

---

## Phase 5: User Story 3 - Reading and Writing Simple URDF Files (Priority: P3)

**Goal**: Create Chapter 3 URDF section with simple robot model examples that learners can validate and visualize

**Independent Test**: Learner can create a URDF file for a simple 3-link robot arm, validate it, and visualize it in RViz2

### Content Drafting (Chapter 3: Part 2 - URDF Basics)

- [ ] T109 [US3] Draft "What is URDF?" section in content-source/module-1/chapter-3-draft.md
- [ ] T110 [US3] Draft "URDF Structure Overview" section in content-source/module-1/chapter-3-draft.md
- [ ] T111 [US3] Draft "Links and Joints" section in content-source/module-1/chapter-3-draft.md
- [ ] T112 [US3] Draft "Creating a Simple Humanoid Robot URDF" section in content-source/module-1/chapter-3-draft.md
- [ ] T113 [US3] Draft "Validating URDF Files" section in content-source/module-1/chapter-3-draft.md
- [ ] T114 [US3] Draft "Visualizing in RViz2" section in content-source/module-1/chapter-3-draft.md

### URDF Examples

- [ ] T115 [US3] Create simple_robot.urdf (3-link humanoid robot: torso, upper arm, forearm) in my-website/static/code-examples/module-1/simple_robot.urdf
- [ ] T116 [US3] Add detailed XML comments explaining each link element in my-website/static/code-examples/module-1/simple_robot.urdf
- [ ] T117 [US3] Add detailed XML comments explaining each joint element in my-website/static/code-examples/module-1/simple_robot.urdf

### URDF Validation

- [ ] T118 [US3] Test simple_robot.urdf with check_urdf tool in ROS 2 Humble environment
- [ ] T119 [US3] Test simple_robot.urdf visualization in RViz2
- [ ] T120 [US3] Create URDF validation script in backend/tests/validation/validate_urdf.sh

### Diagrams & Visuals

- [ ] T121 [US3] Create URDF link-joint tree diagram in content-source/module-1/diagrams/urdf-tree.mmd
- [ ] T122 [US3] Export URDF tree diagram as SVG in my-website/static/diagrams/module-1/urdf-tree.svg
- [ ] T123 [US3] Add alt text to URDF diagram

### MDX Integration

- [ ] T124 [US3] Convert Chapter 3 URDF section to MDX in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T125 [US3] Embed simple_robot.urdf with CodeBlock component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T126 [US3] Add download link for simple_robot.urdf in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T127 [US3] Embed URDF tree diagram in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx

### Interactive Exercises

- [ ] T128 [US3] Create quiz questions for Chapter 3 (URDF concepts) in my-website/docs/module-1-ros2/exercises/quiz-ch3.tsx
- [ ] T129 [US3] Embed quiz component in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx

### Finalization

- [ ] T130 [US3] Draft "Key Takeaways" section for Chapter 3 in my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T131 [US3] Add frontmatter and sidebar position to my-website/docs/module-1-ros2/chapter-3-rclpy-urdf.mdx
- [ ] T132 [US3] Proofread Chapter 3 for clarity and technical accuracy

**Checkpoint**: All user stories should now be independently functional. Learners can read all chapters, run code examples, and create URDF files.

---

## Phase 6: Claude Subagents Implementation

**Purpose**: Implement 5 Claude Subagents for reusable intelligence

### Subagent 1: Content Generator

- [ ] T133 [P] Implement Content Generator Subagent in backend/src/subagents/content_generator.py
- [ ] T134 [P] Create Content Generator endpoint in backend/src/routers/subagents.py (POST /api/subagents/generate-content)

### Subagent 2: Code Explainer

- [ ] T135 [P] Implement Code Explainer Subagent in backend/src/subagents/code_explainer.py
- [ ] T136 [P] Create Code Explainer endpoint in backend/src/routers/subagents.py (POST /api/subagents/explain-code)

### Subagent 3: Diagram Generator

- [ ] T137 [P] Implement Diagram Generator Subagent in backend/src/subagents/diagram_generator.py
- [ ] T138 [P] Create Diagram Generator endpoint in backend/src/routers/subagents.py (POST /api/subagents/generate-diagram)

### Subagent 4: Exercise Generator

- [ ] T139 [P] Implement Exercise Generator Subagent in backend/src/subagents/exercise_generator.py
- [ ] T140 [P] Create Exercise Generator endpoint in backend/src/routers/subagents.py (POST /api/subagents/generate-exercises)

### Subagent 5: Learning Path Recommender

- [ ] T141 [P] Implement Learning Path Recommender Subagent in backend/src/subagents/learning_path.py
- [ ] T142 [P] Create Learning Path Recommender endpoint in backend/src/routers/subagents.py (GET /api/subagents/learning-path)

### Subagent Orchestration

- [ ] T143 Implement Subagent service orchestration in backend/src/services/subagent_service.py
- [ ] T144 Add error handling and logging to all Subagent endpoints

---

## Phase 7: RAG Pipeline & Content Indexing

**Purpose**: Generate embeddings and enable RAG chatbot functionality

### Content Processing

- [ ] T145 Create embedding generation script in backend/scripts/generate_embeddings.py
- [ ] T146 Implement content fetching from deployed Docusaurus site in backend/scripts/generate_embeddings.py
- [ ] T147 Implement text extraction from MDX files in backend/scripts/generate_embeddings.py
- [ ] T148 Implement content chunking (512 tokens, semantic boundaries, 50-token overlap) in backend/scripts/generate_embeddings.py

### Vector Storage

- [ ] T149 Create Qdrant collection "textbook_content" with vector config in backend/scripts/generate_embeddings.py
- [ ] T150 Generate embeddings using OpenAI text-embedding-3-small in backend/scripts/generate_embeddings.py
- [ ] T151 Upload embeddings to Qdrant with metadata (module, chapter, section) in backend/scripts/generate_embeddings.py
- [ ] T152 Store content metadata in Neon Postgres in backend/scripts/generate_embeddings.py

### RAG Query Enhancement

- [ ] T153 Implement two-stage retrieval (similarity search + re-ranking) in backend/src/services/rag_service.py
- [ ] T154 Implement context assembly for OpenAI ChatCompletion in backend/src/services/rag_service.py
- [ ] T155 Add source attribution to RAG responses in backend/src/services/rag_service.py

---

## Phase 8: Translation & Personalization

**Purpose**: Implement Urdu translation and personalized learning paths

### Translation

- [ ] T156 Implement pre-translation script for Chapter 1 in backend/scripts/translate_content.py
- [ ] T157 Implement pre-translation script for Chapter 2 in backend/scripts/translate_content.py
- [ ] T158 Implement pre-translation script for Chapter 3 in backend/scripts/translate_content.py
- [ ] T159 Store translations in Postgres content_translations table
- [ ] T160 Implement translation fallback (on-demand if pre-translation missing) in backend/src/services/translation_service.py

### Personalization

- [ ] T161 Implement beginner learning path logic in backend/src/services/personalization_service.py
- [ ] T162 Implement intermediate learning path logic in backend/src/services/personalization_service.py
- [ ] T163 Implement advanced learning path logic in backend/src/services/personalization_service.py
- [ ] T164 Implement user progress tracking in backend/src/services/personalization_service.py
- [ ] T165 Create user dashboard page in my-website/src/pages/dashboard.tsx

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Quality assurance, testing, and deployment preparation

### Code Quality

- [ ] T166 [P] Run ESLint on all frontend code and fix issues
- [ ] T167 [P] Run Black on all backend code and fix formatting
- [ ] T168 [P] Run Flake8 on all backend code and fix linting issues
- [ ] T169 [P] Add TypeScript type annotations where missing in frontend components
- [ ] T170 [P] Add Python type hints where missing in backend services

### Technical Accuracy Validation

- [ ] T171 Run all rclpy code examples in ROS 2 Humble Docker container
- [ ] T172 Run check_urdf validation on simple_robot.urdf
- [ ] T173 Test simple_robot.urdf visualization in RViz2
- [ ] T174 Cross-reference Chapter 1 explanations with ROS 2 official docs
- [ ] T175 Cross-reference Chapter 2 explanations with ROS 2 official docs
- [ ] T176 Cross-reference Chapter 3 explanations with ROS 2 official docs
- [ ] T177 Verify no deprecated ROS 2 APIs used in code examples

### RAG Testing

- [ ] T178 Create RAG test dataset with 10 sample questions in backend/tests/rag/test_queries.json
- [ ] T179 Implement RAG accuracy validation script in backend/tests/rag/test_accuracy.py
- [ ] T180 Run RAG accuracy tests and verify >80% accuracy
- [ ] T181 Test selected text feature with 5 scenarios
- [ ] T182 Collect and analyze RAG chatbot response quality

### Content Quality

- [ ] T183 [P] Proofread all chapters for grammar and clarity
- [ ] T184 [P] Verify all code blocks have language tags
- [ ] T185 [P] Verify all diagrams have alt text
- [ ] T186 [P] Verify all technical terms are bolded on first use
- [ ] T187 [P] Verify consistent heading hierarchy in all chapters
- [ ] T188 Verify all downloadable code files are present in /static/code-examples/

### Build Validation

- [ ] T189 Run Docusaurus build (npm run build in my-website/)
- [ ] T190 Verify no broken internal links in Docusaurus build
- [ ] T191 Verify all MDX files parse successfully
- [ ] T192 Verify sidebar navigation renders correctly
- [ ] T193 Test local Docusaurus preview (npm run serve)

### Documentation

- [ ] T194 Create quickstart.md with local dev setup instructions in specs/001-ros2-module/quickstart.md
- [ ] T195 Create deployment documentation in specs/001-ros2-module/quickstart.md
- [ ] T196 Create API contract documentation for RAG endpoint in specs/001-ros2-module/contracts/rag-api.md
- [ ] T197 Create API contract documentation for auth endpoint in specs/001-ros2-module/contracts/auth-api.md
- [ ] T198 Create API contract documentation for personalization endpoint in specs/001-ros2-module/contracts/personalization-api.md
- [ ] T199 Create API contract documentation for translation endpoint in specs/001-ros2-module/contracts/translation-api.md
- [ ] T200 Create data model documentation in specs/001-ros2-module/data-model.md

---

## Phase 10: Deployment

**Purpose**: Deploy frontend to GitHub Pages and backend to Vercel/Railway

### Frontend Deployment (GitHub Pages)

- [ ] T201 Configure GitHub Pages settings in repository
- [ ] T202 Update Docusaurus config with correct baseUrl for GitHub Pages in my-website/docusaurus.config.ts
- [ ] T203 Test GitHub Actions workflow for frontend deployment
- [ ] T204 Deploy frontend to GitHub Pages (git push to main)
- [ ] T205 Verify frontend is accessible at GitHub Pages URL

### Backend Deployment (Vercel or Railway)

- [ ] T206 Create vercel.json configuration file in backend/vercel.json
- [ ] T207 Configure environment variables in Vercel/Railway dashboard
- [ ] T208 Deploy backend to Vercel/Railway
- [ ] T209 Verify backend API is accessible at deployment URL
- [ ] T210 Test all API endpoints on deployed backend

### Post-Deployment

- [ ] T211 Run embedding generation script targeting deployed frontend URL
- [ ] T212 Verify Qdrant collection populated with Module 1 content
- [ ] T213 Verify Postgres tables populated with metadata
- [ ] T214 Test RAG chatbot on deployed site
- [ ] T215 Test onboarding flow on deployed site
- [ ] T216 Test personalization recommendations on deployed site
- [ ] T217 Test Urdu translation on deployed site
- [ ] T218 Test all 5 Claude Subagents on deployed backend
- [ ] T219 Run end-to-end smoke tests on deployed site
- [ ] T220 Create deployment checklist for future updates

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (Phase 3): Can start after Foundational
  - User Story 2 (Phase 4): Can start after Foundational (parallel with US1)
  - User Story 3 (Phase 5): Can start after Foundational (parallel with US1 and US2)
- **Subagents (Phase 6)**: Can proceed in parallel with User Stories
- **RAG Pipeline (Phase 7)**: Depends on Foundational + at least one User Story with content
- **Translation & Personalization (Phase 8)**: Depends on User Stories completion
- **Polish (Phase 9)**: Depends on all user stories being complete
- **Deployment (Phase 10)**: Depends on Polish completion

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1 and US2

**IMPORTANT**: All three user stories are independently implementable and testable. They can be worked on in parallel by different developers.

### Within Each User Story

- Content drafting before MDX conversion
- Code examples written and validated before MDX embedding
- Diagrams created before MDX integration
- Interactive components integrated after MDX content is complete

### Parallel Opportunities

- **Setup Phase**: T003, T004, T005, T006, T007, T010, T011 can run in parallel
- **Foundational Phase**: T014, T015, T020, T021, T022 can run in parallel
- **User Story 1**: T061, T062, T065-T069, T072-T073, T074-T079 can run in parallel
- **User Story 2**: T090-T093 can run in parallel; T106-T108 can run in parallel
- **User Story 3**: Diagrams and validation can run in parallel with content drafting
- **Subagents**: T133-T142 can all run in parallel (independent implementations)
- **Polish Phase**: T166-T170, T183-T188 can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch content drafting tasks in parallel:
Task: "Draft Chapter 1 introduction in content-source/module-1/chapter-1-draft.md"
Task: "Draft Chapter 2 introduction in content-source/module-1/chapter-2-draft.md"

# Launch diagram creation tasks in parallel:
Task: "Create topic flow diagram in content-source/module-1/diagrams/topic-flow.mmd"
Task: "Create service flow diagram in content-source/module-1/diagrams/service-flow.mmd"

# Launch integration tasks in parallel:
Task: "Integrate PersonalizationBanner in chapter-1-middleware.mdx"
Task: "Integrate PersonalizationBanner in chapter-2-nodes-topics-services.mdx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Understanding ROS 2 Communication)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy Chapter 1 & 2 for early feedback

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy (MVP: Chapters 1-2)
3. Add User Story 2 → Test independently → Deploy (MVP+ : Add Chapter 3 rclpy examples)
4. Add User Story 3 → Test independently → Deploy (Complete: Add URDF section)
5. Add Subagents + RAG + Translation → Full feature set
6. Polish and deploy final version

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Chapters 1-2)
   - Developer B: User Story 2 (Chapter 3 rclpy section)
   - Developer C: User Story 3 (Chapter 3 URDF section)
   - Developer D: Subagents implementation
3. Stories complete and integrate independently
4. Team converges for RAG pipeline and deployment

---

## Notes

- **[P] tasks**: Different files, no dependencies - can run in parallel
- **[Story] label**: Maps task to specific user story for traceability (US1, US2, US3)
- Each user story should be independently completable and testable
- Verify technical accuracy before marking content tasks complete
- Test all code examples in ROS 2 Humble environment before committing
- Run Docusaurus build after each MDX file addition to catch errors early
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **Avoid**: Vague tasks, same file conflicts, cross-story dependencies that break independence

**Total Task Count**: 220 tasks
**Task Count per User Story**:
- Setup: 11 tasks
- Foundational: 35 tasks
- User Story 1 (P1): 36 tasks
- User Story 2 (P2): 26 tasks
- User Story 3 (P3): 24 tasks
- Subagents: 12 tasks
- RAG Pipeline: 11 tasks
- Translation & Personalization: 10 tasks
- Polish: 35 tasks
- Deployment: 20 tasks

**Parallel Opportunities**: 60+ tasks can run in parallel across different phases
**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1 only) = 82 tasks
