---
id: 0002
title: Module 1 Implementation Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ros2-module
branch: 001-ros2-module
user: unknown
command: /sp.plan
labels: ["ros2", "textbook", "architecture", "rag", "planning", "deployment"]
links:
  spec: specs/001-ros2-module/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ros2-module/plan.md
tests:
  - N/A (planning document)
---

## Prompt

```text
/sp.plan

Create:
- Architecture sketch for Docusaurus book + RAG backend
- Module → chapter outline
- Integration plan for chatbot, auth, personalization, translation
- Deployment workflow (frontend + backend)

Document decisions:
- Book structure style
- Code/diagram formatting rules
- Sidebar/navigation layout
- Versioning + update strategy
- RAG pipeline design (chunking, embeddings, storage)
- Auth + onboarding flow
- Personalization + translation approach

Testing strategy:
- Docusaurus build check
- Formatting consistency (MDX, code blocks)
- RAG accuracy + selected-text behavior
- Technical correctness vs official docs

Technical details:
- Write iteratively (module-by-module)
- Follow constitution rules
- Phases: Structure → Drafting → Integration → Review
```

## Response snapshot

Successfully created comprehensive implementation plan for Module 1 and the overall AI-Native Textbook + RAG Chatbot system.

**Architecture**:
- **Frontend**: Docusaurus 3.x (TypeScript/MDX) → GitHub Pages
- **Backend**: FastAPI (Python 3.11+) → Vercel/Railway
- **Data Layer**: Qdrant (vectors), Neon Postgres (metadata/users)
- **Services**: RAG chatbot, Better-Auth, personalization, Urdu translation, 5 Claude Subagents

**Book Structure**:
- Module → chapter hierarchy (module-1-ros2/chapter-N-title.mdx)
- Consistent MDX template: personalization banner, translation toggle, diagrams, code blocks with download, quizzes, key takeaways
- Sidebar auto-generated with category grouping

**RAG Pipeline**:
- **Chunking**: 512 tokens, semantic boundaries (## headings), 50-token overlap
- **Embedding**: OpenAI text-embedding-3-small (1536 dims)
- **Storage**: Qdrant collection with metadata (module, chapter, type, keywords)
- **Selected Text**: Passed as primary context for targeted Q&A
- **Re-ranking**: Two-stage retrieval (top 20 → top 3-5)

**Claude Subagents** (5 implemented):
1. Content Generator - Draft chapter sections from outlines
2. Code Explainer - Generate inline comments and explanations
3. Diagram Generator - Create Mermaid diagrams from descriptions
4. Exercise Generator - Create quizzes and hands-on exercises
5. Learning Path Recommender - Personalized recommendations based on user profile

**Auth & Personalization**:
- Better-Auth with email/password + onboarding (hardware/software background questions)
- Personalization levels: beginner/intermediate/advanced paths with adaptive content
- User progress tracking in Neon Postgres

**Translation**:
- Pre-translation (Google Translate API) during deployment
- Urdu translation stored in Postgres, excludes code blocks and technical terms
- Toggle UI component with user preference persistence

**Testing**:
- Docusaurus build validation (CI/CD)
- ROS 2 code examples tested in Humble Docker container
- URDF validation with check_urdf + RViz2
- RAG accuracy target: >80% (keyword presence + source relevance)
- Manual checklists for formatting consistency and technical correctness

**Deployment**:
- Frontend: GitHub Actions → GitHub Pages (automatic on push to main)
- Backend: Vercel/Railway (manual deployment with environment variables)
- Embedding generation: Post-deployment batch script (~5-10 min for Module 1)

**5-Phase Structure**:
0. Research & Setup (1-2 days)
1. Content Drafting (5-7 days)
2. Backend Integration (7-10 days)
3. Frontend Integration (5-7 days)
4. Review & Polish (3-5 days)
5. Deployment (2-3 days)

**7 Architectural Decisions Documented**:
- Docusaurus for frontend (MDX support, docs features)
- Qdrant for vector storage (Python SDK, metadata filtering)
- Neon Postgres for metadata (serverless, free tier)
- Better-Auth for authentication (lightweight, self-hosted)
- Google Translate API for Urdu (supports Urdu, high quality)
- Pre-translation strategy (faster UX, lower API costs)
- Module-first content structure (learning progression)

**Constitution Check**: All 6 principles validated ✅

## Outcome

- ✅ Impact: Complete implementation plan for Module 1 and full system architecture; ready for task generation
- 🧪 Tests: Testing strategy defined (build checks, RAG accuracy, code validation, formatting consistency)
- 📁 Files: 1 created (plan.md - comprehensive 800+ line plan)
- 🔁 Next prompts: `/sp.tasks` to generate actionable tasks from plan; `/sp.adr` if formal ADRs needed for key architectural decisions
- 🧠 Reflection: Plan successfully balances hackathon constraints (time, scope) with constitution requirements (quality, completeness). Modular architecture enables parallel development (content + backend + frontend). Clear phase structure with quality gates ensures iterative progress validation. Risk mitigations address key challenges (ROS 2 installation, RAG accuracy, deployment complexity).

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution check - PASS (all 6 principles validated)
- Prompt variant (if applicable): N/A (initial plan)
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to break down 5 phases into granular, parallelizable tasks organized by user story (P1: Understanding → P2: Running Examples → P3: URDF Modeling)
