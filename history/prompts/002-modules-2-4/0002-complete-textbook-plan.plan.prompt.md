---
id: 0002
title: Complete Textbook Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5
feature: 002-modules-2-4
branch: 002-modules-2-4
user: user
command: /sp.plan
labels: ["architecture", "modules", "gazebo", "unity", "isaac", "vla", "rag", "planning"]
links:
  spec: specs/002-modules-2-4/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-modules-2-4/plan.md
  - specs/002-modules-2-4/research.md
tests:
  - All 6 constitution gates passed (pre and post-design)
---

## Prompt

/sp.plan Full Book (Modules 1–4)

Create:
- High-level architecture for the complete textbook
- Module → chapter structure for all four modules
- Integration plan connecting ROS 2 → Simulation → Isaac → VLA
- Content flow ensuring smooth progression across modules

Document decisions:
- Book structure style and chapter formatting
- Code/diagram consistency across modules
- Navigation + sidebar organization
- Terminology alignment for robotics concepts
- How examples scale in complexity from Module 1 → 4

Testing strategy:
- Check clarity and continuity between modules
- Validate technical accuracy using official docs (ROS 2, Gazebo, Unity, Isaac, Whisper)
- Ensure each module builds upon the previous one logically
- Confirm all chapters meet learning goals

Technical details:
- Write using the same pattern established for Module 1
- Develop module-by-module in phases: Structure → Draft → Review → Refine
- Keep chapter scope limited to essentials; avoid deep dives outside module focus

## Response snapshot

Created comprehensive implementation plan for complete textbook covering Modules 1-4 (ROS 2, Gazebo/Unity, Isaac, VLA) with full backend platform architecture.

**Plan Structure** (656 lines):
- Summary: 4 modules, 12 chapters, progressive learning (P1→P4)
- Technical Context: TypeScript/Python, Docusaurus/FastAPI, Qdrant/Neon, performance goals
- Constitution Check: All 6 gates passed (✅ alignment, ✅ architecture, ✅ subagents, ✅ completeness, ✅ quality, ✅ deployment)
- Project Structure: Detailed directory trees for frontend (my-website), backend (FastAPI), content-source, CI/CD
- Phase 0 Research: 10 technical decisions with rationale (Gazebo 11, Unity ROS-TCP-Connector, Isaac cuVSLAM, Whisper Small, etc.)
- Phase 1 Design: Data model outline, API contracts (RAG, auth, subagents, translation), quickstart guide structure

**Research Findings** (10 decisions documented in research.md):
1. Gazebo 11 (Classic) over Fortress - better beginner docs, ROS 2 Humble support
2. Unity ROS-TCP-Connector for Module 2 Chapter 3 - official, well-documented
3. Isaac Sim GPU: RTX 3060 12GB min, AWS G4dn cloud alternative
4. Isaac ROS cuVSLAM for Module 3 - GPU-accelerated, real-time VSLAM
5. Whisper Small model (244M params) - 95% accuracy, <1s latency, 2GB VRAM
6. LLM action JSON schema - structured output with action_type/params/preconditions
7. RAG chunking: 512 tokens, 50 overlap, Markdown-aware splitting
8. Better-Auth on backend API - static site limitation workaround
9. Pre-translation workflow - instant language switching, code/term preservation
10. Qdrant metadata schema - module/chapter/section/content_type/page_url for filtered search

**Content Structure** (my-website/docs/):
- Module 1: ROS 2 Fundamentals (3 chapters, 5 sections each = 15 files)
- Module 2: Digital Twin Gazebo/Unity (3 chapters, 4 sections each = 12 files)
- Module 3: AI-Robot Brain Isaac (3 chapters, 4 sections each = 12 files)
- Module 4: Vision-Language-Action (3 chapters, 4 sections each = 12 files)
- Total: 51 MDX files + exercises + resources

**Backend Architecture** (backend/src/):
- Routers: rag.py, auth.py, personalization.py, translation.py, subagents.py
- Services: embedding.py, vector_store.py, chat.py, translator.py, personalization_engine.py
- Models: user.py, chat.py, content.py, subagent.py (Pydantic schemas)
- DB: postgres.py (Neon), qdrant.py (vectors)
- Subagents: content_generator, code_explainer, diagram_generator, exercise_generator, learning_path (5 Claude subagents)

**Integration Flow**:
```
Student → Docusaurus UI → FastAPI Backend
                              ↓
         ┌────────────────────┼────────────────────┐
         ↓                    ↓                    ↓
    Qdrant (RAG)     Better-Auth (Session)   Claude (Subagents)
         ↓                    ↓                    ↓
    Neon Postgres (Chat History + User Profiles)
```

**Technology Stack**:
- Frontend: Docusaurus 3.9.2, React 19, TypeScript 5.6+, MDX
- Backend: FastAPI 0.109, Python 3.11+
- Databases: Qdrant Cloud (1536-dim vectors), Neon Postgres (metadata)
- AI: OpenAI GPT-4 + text-embedding-3-small, Anthropic Claude (subagents), Whisper Small
- Infrastructure: GitHub Pages (frontend), Vercel/Railway (backend), GitHub Actions (CI/CD)

**Performance Goals**:
- Frontend: <3s initial load, <500ms chapter navigation
- Backend RAG: <2s query response (embed + search + generate)
- Translation: <1s chapter switch (pre-cached)
- Subagents: <5s content generation response

**Scale/Scope**:
- 4 modules, 12 chapters, ~50-60 pages
- 200-300 code snippets
- 50-80 diagrams
- 15-20 hands-on exercises
- 1000-1500 content chunks in RAG
- 100-500 expected students (hackathon demo scale)

## Outcome

- ✅ Impact: Complete architecture plan for 4-module textbook + full backend platform (RAG, auth, personalization, translation, 5 subagents)
- 🧪 Tests: 6/6 constitution gates passed pre-design, 6/6 re-validated post-design
- 📁 Files: plan.md (656 lines), research.md (10 decisions with rationale)
- 🔁 Next prompts: /sp.tasks to break down into 113-162 actionable tasks
- 🧠 Reflection: Plan balances educational content creation with backend platform development. Progressive learning structure (ROS 2 → Simulation → Isaac → VLA) mirrors real robotics workflow. All technical uncertainties resolved through research phase (Gazebo version, Unity bridge, Isaac GPU, Whisper model, LLM schema, RAG chunking, auth pattern, translation workflow, Qdrant schema). Ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed without blockers
- Graders run and results (PASS/FAIL):
  - Constitution Check (Pre-Design): 6/6 PASS
  - Constitution Check (Post-Design): 6/6 PASS
  - Research Phase: 10/10 decisions resolved
  - Overall: PASS - all gates green
- Prompt variant (if applicable): Standard /sp.plan workflow with multi-module scope
- Next experiment (smallest change to try): Consider adding specific module-to-module transition guides (brief intro at start of each module explaining how it builds on previous) to strengthen learning continuity
