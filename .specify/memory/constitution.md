# Physical AI & Humanoid Robotics Textbook + RAG Chatbot Constitution

<!--
SYNC IMPACT REPORT - Constitution v1.0.0
Version Change: Initial creation → v1.0.0
Rationale: MINOR - Initial constitution establishing project governance and principles

Modified Principles: N/A (initial creation)
Added Sections:
  - Core Principles (6 principles defined)
  - Technology Stack Standards
  - Development Workflow
  - Governance

Removed Sections: N/A (initial creation)

Templates Requiring Updates:
  ✅ .specify/templates/plan-template.md - Reviewed, aligns with constitution requirements
  ✅ .specify/templates/spec-template.md - Reviewed, aligns with user story and requirements structure
  ✅ .specify/templates/tasks-template.md - Reviewed, aligns with task categorization principles
  ✅ .claude/commands/*.md - Reviewed, no conflicts with constitution principles

Follow-up TODOs: None - all placeholders filled
-->

## Core Principles

### I. Course Alignment & Technical Accuracy

All textbook content MUST align precisely with the defined course modules: ROS 2, Gazebo/Unity simulation, Isaac Sim, and Vision-Language-Action (VLA) models. Technical explanations, code examples, and diagrams MUST be accurate, runnable as-is, and appropriate for robotics engineering learners.

**Rationale**: This is an educational product. Inaccurate or misaligned content undermines learning outcomes and user trust. Content must serve the learner's progression through Physical AI and Humanoid Robotics topics.

**Enforcement**:
- Every chapter MUST map to specific course module(s)
- All code snippets MUST be executable without modification
- Technical claims MUST be verifiable against official documentation (ROS 2, Isaac, etc.)
- Reject vague explanations; prefer concrete, testable examples

### II. Modular, Maintainable Architecture

The project structure MUST be modular and maintainable using Spec-Kit Plus development methodology combined with Docusaurus for the textbook frontend. Backend services (RAG chatbot, auth, personalization) MUST follow clear separation of concerns.

**Rationale**: Hackathon deliverables must be extensible beyond the competition. Modularity enables independent development of features (textbook content, RAG chatbot, auth, translation) and reusability of Claude Subagents.

**Enforcement**:
- Docusaurus site structure: organized by chapter/module with consistent MDX formatting
- Backend API: FastAPI with clear service boundaries (RAG, auth, personalization, translation)
- Subagents MUST be callable and reusable across different contexts
- No monolithic files; enforce single-responsibility principle
- All major features documented in `/specs/[feature]/` with spec.md, plan.md, tasks.md

### III. Reusable Intelligence via Claude Subagents

The project MUST implement at least 5 Claude Subagents that provide reusable, callable intelligence. Subagents MUST be designed for tasks like content generation, code explanation, diagram creation, exercise generation, and personalized learning paths.

**Rationale**: This is a core hackathon requirement and differentiator. Subagents transform the textbook from static content to an intelligent, adaptive learning system.

**Enforcement**:
- Minimum 5 distinct Subagents implemented and documented
- Each Subagent MUST have clear input/output contracts
- Subagents MUST be accessible via API endpoints
- Subagent functionality MUST be testable independently
- Document each Subagent's purpose, usage, and integration points

### IV. Functional Completeness: Hackathon Scoring Requirements

All deliverables MUST meet hackathon scoring criteria: base textbook features + RAG chatbot + authentication + personalization + Urdu translation. No placeholder implementations; every feature MUST be functional.

**Rationale**: Partial implementations score zero. Functional completeness ensures all scoring categories are addressed.

**Required Features**:
- **Base Textbook**: Complete MDX chapters covering all course modules + capstone
- **RAG Chatbot**: FastAPI backend, OpenAI Agents/ChatKit integration, Qdrant vectors, Neon Postgres metadata, "answer from selected text" capability
- **Authentication**: Better-Auth integration with onboarding questions (hardware + software background)
- **Personalization**: User profiles, adaptive content recommendations based on background
- **Urdu Translation**: Per-chapter translation toggle, functional translation service
- **Claude Subagents**: Minimum 5 implemented and integrated

### V. Consistent Code Quality & Runnable Examples

All code examples in the textbook and all backend services MUST be runnable as-is. Code MUST follow consistent style conventions, include proper error handling, and be production-ready (not pseudocode or incomplete snippets).

**Rationale**: Learners will copy/paste code examples. Broken or incomplete code damages credibility and learning outcomes. Production-ready backend code ensures deployment viability.

**Enforcement**:
- Every code example MUST be tested before inclusion
- Backend code MUST include error handling, logging, and validation
- Follow language-specific style guides (PEP 8 for Python, ESLint for JavaScript/TypeScript)
- No hardcoded secrets; use environment variables and `.env` files
- Document dependencies and setup instructions clearly

### VI. Deployment Readiness

The entire system MUST be deployable: textbook to GitHub Pages, backend services to Vercel or Railway. Deployment configurations, environment variables, and build processes MUST be documented and functional.

**Rationale**: Hackathon judges will evaluate the live system. Non-deployable projects fail evaluation.

**Enforcement**:
- GitHub Pages deployment configured for Docusaurus site
- Backend deployment configuration for Vercel/Railway documented
- Environment variable templates provided (`.env.example`)
- Build and deployment scripts tested and documented
- CI/CD pipeline optional but deployment MUST be manual-executable at minimum

## Technology Stack Standards

**Textbook Frontend**:
- Docusaurus (latest stable version)
- Spec-Kit Plus for development workflow
- MDX for chapter content
- React components for interactive elements

**Backend Services**:
- FastAPI (Python) for API layer
- OpenAI Agents/ChatKit for RAG chatbot intelligence
- Qdrant for vector storage (embeddings)
- Neon Postgres for metadata and user data
- Better-Auth for authentication
- Translation service (Google Translate API or equivalent) for Urdu support

**Development Tools**:
- Git for version control
- Spec-Kit Plus commands (`/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`)
- Claude Subagents for content generation and assistance

**Deployment Targets**:
- GitHub Pages for textbook static site
- Vercel or Railway for backend API services

## Development Workflow

### Feature Development Process

1. **Specification** (`/sp.specify`): Define feature requirements with user stories, acceptance criteria, and success metrics
2. **Planning** (`/sp.plan`): Create architectural plan with technical decisions, dependencies, and structure
3. **Task Generation** (`/sp.tasks`): Break down plan into testable, executable tasks organized by user story
4. **Implementation** (`/sp.implement`): Execute tasks with verification at each checkpoint
5. **ADR Documentation**: For architecturally significant decisions, create ADRs using `/sp.adr`
6. **PHR Recording**: Capture Prompt History Records for all major development interactions

### Quality Gates

**Constitution Check**: Every plan MUST verify compliance with the 6 core principles before implementation.

**Content Quality**:
- Technical accuracy verified against official documentation
- Code examples tested and runnable
- Diagrams clear and correctly labeled
- Writing clarity appropriate for target audience (robotics learners)

**Backend Quality**:
- API endpoints documented (OpenAPI/Swagger)
- Error handling comprehensive
- Security best practices followed (auth, input validation, secrets management)
- Performance acceptable for expected load

**Integration Testing**:
- RAG chatbot retrieval accuracy validated
- "Answer from selected text" feature functional
- Authentication flow complete
- Personalization logic verified
- Translation accuracy spot-checked

### Git & Commit Practices

- **Branch Naming**: `[###-feature-name]` for feature branches
- **Commits**: Small, testable changes with clear messages
- **PRs**: Include summary, testing notes, and related spec/plan/tasks references
- **No Force Push**: to `main`/`master` branch

## Governance

### Constitution Authority

This constitution supersedes all other development practices and preferences. When conflicts arise between convenience and constitutional principles, principles win.

### Amendment Process

1. **Proposal**: Document proposed change with rationale and impact analysis
2. **Version Bump**: Determine MAJOR (breaking principle change), MINOR (new principle/section), or PATCH (clarification/typo fix)
3. **Sync Impact Review**: Identify affected templates, specs, and documentation
4. **Update & Propagate**: Update constitution and all dependent artifacts
5. **Record**: Embed sync impact report in constitution file

### Compliance Review

- All PRs MUST verify constitutional compliance
- Complexity violations MUST be explicitly justified in plan.md
- Regular audits of specs, plans, and code against constitutional principles
- Use `/sp.constitution` command to update this file with new principles or amendments

### Runtime Guidance

For agent-specific execution guidance, refer to `CLAUDE.md`. Constitution defines WHAT principles govern the project; runtime guidance defines HOW agents execute within those principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
