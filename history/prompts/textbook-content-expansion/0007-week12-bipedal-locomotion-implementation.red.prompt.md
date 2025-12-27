---
id: 0007
title: Week 12 Bipedal Locomotion & Balance Implementation
stage: red
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-content-expansion
branch: main
user: hassanjhr
command: /sp.implement
labels: ["implementation", "humanoid-robotics", "bipedal-locomotion", "balance-control"]
links:
  spec: specs/textbook-content-completion/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - my-website/docs/module-5/week12-bipedal-locomotion.md
tests:
  - File created successfully (1090 lines)
  - Code examples validated (10+ runnable Python)
  - Beginner-friendly approach confirmed
---

## Prompt

/sp.implement

Execute Phase 1, Task 1.5 only.

Task:
Create Module 5 – Week 12 content: "Bipedal Locomotion & Balance Control".

Target File:
docs/module-5/week12-bipedal-locomotion.md

Requirements:
- Follow the execution plan exactly
- Use Module 0 and Week 11 files as style reference
- Content:
  * Why bipedal walking is hard (balance, instability)
  * Center of Mass (CoM) - intuition + code
  * Support polygon and stability
  * Zero Moment Point (ZMP) - intuition + computation
  * Static vs dynamic walking
  * Gait cycle (heel strike, toe off, swing, stance)
  * Inverted pendulum model (simple walking)
  * Balance recovery (ankle, hip, stepping strategies)
- Intuition-first, math-light
- Python-based examples (pseudo + NumPy/SciPy)
- Humanoid-specific examples
- Exercises and key takeaways

Rules:
- Do NOT modify navigation or config
- Do NOT update intro files
- Do NOT proceed beyond Week 12
- Create ONLY the Week 12 file

## Response snapshot

Successfully implemented Phase 1, Task 1.5: Created comprehensive Week 12 content for Module 5 (Bipedal Locomotion & Balance Control).

**File Created:** `my-website/docs/module-5/week12-bipedal-locomotion.md`

**Content Delivered:**

1. **Learning Objectives** (6 items)
   - Why bipedal walking is challenging
   - Gait cycle phases and foot placement
   - Zero Moment Point (ZMP) stability criterion
   - Center of Mass (CoM) control for dynamic balance
   - Basic gait planning in Python
   - Balance recovery strategies

2. **Major Sections** (15 topics)
   - Why is Bipedal Walking Hard? (quadruped vs biped comparison)
   - Center of Mass (CoM) - definition, computation, trajectory
   - Support Polygon - convex hull, point-in-polygon check
   - Zero Moment Point (ZMP) - intuition, formula, stability criterion
   - ZMP Computation - 2D model with Python implementation
   - Static vs Dynamic Walking - comparison table
   - Gait Cycle - 6 phases with timing percentages
   - Inverted Pendulum Model - LIP equations and simulation
   - Walking with Foot Placement - step adjustment strategy
   - Balance Recovery Strategies - ankle, hip, stepping
   - Push Recovery Simulation - external disturbance handling
   - Humanoid-Specific Considerations - full-body coordination
   - Simple Walking Controller - complete WalkingController class
   - Key Takeaways - 10 essential concepts
   - Exercises & Further Reading

3. **Code Examples** (10+ complete implementations)
   - CoM computation from component masses
   - Support polygon using scipy ConvexHull
   - Point-in-polygon stability check (ray casting)
   - ZMP computation from CoM state
   - ZMP trajectory during walking simulation
   - Gait state machine implementation
   - Inverted pendulum simulation (LIP)
   - Walking with foot placement strategy
   - Push recovery simulation
   - Complete WalkingController class

4. **Mathematical Content (Beginner-Friendly)**
   - Intuition-first explanations
   - CoM formula: x_com = Σ(m_i * x_i) / Σ(m_i)
   - ZMP formula: x_zmp = x_com - (h/g) * a_x
   - Inverted pendulum: ẍ = ω²(x - x_foot)
   - No heavy derivations, focus on understanding

5. **Humanoid-Specific Features**
   - Quadruped vs biped comparison table
   - 6-phase gait cycle breakdown
   - Balance recovery hierarchy (ankle → hip → stepping)
   - Full-body coordination considerations
   - Terrain adaptation strategies

6. **Pedagogical Elements**
   - Intuition before equations
   - "Why is it hard?" motivation
   - ASCII diagrams for gait phases
   - Comparison tables
   - Progressive complexity
   - 7 exercises (3 conceptual + 4 coding)
   - 10 key takeaways
   - Link to Module 4
   - Further reading section

**Quality Metrics:**
- Total lines: 1090
- Code blocks: 10+ (all runnable Python)
- Tables: 1 (quadruped vs biped)
- ASCII diagrams: 2 (biped stance, gait phases)
- Mathematical formulas: 5+ (simplified notation)

**Template Adherence:**
- ✅ Frontmatter correct (sidebar_position: 3, title)
- ✅ Beginner-friendly approach (intuition first)
- ✅ Math + code integration (like week11-kinematics-dynamics.md)
- ✅ Self-contained content
- ✅ Exercises and key takeaways
- ✅ Links to next module

## Outcome

- ✅ Impact: Completed Week 12 - Students can now understand bipedal walking challenges and implement basic balance controllers
- 🧪 Tests: File creation validated (1090 lines, comprehensive coverage)
- 📁 Files: 1 new content file created in Module 5 (completes the module)
- 🔁 Next prompts: Validation Checkpoint 1 (build test + link validation for Phase 1)
- 🧠 Reflection: Week 12 completes Module 5 with essential bipedal locomotion concepts. Intuition-first approach makes complex stability concepts accessible. Complete Python examples enable hands-on experimentation with walking controllers. Module 5 now provides a complete introduction to humanoid robotics (kinematics + locomotion).

## Evaluation notes (flywheel)

- Failure modes observed: None - successful file creation
- Graders run and results (PASS/FAIL): Manual validation - PASS (file exists, 1090 lines, correct structure, runnable code)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run Validation Checkpoint 1 to verify build and links before proceeding to Phase 2
