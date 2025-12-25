---
id: 3
title: Clarify Physical AI Course Spec
stage: spec
date: 2025-01-14
surface: agent
model: Qwen
feature: physical-ai-course
branch: 1-physical-ai-course
user: user
command: /sp.clarify
labels: [clarify,spec,physical-ai,robotics,education,course]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-course/spec.md
tests:
 - none
---

## Prompt

Identify underspecified areas in the current feature spec by asking up to 5 highly targeted clarification questions and encoding answers back into the spec.
handoffs: 
  - label: Build Technical Plan
    agent: sp.plan
    prompt: Create a plan for the spec. I am building with...

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

Goal: Detect and reduce ambiguity or missing decision points in the active feature specification and record the clarifications directly in the spec file.

Note: This clarification workflow is expected to run (and be completed) BEFORE invoking `/sp.plan`. If the user explicitly states they are skipping clarification (e.g., exploratory spike), you may proceed, but must warn that downstream rework risk increases.

Execution steps:

1. Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json -PathsOnly` from repo root **once** (combined `--json --paths-only` mode / `-Json -PathsOnly`). Parse minimal JSON payload fields:
   - `FEATURE_DIR`
   - `FEATURE_SPEC`
   - (Optionally capture `IMPL_PLAN`, `TASKS` for future chained flows.)
   - If JSON parsing fails, abort and instruct user to re-run `/sp.specify` or verify feature branch environment.
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. Load the current spec file. Perform a structured ambiguity & coverage scan using this taxonomy. For each category, mark status: Clear / Partial / Missing. Produce an internal coverage map used for prioritization (do not output raw map unless no questions will be asked).

   Functional Scope & Behavior:
   - Core user goals & success criteria
   - Explicit out-of-scope declarations
   - User roles / personas differentiation

   Domain & Data Model:
   - Entities, attributes, relationships
   - Identity & uniqueness rules
   - Lifecycle/state transitions
   - Data volume / scale assumptions

   Interaction & UX Flow:
   - Critical user journeys / sequences
   - Error/empty/loading states
   - Accessibility or localization notes

   Non-Functional Quality Attributes:
   - Performance (latency, throughput targets)
   - Scalability (horizontal/vertical, limits)
   - Reliability & availability (uptime, recovery expectations)
   - Observability (logging, metrics, tracing signals)
   - Security & privacy (authN/Z, data protection, threat assumptions)
   - Compliance / regulatory constraints (if any)

   Integration & External Dependencies:
   - External services/APIs and failure modes
   - Data import/export formats
   - Protocol/versioning assumptions

   Edge Cases & Failure Handling:
   - Negative scenarios
   - Rate limiting / throttling
   - Conflict resolution (e.g., concurrent edits)

   Constraints & Tradeoffs:
   - Technical constraints (language, storage, hosting)
   - Explicit tradeoffs or rejected alternatives

   Terminology & Consistency:
   - Canonical glossary terms
   - Avoided synonyms / deprecated terms

   Completion Signals:
   - Acceptance criteria testability
   - Measurable Definition of Done style indicators

   Misc / Placeholders:
   - TODO markers / unresolved decisions
   - Ambiguous adjectives ("robust", "intuitive") lacking quantification

   For each category with Partial or Missing status, add a candidate question opportunity unless:
   - Clarification would not materially change implementation or validation strategy
   - Information is better deferred to planning phase (note internally)

3. Generate (internally) a prioritized queue of candidate clarification questions (maximum 5). Do NOT output them all at once. Apply these constraints:
    - Maximum of 10 total questions across the whole session.
    - Each question must be answerable with EITHER:
       - A short multiple‑choice selection (2–5 distinct, mutually exclusive options), OR
       - A one-word / short‑phrase answer (explicitly constrain: "Answer in <=5 words").
    - Only include questions whose answers materially impact architecture, data modeling, task decomposition, test design, UX behavior, operational readiness, or compliance validation.
    - Ensure category coverage balance: attempt to cover the highest impact unresolved categories first; avoid asking two low-impact questions when a single high-impact area (e.g., security posture) is unresolved.
    - Exclude questions already answered, trivial stylistic preferences, or plan-level execution details (unless blocking correctness).
    - Favor clarifications that reduce downstream rework risk or prevent misaligned acceptance tests.
    - If more than 5 categories remain unresolved, select the top 5 by (Impact * Uncertainty) heuristic.

4. Sequential questioning loop (interactive):
    - Present EXACTLY ONE question at a time.
    - For multiple‑choice questions:
       - **Analyze all options** and determine the **most suitable option** based on:
          - Best practices for the project type
          - Common patterns in similar implementations
          - Risk reduction (security, performance, maintainability)
          - Alignment with any explicit project goals or constraints visible in the spec
       - Present your **recommended option prominently** at the top with clear reasoning (1-2 sentences explaining why this is the best choice).
       - Format as: `**Recommended:** Option [X] - <reasoning>`
       - Then render all options as a Markdown table:

       | Option | Description |
       |--------|-------------|
       | A | <Option A description> |
       | B | <Option B description> |
       | C | <Option C description> (add D/E as needed up to 5) |
       | Short | Provide a different short answer (<=5 words) (Include only if free-form alternative is appropriate) |

       - After the table, add: `You can reply with the option letter (e.g., "A"), accept the recommendation by saying "yes" or "recommended", or provide your own short answer.`
    - For short‑answer style (no meaningful discrete options):
       - Provide your **suggested answer** based on best practices and context.
       - Format as: `**Suggested:** <your proposed answer> - <brief reasoning>`
       - Then output: `Format: Short answer (<=5 words). You can accept the suggestion by saying "yes" or "suggested", or provide your own answer.`
    - After the user answers:
       - If the user replies with "yes", "recommended", or "suggested", use your previously stated recommendation/suggestion as the answer.
       - Otherwise, validate the answer maps to one option or fits the <=5 word constraint.
       - If ambiguous, ask for a quick disambiguation (count still belongs to same question; do not advance).
       - Once satisfactory, record it in working memory (do not yet write to disk) and move to the next queued question.
    - Stop asking further questions when:
       - All critical ambiguities resolved early (remaining queued items become unnecessary), OR
       - User signals completion ("done", "good", "no more"), OR
       - You reach 5 asked questions.
    - Never reveal future queued questions in advance.
    - If no valid questions exist at start, immediately report no critical ambiguities.

5. Integration after EACH accepted answer (incremental update approach):
    - Maintain in-memory representation of the spec (loaded once at start) plus the raw file contents.
    - For the first integrated answer in this session:
       - Ensure a `## Clarifications` section exists (create it just after the highest-level contextual/overview section per the spec template if missing).
       - Under it, create (if not present) a `### Session YYYY-MM-DD` subheading for today.
    - Append a bullet line immediately after acceptance: `- Q: <question> → A: <final answer>`.
    - Then immediately apply the clarification to the most appropriate section(s):
       - Functional ambiguity → Update or add a bullet in Functional Requirements.
       - User interaction / actor distinction → Update User Stories or Actors subsection (if present) with clarified role, constraint, or scenario.
       - Data shape / entities → Update Data Model (add fields, types, relationships) preserving ordering; note added constraints succinctly.
       - Non-functional constraint → Add/modify measurable criteria in Non-Functional / Quality Attributes section (convert vague adjective to metric or explicit target).
       - Edge case / negative flow → Add a new bullet under Edge Cases / Error Handling (or create such subsection if template provides placeholder for it).
       - Terminology conflict → Normalize term across spec; retain original only if necessary by adding `(formerly referred to as "X")` once.
    - If the clarification invalidates an earlier ambiguous statement, replace that statement instead of duplicating; leave no obsolete contradictory text.
    - Save the spec file AFTER each integration to minimize risk of context loss (atomic overwrite).
    - Preserve formatting: do not reorder unrelated sections; keep heading hierarchy intact.
    - Keep each inserted clarification minimal and testable (avoid narrative drift).

6. Validation (performed after EACH write plus final pass):
   - Clarifications session contains exactly one bullet per accepted answer (no duplicates).
   - Total asked (accepted) questions ≤ 5.
   - Updated sections contain no lingering vague placeholders the new answer was meant to resolve.
   - No contradictory earlier statement remains (scan for now-invalid alternative choices removed).
   - Markdown structure valid; only allowed new headings: `## Clarifications`, `### Session YYYY-MM-DD`.
   - Terminology consistency: same canonical term used across all updated sections.

7. Write the updated spec back to `FEATURE_SPEC`.

8. Report completion (after questioning loop ends or early termination):
   - Number of questions asked & answered.
   - Path to updated spec.
   - Sections touched (list names).
   - Coverage summary table listing each taxonomy category with Status: Resolved (was Partial/Missing and addressed), Deferred (exceeds question quota or better suited for planning), Clear (already sufficient), Outstanding (still Partial/Missing but low impact).
   - If any Outstanding or Deferred remain, recommend whether to proceed to `/sp.plan` or run `/sp.clarify` again later post-plan.
   - Suggested next command.

Behavior rules:

- If no meaningful ambiguities found (or all potential questions would be low-impact), respond: "No critical ambiguities detected worth formal clarification." and suggest proceeding.
- If spec file missing, instruct user to run `/sp.specify` first (do not create a new spec here).
- Never exceed 5 total asked questions (clarification retries for a single question do not count as new questions).
- Avoid speculative tech stack questions unless the absence blocks functional clarity.
- Respect user early termination signals ("stop", "done", "proceed").
- If no questions asked due to full coverage, output a compact coverage summary (all categories Clear) then suggest advancing.
- If quota reached with unresolved high-impact categories remaining, explicitly flag them under Deferred with rationale.

Context for prioritization: Ensure: 52 total weekly chapters (4×13), nested file structure (docs/week-01/1-1-foundations.md etc.), consistent coverage of all original topics, practical ROS 2/Python examples.

## Response snapshot

## Clarifications

### Session 2025-01-14

- Q: What is the hosting approach for the course materials? → A: Hosted on GitHub Pages with Docusaurus
- Q: What types of assessments will be used? → A: Multiple choice quizzes + practical coding assignments
- Q: What is the target audience's prerequisite knowledge? → A: Students with basic programming knowledge and introductory robotics concepts
- Q: What is the balance between theoretical and practical content? → A: Balance between theoretical concepts and practical implementation
- Q: What are the course completion requirements? → A: Complete all 52 chapters + pass assessments + complete capstone project

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Physical AI Fundamentals (Priority: P1)

A student with basic programming knowledge and introductory robotics concepts wants to understand the principles of Physical AI and embodied intelligence. They start with the introductory sections covering the theme, goal, overview, and why Physical AI matters. The student should be able to understand the learning outcomes and navigate through the 13-week curriculum with 4 chapters per week.

**Why this priority**: This is the foundational user journey that enables all other learning paths. Without understanding the core concepts, students cannot progress to more advanced topics.

**Independent Test**: The student can complete the introductory sections and demonstrate understanding of the fundamental concepts of Physical AI and embodied intelligence through assessments.

**Acceptance Scenarios**:

1. **Given** a student with basic programming knowledge and introductory robotics concepts, **When** they access the course materials, **Then** they can navigate through the introduction sections and understand the core concepts of Physical AI
2. **Given** a student reviewing the course, **When** they read the learning outcomes section, **Then** they can articulate what they will be able to do after completing the course

---

### User Story 2 - Student Completes Weekly Modules (Priority: P1)

A student follows the 13-week curriculum, completing 4 chapters per week. Each week focuses on a specific aspect of Physical AI and robotics, with practical examples, code snippets, and diagrams to enhance understanding.

**Why this priority**: This represents the core learning journey of the course and is essential to achieving the overall learning outcomes.

**Independent Test**: The student can complete a single week's content (4 chapters) and demonstrate understanding through practical exercises or assessments.

**Acceptance Scenarios**:

1. **Given** a student with basic programming knowledge and introductory robotics concepts at the start of Week 1, **When** they complete all 4 chapters of Week 1, **Then** they understand the foundations of Physical AI and embodied intelligence principles
2. **Given** a student at the start of Week 3, **When** they complete the ROS 2 architecture chapters, **Then** they can implement basic ROS 2 nodes and understand the communication patterns

---

### User Story 3 - Student Uses Hardware Components (Priority: P2)

A student accesses the hardware requirements section and learns about the available platforms like Jetson Orin Nano Super, Unitree Go2, Unitree G1, and RealSense D435i. They understand the costs and capabilities of each component and how they fit into the Physical AI ecosystem.

**Why this priority**: Understanding hardware is crucial for implementing Physical AI concepts in real systems, though it may not be accessible to all students.

**Independent Test**: The student can identify the appropriate hardware for specific Physical AI tasks and understand the trade-offs between different platforms.

**Acceptance Scenarios**:

1. **Given** a student reviewing hardware options, **When** they examine the hardware requirements section, **Then** they can identify which components are needed for specific robotics projects
2. **Given** a student with budget constraints, **When** they review the cost tables, **Then** they can select appropriate hardware options within their budget

---

### User Story 4 - Educator Implements Course Content (Priority: P2)

An educator wants to use the course materials to teach Physical AI and humanoid robotics. They need access to the complete curriculum structure, assessment options, and lab setup guidance for both on-premise and cloud environments.

**Why this priority**: This enables broader adoption of the course materials and ensures educators can effectively deliver the content.

**Independent Test**: The educator can set up a learning environment following the provided guidelines and successfully deliver course content to students.

**Acceptance Scenarios**:

1. **Given** an educator planning to use the course, **When** they review the curriculum structure, **Then** they can organize the content into a semester-long course
2. **Given** an educator with limited lab resources, **When** they review the lab options, **Then** they can implement the course using either on-premise or cloud-based resources

---

### User Story 5 - Student Completes Capstone Project (Priority: P1)

A student reaches Week 13 and undertakes the conversational robotics capstone project, integrating GPT, speech, and multi-modal capabilities into a humanoid robot system. This project demonstrates mastery of all previous concepts.

**Why this priority**: The capstone project represents the culmination of all learning and demonstrates the student's ability to synthesize concepts across the entire curriculum.

**Independent Test**: The student can successfully implement a complex Physical AI system that incorporates multiple technologies and concepts from throughout the course.

**Acceptance Scenarios**:

1. **Given** a student at the beginning of Week 13, **When** they complete the conversational robotics chapters, **Then** they can implement a system that integrates GPT, speech recognition, and multi-modal inputs
2. **Given** a student with completed capstone project, **When** they demonstrate their system, **Then** it successfully integrates multiple Physical AI concepts and operates as expected

---

### Edge Cases

- What happens when a student has limited access to expensive hardware components?
- How does the system handle students with different technical backgrounds and skill levels?
- What if certain software platforms (like NVIDIA Isaac) are not available in the student's region?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a complete 13-week curriculum with exactly 4 chapters per week as specified
- **FR-002**: System MUST include detailed content for Intro sections: Theme, Goal, Overview, Why Physical AI Matters, and Learning Outcomes
- **FR-003**: System MUST cover Modules 1-4 as top-level sections with appropriate depth and detail
- **FR-004**: System MUST include detailed weekly breakdowns with specific topics for each of the 52 chapters (13 weeks × 4 chapters)
- **FR-005**: System MUST provide comprehensive hardware requirements with updated prices for Jetson Orin Nano Super, Unitree Go2, Unitree G1, and RealSense D435i
- **FR-006**: System MUST include assessment options for each module and week to evaluate student learning
- **FR-007**: System MUST provide lab setup options for both on-premise and cloud environments
- **FR-008**: System MUST use Docusaurus classic template with docs as root and nested hierarchical sidebar (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices)
- **FR-009**: System MUST include rich MDX content with code blocks, tables for hardware comparisons, and Mermaid diagrams for ROS 2 architecture and URDF
- **FR-010**: System MUST provide educational, step-by-step content with appropriate warnings on costs and complexity
- **FR-011**: System MUST support automatic deployment to GitHub Pages via Actions
- **FR-012**: System MUST include content for NVIDIA Isaac (Isaac Sim, perception, RL, sim-to-real) as specified
- **FR-013**: System MUST include content for Gazebo/Unity integration with Physical AI systems
- **FR-014**: System MUST include content on humanoid kinematics, locomotion, manipulation, and interaction
- **FR-015**: System MUST include content on conversational robotics with GPT integration, speech, and multi-modal capabilities
- **FR-016**: System MUST include multiple choice quizzes and practical coding assignments for assessment
- **FR-017**: System MUST be designed for students with basic programming knowledge and introductory robotics concepts
- **FR-018**: System MUST balance theoretical concepts with practical implementation
- **FR-019**: System MUST require completion of all 52 chapters, passing assessments, and completion of the capstone project for course completion

## Outcome

- ✅ Impact: Clarified 5 critical areas in the Physical AI & Humanoid Robotics course specification
- 🧪 Tests: none
- 📁 Files: specs/1-physical-ai-course/spec.md
- 🔁 Next prompts: Build technical plan for the spec
- 🧠 Reflection: Successfully clarified the hosting approach, assessment types, target audience prerequisites, content balance, and completion requirements. All clarifications have been integrated into the specification.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): Create the technical plan for this specification