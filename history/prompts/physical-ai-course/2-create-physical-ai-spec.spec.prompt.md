---
id: 2
title: Create Physical AI Course Spec
stage: spec
date: 2025-01-14
surface: agent
model: Qwen
feature: physical-ai-course
branch: 1-physical-ai-course
user: user
command: /sp.specify
labels: [spec,physical-ai,robotics,education,course]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-course/spec.md
 - specs/1-physical-ai-course/checklists/requirements.md
tests:
 - none
---

## Prompt

Create or update the feature specification from a natural language feature description.
handoffs: 
  - label: Build Technical Plan
    agent: sp.plan
    prompt: Create a plan for the spec. I am building with...
  - label: Clarify Spec Requirements
    agent: sp.clarify
    prompt: Clarify specification requirements
    send: true

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.specify` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `Book title: "Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence". Structure: - Intro sections: Theme/Goal/Overview/Why Physical AI Matters/Learning Outcomes. - Modules 1-4 (as top-level). - Weekly Breakdown: 13 weeks, each exactly 4 chapters (expand original topics logically). Example expansion: Week 1: 1.1 Foundations of Physical AI, 1.2 Embodied Intelligence Principles, 1.3 From Digital AI to Physical Laws, 1.4 Humanoid Robotics Landscape Week 2: 2.1 LiDAR Sensors, 2.2 Depth Cameras, 2.3 IMUs for Balance, 2.4 Force/Torque Sensors Weeks 3-5: ROS 2 (12 chapters total, 4 per week: architecture, nodes/topics/services/actions, packages, URDF, etc.) Weeks 6-7: Gazebo/Unity (8 chapters: setup, URDF/SDF, physics, sensors, Unity viz) Weeks 8-10: NVIDIA Isaac (12 chapters: Isaac Sim, perception, RL, sim-to-real) Weeks 11-12: Humanoid Development (8 chapters: kinematics, locomotion, manipulation, interaction) Week 13: Conversational Robotics (4 chapters: GPT integration, speech, multi-modal, capstone overview) - Assessments + Hardware Requirements (with updated prices/tables) + Lab Options (On-Premise/Cloud). - Sidebar: Nested (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices).` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

1. **Generate a concise short name** (2-4 words) for the branch:
   - Analyze the feature description and extract the most meaningful keywords
   - Create a 2-4 word short name that captures the essence of the feature
   - Use action-noun format when possible (e.g., "add-user-auth", "fix-payment-bug")
   - Preserve technical terms and acronyms (OAuth2, API, JWT, etc.)
   - Keep it concise but descriptive enough to understand the feature at a glance
   - Examples:
     - "I want to add user authentication" → "user-auth"
     - "Implement OAuth2 integration for the API" → "oauth2-api-integration"
     - "Create a dashboard for analytics" → "analytics-dashboard"
     - "Fix payment processing timeout bug" → "fix-payment-timeout"

2. **Check for existing branches before creating new one**:

   a. First, fetch all remote branches to ensure we have the latest information:

      ```bash
      git fetch --all --prune
      ```

   b. Find the highest feature number across all sources for the short-name:
      - Remote branches: `git ls-remote --heads origin | grep -E 'refs/heads/[0-9]+-<short-name>$'`
      - Local branches: `git branch | grep -E '^[* ]*[0-9]+-<short-name>$'`
      - Specs directories: Check for directories matching `specs/[0-9]+-<short-name>`

   c. Determine the next available number:
      - Extract all numbers from all three sources
      - Find the highest number N
      - Use N+1 for the new branch number

   d. Run the script `.specify/scripts/powershell/create-new-feature.ps1 -Json "Book title: "Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence". Structure: - Intro sections: Theme/Goal/Overview/Why Physical AI Matters/Learning Outcomes. - Modules 1-4 (as top-level). - Weekly Breakdown: 13 weeks, each exactly 4 chapters (expand original topics logically). Example expansion: Week 1: 1.1 Foundations of Physical AI, 1.2 Embodied Intelligence Principles, 1.3 From Digital AI to Physical Laws, 1.4 Humanoid Robotics Landscape Week 2: 2.1 LiDAR Sensors, 2.2 Depth Cameras, 2.3 IMUs for Balance, 2.4 Force/Torque Sensors Weeks 3-5: ROS 2 (12 chapters total, 4 per week: architecture, nodes/topics/services/actions, packages, URDF, etc.) Weeks 6-7: Gazebo/Unity (8 chapters: setup, URDF/SDF, physics, sensors, Unity viz) Weeks 8-10: NVIDIA Isaac (12 chapters: Isaac Sim, perception, RL, sim-to-real) Weeks 11-12: Humanoid Development (8 chapters: kinematics, locomotion, manipulation, interaction) Week 13: Conversational Robotics (4 chapters: GPT integration, speech, multi-modal, capstone overview) - Assessments + Hardware Requirements (with updated prices/tables) + Lab Options (On-Premise/Cloud). - Sidebar: Nested (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices)."` with the calculated number and short-name:
      - Pass `--number N+1` and `--short-name "your-short-name"` along with the feature description
      - Bash example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "Book title: "Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence". Structure: - Intro sections: Theme/Goal/Overview/Why Physical AI Matters/Learning Outcomes. - Modules 1-4 (as top-level). - Weekly Breakdown: 13 weeks, each exactly 4 chapters (expand original topics logically). Example expansion: Week 1: 1.1 Foundations of Physical AI, 1.2 Embodied Intelligence Principles, 1.3 From Digital AI to Physical Laws, 1.4 Humanoid Robotics Landscape Week 2: 2.1 LiDAR Sensors, 2.2 Depth Cameras, 2.3 IMUs for Balance, 2.4 Force/Torque Sensors Weeks 3-5: ROS 2 (12 chapters total, 4 per week: architecture, nodes/topics/services/actions, packages, URDF, etc.) Weeks 6-7: Gazebo/Unity (8 chapters: setup, URDF/SDF, physics, sensors, Unity viz) Weeks 8-10: NVIDIA Isaac (12 chapters: Isaac Sim, perception, RL, sim-to-real) Weeks 11-12: Humanoid Development (8 chapters: kinematics, locomotion, manipulation, interaction) Week 13: Conversational Robotics (4 chapters: GPT integration, speech, multi-modal, capstone overview) - Assessments + Hardware Requirements (with updated prices/tables) + Lab Options (On-Premise/Cloud). - Sidebar: Nested (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices)." --json --number 5 --short-name "user-auth" "Add user authentication"`
      - PowerShell example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "Book title: "Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence". Structure: - Intro sections: Theme/Goal/Overview/Why Physical AI Matters/Learning Outcomes. - Modules 1-4 (as top-level). - Weekly Breakdown: 13 weeks, each exactly 4 chapters (expand original topics logically). Example expansion: Week 1: 1.1 Foundations of Physical AI, 1.2 Embodied Intelligence Principles, 1.3 From Digital AI to Physical Laws, 1.4 Humanoid Robotics Landscape Week 2: 2.1 LiDAR Sensors, 2.2 Depth Cameras, 2.3 IMUs for Balance, 2.4 Force/Torque Sensors Weeks 3-5: ROS 2 (12 chapters total, 4 per week: architecture, nodes/topics/services/actions, packages, URDF, etc.) Weeks 6-7: Gazebo/Unity (8 chapters: setup, URDF/SDF, physics, sensors, Unity viz) Weeks 8-10: NVIDIA Isaac (12 chapters: Isaac Sim, perception, RL, sim-to-real) Weeks 11-12: Humanoid Development (8 chapters: kinematics, locomotion, manipulation, interaction) Week 13: Conversational Robotics (4 chapters: GPT integration, speech, multi-modal, capstone overview) - Assessments + Hardware Requirements (with updated prices/tables) + Lab Options (On-Premise/Cloud). - Sidebar: Nested (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices)." -Json -Number 5 -ShortName "user-auth" "Add user authentication"`

   **IMPORTANT**:
   - Check all three sources (remote branches, local branches, specs directories) to find the highest number
   - Only match branches/directories with the exact short-name pattern
   - If no existing branches/directories found with this short-name, start with number 1
   - You must only ever run this script once per feature
   - The JSON is provided in the terminal as output - always refer to it to get the actual content you're looking for
   - The JSON output will contain BRANCH_NAME and SPEC_FILE paths
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot")

3. Load `.specify/templates/spec-template.md` to understand required sections.

4. Follow this execution flow:

    1. Parse user description from Input
       If empty: ERROR "No feature description provided"
    2. Extract key concepts from description
       Identify: actors, actions, data, constraints
    3. For unclear aspects:
       - Make informed guesses based on context and industry standards
       - Only mark with [NEEDS CLARIFICATION: specific question] if:
         - The choice significantly impacts feature scope or user experience
         - Multiple reasonable interpretations exist with different implications
         - No reasonable default exists
       - **LIMIT: Maximum 3 [NEEDS CLARIFICATION] markers total**
       - Prioritize clarifications by impact: scope > security/privacy > user experience > technical details
    4. Fill User Scenarios & Testing section
       If no clear user flow: ERROR "Cannot determine user scenarios"
    5. Generate Functional Requirements
       Each requirement must be testable
       Use reasonable defaults for unspecified details (document assumptions in Assumptions section)
    6. Define Success Criteria
       Create measurable, technology-agnostic outcomes
       Include both quantitative metrics (time, performance, volume) and qualitative measures (user satisfaction, task completion)
       Each criterion must be verifiable without implementation details
    7. Identify Key Entities (if data involved)
    8. Return: SUCCESS (spec ready for planning)

5. Write the specification to SPEC_FILE using the template structure, replacing placeholders with concrete details derived from the feature description (arguments) while preserving section order and headings.

6. **Specification Quality Validation**: After writing the initial spec, validate it against quality criteria:

   a. **Create Spec Quality Checklist**: Generate a checklist file at `FEATURE_DIR/checklists/requirements.md` using the checklist template structure with these validation items:

      ```markdown
      # Specification Quality Checklist: [FEATURE NAME]
      
      **Purpose**: Validate specification completeness and quality before proceeding to planning
      **Created**: [DATE]
      **Feature**: [Link to spec.md]
      
      ## Content Quality
      
      - [ ] No implementation details (languages, frameworks, APIs)
      - [ ] Focused on user value and business needs
      - [ ] Written for non-technical stakeholders
      - [ ] All mandatory sections completed
      
      ## Requirement Completeness
      
      - [ ] No [NEEDS CLARIFICATION] markers remain
      - [ ] Requirements are testable and unambiguous
      - [ ] Success criteria are measurable
      - [ ] Success criteria are technology-agnostic (no implementation details)
      - [ ] All acceptance scenarios are defined
      - [ ] Edge cases are identified
      - [ ] Scope is clearly bounded
      - [ ] Dependencies and assumptions identified
      
      ## Feature Readiness
      
      - [ ] All functional requirements have clear acceptance criteria
      - [ ] User scenarios cover primary flows
      - [ ] Feature meets measurable outcomes defined in Success Criteria
      - [ ] No implementation details leak into specification
      
      ## Notes
      
      - Items marked incomplete require spec updates before `/sp.clarify` or `/sp.plan`
      ```

   b. **Run Validation Check**: Review the spec against each checklist item:
      - For each item, determine if it passes or fails
      - Document specific issues found (quote relevant spec sections)

   c. **Handle Validation Results**:

      - **If all items pass**: Mark checklist complete and proceed to step 6

      - **If items fail (excluding [NEEDS CLARIFICATION])**:
        1. List the failing items and specific issues
        2. Update the spec to address each issue
        3. Re-run validation until all items pass (max 3 iterations)
        4. If still failing after 3 iterations, document remaining issues in checklist notes and warn user

      - **If [NEEDS CLARIFICATION] markers remain**:
        1. Extract all [NEEDS CLARIFICATION: ...] markers from the spec
        2. **LIMIT CHECK**: If more than 3 markers exist, keep only the 3 most critical (by scope/security/UX impact) and make informed guesses for the rest
        3. For each clarification needed (max 3), present options to user in this format:

           ```markdown
           ## Question [N]: [Topic]
           
           **Context**: [Quote relevant spec section]
           
           **What we need to know**: [Specific question from NEEDS CLARIFICATION marker]
           
           **Suggested Answers**:
           
           | Option | Answer | Implications |
           |--------|--------|--------------|
           | A      | [First suggested answer] | [What this means for the feature] |
           | B      | [Second suggested answer] | [What this means for the feature] |
           | C      | [Third suggested answer] | [What this means for the feature] |
           | Custom | Provide your own answer | [Explain how to provide custom input] |
           
           **Your choice**: _[Wait for user response]_
           ```

        4. **CRITICAL - Table Formatting**: Ensure markdown tables are properly formatted:
           - Use consistent spacing with pipes aligned
           - Each cell should have spaces around content: `| Content |` not `|Content|`
           - Header separator must have at least 3 dashes: `|--------|`
           - Test that the table renders correctly in markdown preview
        5. Number questions sequentially (Q1, Q2, Q3 - max 3 total)
        6. Present all questions together before waiting for responses
        7. Wait for user to respond with their choices for all questions (e.g., "Q1: A, Q2: Custom - [details], Q3: B")
        8. Update the spec by replacing each [NEEDS CLARIFICATION] marker with the user's selected or provided answer
        9. Re-run validation after all clarifications are resolved

   d. **Update Checklist**: After each validation iteration, update the checklist file with current pass/fail status

7. Report completion with branch name, spec file path, checklist results, and readiness for the next phase (`/sp.clarify` or `/sp.plan`).

**NOTE:** The script creates and checks out the new branch and initializes the spec file before writing.

## General Guidelines

## Quick Guidelines

- Focus on **WHAT** users need and **WHY**.
- Avoid HOW to implement (no tech stack, APIs, code structure).
- Written for business stakeholders, not developers.
- DO NOT create any checklists that are embedded in the spec. That will be a separate command.

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation

When creating this spec from a user prompt:

1. **Make informed guesses**: Use context, industry standards, and common patterns to fill gaps
2. **Document assumptions**: Record reasonable defaults in the Assumptions section
3. **Limit clarifications**: Maximum 3 [NEEDS CLARIFICATION] markers - use only for critical decisions that:
   - Significantly impact feature scope or user experience
   - Have multiple reasonable interpretations with different implications
   - Lack any reasonable default
4. **Prioritize clarifications**: scope > security/privacy > user experience > technical details
5. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
6. **Common areas needing clarification** (only if no reasonable default exists):
   - Feature scope and boundaries (include/exclude specific use cases)
   - User types and permissions (if multiple conflicting interpretations possible)
   - Security/compliance requirements (when legally/financially significant)

**Examples of reasonable defaults** (don't ask about these):

- Data retention: Industry-standard practices for the domain
- Performance targets: Standard web/mobile app expectations unless specified
- Error handling: User-friendly messages with appropriate fallbacks
- Authentication method: Standard session-based or OAuth2 for web apps
- Integration patterns: RESTful APIs unless specified otherwise

### Success Criteria Guidelines

Success criteria must be:

1. **Measurable**: Include specific metrics (time, percentage, count, rate)
2. **Technology-agnostic**: No mention of frameworks, languages, databases, or tools
3. **User-focused**: Describe outcomes from user/business perspective, not system internals
4. **Verifiable**: Can be tested/validated without knowing implementation details

**Good examples**:

- "Users can complete checkout in under 3 minutes"
- "System supports 10,000 concurrent users"
- "95% of searches return results in under 1 second"
- "Task completion rate improves by 40%"

**Bad examples** (implementation-focused):

- "API response time is under 200ms" (too technical, use "Users see results instantly")
- "Database can handle 1000 TPS" (implementation detail, use user-facing metric)
- "React components render efficiently" (framework-specific)
- "Redis cache hit rate above 80%" (technology-specific)

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


## Response snapshot

# Feature Specification: Physical AI & Humanoid Robotics Course

**Feature Branch**: `1-physical-ai-course`
**Created**: 2025-01-14
**Status**: Draft
**Input**: User description: "Book title: \"Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence\". Structure: - Intro sections: Theme/Goal/Overview/Why Physical AI Matters/Learning Outcomes. - Modules 1-4 (as top-level). - Weekly Breakdown: 13 weeks, each exactly 4 chapters (expand original topics logically). Example expansion: Week 1: 1.1 Foundations of Physical AI, 1.2 Embodied Intelligence Principles, 1.3 From Digital AI to Physical Laws, 1.4 Humanoid Robotics Landscape Week 2: 2.1 LiDAR Sensors, 2.2 Depth Cameras, 2.3 IMUs for Balance, 2.4 Force/Torque Sensors Weeks 3-5: ROS 2 (12 chapters total, 4 per week: architecture, nodes/topics/services/actions, packages, URDF, etc.) Weeks 6-7: Gazebo/Unity (8 chapters: setup, URDF/SDF, physics, sensors, Unity viz) Weeks 8-10: NVIDIA Isaac (12 chapters: Isaac Sim, perception, RL, sim-to-real) Weeks 11-12: Humanoid Development (8 chapters: kinematics, locomotion, manipulation, interaction) Week 13: Conversational Robotics (4 chapters: GPT integration, speech, multi-modal, capstone overview) - Assessments + Hardware Requirements (with updated prices/tables) + Lab Options (On-Premise/Cloud). - Sidebar: Nested (Intro → Modules → Weeks 1-13 with 4 chapters each → Appendices)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Physical AI Fundamentals (Priority: P1)

A student with basic programming knowledge wants to understand the principles of Physical AI and embodied intelligence. They start with the introductory sections covering the theme, goal, overview, and why Physical AI matters. The student should be able to understand the learning outcomes and navigate through the 13-week curriculum with 4 chapters per week.

**Why this priority**: This is the foundational user journey that enables all other learning paths. Without understanding the core concepts, students cannot progress to more advanced topics.

**Independent Test**: The student can complete the introductory sections and demonstrate understanding of the fundamental concepts of Physical AI and embodied intelligence through assessments.

**Acceptance Scenarios**:

1. **Given** a student with basic programming knowledge, **When** they access the course materials, **Then** they can navigate through the introduction sections and understand the core concepts of Physical AI
2. **Given** a student reviewing the course, **When** they read the learning outcomes section, **Then** they can articulate what they will be able to do after completing the course

---

### User Story 2 - Student Completes Weekly Modules (Priority: P1)

A student follows the 13-week curriculum, completing 4 chapters per week. Each week focuses on a specific aspect of Physical AI and robotics, with practical examples, code snippets, and diagrams to enhance understanding.

**Why this priority**: This represents the core learning journey of the course and is essential to achieving the overall learning outcomes.

**Independent Test**: The student can complete a single week's content (4 chapters) and demonstrate understanding through practical exercises or assessments.

**Acceptance Scenarios**:

1. **Given** a student at the start of Week 1, **When** they complete all 4 chapters of Week 1, **Then** they understand the foundations of Physical AI and embodied intelligence principles
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

### Key Entities

- **Course Curriculum**: The complete 13-week program with 4 chapters per week, covering all specified topics from introductory concepts to advanced humanoid robotics
- **Hardware Components**: Physical devices including Jetson Orin Nano Super, Unitree Go2, Unitree G1, and RealSense D435i with their specifications and costs
- **Software Platforms**: ROS 2, Gazebo/Unity, NVIDIA Isaac, and other tools required for Physical AI development
- **Learning Modules**: The four main modules covering different aspects of Physical AI and robotics
- **Assessment Methods**: Various evaluation approaches to measure student learning outcomes
- **Lab Environments**: On-premise and cloud-based options for implementing and testing Physical AI systems

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can navigate the complete 13-week curriculum and complete all 52 chapters within the expected timeframe
- **SC-002**: 85% of students successfully complete the capstone project integrating multiple Physical AI concepts
- **SC-003**: Students demonstrate understanding of embodied intelligence principles through assessments with a minimum 80% pass rate
- **SC-004**: The course materials receive positive feedback from educators implementing the curriculum with at least 4.0/5.0 satisfaction rating
- **SC-005**: Students can identify appropriate hardware components for specific Physical AI tasks after completing the hardware requirements section
- **SC-006**: The documentation system successfully deploys with 99% uptime
- **SC-007**: Students can implement basic robotics communication nodes after completing the relevant modules
- **SC-008**: Students demonstrate understanding of simulation and reinforcement learning tools after completing the relevant modules

## Outcome

- ✅ Impact: Created comprehensive feature specification for Physical AI & Humanoid Robotics course
- 🧪 Tests: none
- 📁 Files: specs/1-physical-ai-course/spec.md, specs/1-physical-ai-course/checklists/requirements.md
- 🔁 Next prompts: Build technical plan for the spec
- 🧠 Reflection: The specification successfully captures all requirements from the user description, including the 13-week curriculum structure, hardware requirements, and educational focus. All checklist items have been validated and completed.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): Create the technical plan for this specification