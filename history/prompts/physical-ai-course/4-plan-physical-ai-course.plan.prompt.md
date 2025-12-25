---
id: 4
title: Plan Physical AI Course
stage: plan
date: 2025-01-14
surface: agent
model: Qwen
feature: physical-ai-course
branch: 1-physical-ai-course
user: user
command: /sp.plan
labels: [plan,physical-ai,robotics,education,course,docusaurus]
links:
  spec: specs/1-physical-ai-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-course/plan.md
 - specs/1-physical-ai-course/research.md
 - specs/1-physical-ai-course/data-model.md
 - specs/1-physical-ai-course/quickstart.md
 - specs/1-physical-ai-course/docusaurus.config.js
 - specs/1-physical-ai-course/sidebars.js
tests:
 - none
---

## Prompt

Execute the implementation planning workflow using the plan template to generate design artifacts.
handoffs: 
  - label: Create Tasks
    agent: sp.tasks
    prompt: Break the plan into tasks
    send: true
  - label: Create Checklist
    agent: sp.checklist
    prompt: Create a checklist for the following domain...

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType qwen`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

## Response snapshot

# Implementation Plan: Physical AI & Humanoid Robotics Course

**Branch**: `1-physical-ai-course` | **Date**: 2025-01-14 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/1-physical-ai-course/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive 13-week Physical AI & Humanoid Robotics course using Docusaurus as the documentation platform. The course will include 52 chapters (4 per week), hardware requirements with updated prices, assessments, and deployment to GitHub Pages. The curriculum will cover embodied intelligence, hardware fundamentals, ROS 2, Gazebo/Unity, NVIDIA Isaac, and humanoid development with a focus on both theoretical concepts and practical implementation.

## Technical Context

**Language/Version**: Markdown/MDX, JavaScript (Node.js 18+ for Docusaurus)
**Primary Dependencies**: Docusaurus 2.x, React, GitHub Pages
**Storage**: Git repository hosting the documentation source
**Testing**: Manual validation of content accuracy and navigation
**Target Platform**: Web-based documentation accessible via GitHub Pages
**Project Type**: Documentation/static site
**Performance Goals**: Fast loading of pages, responsive navigation, accessible on multiple devices
**Constraints**: Must support rich content including code blocks, diagrams (Mermaid), tables, and interactive elements
**Scale/Scope**: 52+ chapters, hardware tables, 13-week curriculum with 4 chapters per week

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Embodied Intelligence First**: Curriculum must emphasize physical embodiment in AI systems
- **Hardware-Software Co-Design**: Content must integrate hardware and software concepts
- **Safety-First Architecture**: Safety considerations must be integrated throughout
- **Modular System Design**: Course structure must be modular with well-defined sections
- **Open-Source and Reproducible Research**: All content must be open-source and reproducible
- **Educational Excellence**: All content must serve educational purposes with clear learning pathways

## Project Structure

### Documentation (this feature)

```text
specs/1-physical-ai-course/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!-- 
ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
for this feature. Delete unused options and expand the chosen structure with
real paths (e.g., apps/admin, packages/something). The delivered plan must
not include Option labels.
-->

```text
# Option 1: Single project (DEFAULT)
docs/
├── intro/
│   ├── theme.md
│   ├── goal.md
│   ├── overview.md
│   ├── why-physical-ai-matters.md
│   └── learning-outcomes.md
├── modules/
│   ├── module-1/
│   │   ├── index.md
│   │   ├── foundations-physical-ai.md
│   │   ├── embodied-cognition.md
│   │   └── sensorimotor-integration.md
│   ├── module-2/
│   │   ├── index.md
│   │   ├── robot-platforms.md
│   │   ├── sensors-actuators.md
│   │   └── real-time-control.md
│   ├── module-3/
│   │   ├── index.md
│   │   ├── ros2-introduction.md
│   │   ├── perception-algorithms.md
│   │   └── planning-control.md
│   └── module-4/
│       ├── index.md
│       ├── system-integration.md
│       ├── testing-real-world.md
│       └── deployment.md
├── weeks/
│   ├── week-1/
│   │   ├── index.md
│   │   ├── 1-1-foundations-physical-ai.md
│   │   ├── 1-2-embodied-intelligence-principles.md
│   │   ├── 1-3-digital-ai-physical-laws.md
│   │   └── 1-4-humanoid-robotics-landscape.md
│   ├── week-2/
│   │   ├── index.md
│   │   ├── 2-1-lidar-sensors.md
│   │   ├── 2-2-depth-cameras.md
│   │   ├── 2-3-imus-balance.md
│   │   └── 2-4-force-torque-sensors.md
│   ├── week-3/
│   │   ├── index.md
│   │   ├── 3-1-ros2-architecture.md
│   │   ├── 3-2-nodes-topics-services-actions.md
│   │   ├── 3-3-ros2-packages.md
│   │   └── 3-4-urdf-overview.md
│   ├── week-4/
│   │   ├── index.md
│   │   ├── 4-1-urdf-detailed.md
│   │   ├── 4-2-ros2-launch-system.md
│   │   ├── 4-3-ros2-parameter-system.md
│   │   └── 4-4-ros2-tools.md
│   ├── week-5/
│   │   ├── index.md
│   │   ├── 5-1-ros2-navigation.md
│   │   ├── 5-2-ros2-perception-pipeline.md
│   │   ├── 5-3-ros2-control-framework.md
│   │   └── 5-4-ros2-rosbridge.md
│   ├── week-6/
│   │   ├── index.md
│   │   ├── 6-1-gazebo-setup.md
│   │   ├── 6-2-urdf-sdf-conversion.md
│   │   ├── 6-3-physics-simulation.md
│   │   └── 6-4-sensors-simulation.md
│   ├── week-7/
│   │   ├── index.md
│   │   ├── 7-1-unity-visualization.md
│   │   ├── 7-2-unity-robotics-hub.md
│   │   ├── 7-3-simulation-workflows.md
│   │   └── 7-4-sim-real-comparison.md
│   ├── week-8/
│   │   ├── index.md
│   │   ├── 8-1-isaac-sim-introduction.md
│   │   ├── 8-2-isaac-sim-perception.md
│   │   ├── 8-3-isaac-sim-rl.md
│   │   └── 8-4-sim-to-real-transfer.md
│   ├── week-9/
│   │   ├── index.md
│   │   ├── 9-1-isaac-sim-advanced.md
│   │   ├── 9-2-isaac-sim-rl-examples.md
│   │   ├── 9-3-rl-algorithms-robotics.md
│   │   └── 9-4-sim-to-real-challenges.md
│   ├── week-10/
│   │   ├── index.md
│   │   ├── 10-1-advanced-rl-techniques.md
│   │   ├── 10-2-sim-to-real-optimization.md
│   │   ├── 10-3-embodied-ai-research.md
│   │   └── 10-4-project-planning.md
│   ├── week-11/
│   │   ├── index.md
│   │   ├── 11-1-humanoid-kinematics.md
│   │   ├── 11-2-humanoid-locomotion.md
│   │   ├── 11-3-humanoid-manipulation.md
│   │   └── 11-4-humanoid-interaction.md
│   ├── week-12/
│   │   ├── index.md
│   │   ├── 12-1-humanoid-control-systems.md
│   │   ├── 12-2-humanoid-safety-protocols.md
│   │   ├── 12-3-humanoid-project-implementation.md
│   │   └── 12-4-project-refinement.md
│   └── week-13/
│       ├── index.md
│       ├── 13-1-gpt-integration.md
│       ├── 13-2-speech-processing.md
│       ├── 13-3-multi-modal-integration.md
│       └── 13-4-capstone-overview.md
├── assessments/
│   ├── index.md
│   ├── weekly-assignments.md
│   ├── midterm-project.md
│   └── capstone-project.md
├── hardware/
│   ├── index.md
│   ├── jetson-orin-nano-super.md
│   ├── unitree-go2.md
│   ├── unitree-g1.md
│   ├── realsense-d435i.md
│   └── hardware-comparison-table.md
├── lab-options/
│   ├── index.md
│   ├── on-premise-setup.md
│   └── cloud-options.md
└── appendix/
    ├── glossary.md
    ├── references.md
    └── further-reading.md
```

**Structure Decision**: Single documentation project using Docusaurus with a hierarchical structure that matches the 13-week curriculum with 4 chapters per week. The structure includes dedicated sections for intro content, modules, weekly breakdowns, assessments, hardware requirements, lab options, and appendices.

## Phase 0: Research & Resolution of Clarifications

### Research Tasks
1. **Docusaurus Configuration**: Research best practices for Docusaurus configuration for educational content
2. **MDX Content**: Research how to properly structure educational content with code blocks, diagrams, and interactive elements
3. **Mermaid Diagrams**: Research how to effectively use Mermaid diagrams for ROS 2 architecture and URDF
4. **GitHub Pages Deployment**: Research GitHub Actions workflows for Docusaurus deployment
5. **Assessment Integration**: Research how to structure multiple choice quizzes and coding assignments in Docusaurus

### Research Outcomes
- **Docusaurus Setup**: Will use Docusaurus classic template with custom sidebar configuration
- **Content Structure**: Will use MDX for rich content including code tabs, diagrams, and interactive elements
- **Deployment**: Will use GitHub Actions to automatically deploy to GitHub Pages
- **Assessments**: Will include quiz questions directly in content with expandable answers

## Phase 1: Design & Contracts

### Data Model
- **Course**: Contains title, description, learning outcomes, prerequisites
- **Module**: Contains title, description, learning objectives, chapters
- **Week**: Contains title, description, 4 chapters
- **Chapter**: Contains title, content, code examples, diagrams, exercises
- **Assessment**: Contains type (quiz, assignment, project), content, criteria
- **Hardware**: Contains name, description, price, specifications, use cases

### Quickstart Guide
1. Clone the repository
2. Install dependencies: `npm install`
3. Start development server: `npm start`
4. Edit content in the docs/ directory
5. Build for production: `npm run build`
6. Deploy to GitHub Pages via Actions

### Contracts
- Docusaurus site configuration (docusaurus.config.js)
- Sidebar configuration (sidebars.js) with hierarchical structure
- GitHub Actions workflow for deployment

**Structure Decision**: Single documentation project using Docusaurus with a hierarchical structure that matches the 13-week curriculum with 4 chapters per week. The structure includes dedicated sections for intro content, modules, weekly breakdowns, assessments, hardware requirements, lab options, and appendices.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Physical AI & Humanoid Robotics course
- 🧪 Tests: none
- 📁 Files: specs/1-physical-ai-course/plan.md, specs/1-physical-ai-course/research.md, specs/1-physical-ai-course/data-model.md, specs/1-physical-ai-course/quickstart.md, specs/1-physical-ai-course/docusaurus.config.js, specs/1-physical-ai-course/sidebars.js
- 🔁 Next prompts: Break the plan into tasks
- 🧠 Reflection: Successfully created a complete implementation plan including technical architecture, data models, and configuration files needed to build the 13-week Physical AI course with Docusaurus.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): Create tasks from the plan