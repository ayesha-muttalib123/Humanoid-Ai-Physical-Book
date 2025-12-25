---
id: 6
title: Analyze Consistency Physical AI Course
stage: analyze
date: 2025-01-14
surface: agent
model: Qwen
feature: physical-ai-course
branch: 1-physical-ai-course
user: user
command: /sp.analyze
labels: [analyze,spec,plan,tasks,consistency,physical-ai,robotics,education]
links:
  spec: specs/1-physical-ai-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-physical-ai-course/spec.md
 - specs/1-physical-ai-course/plan.md
 - specs/1-physical-ai-course/tasks.md
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Perform a non-destructive cross-artifact consistency and quality analysis across spec.md, plan.md, and tasks.md after task generation.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Goal

Identify inconsistencies, duplications, ambiguities, and underspecified items across the three core artifacts (`spec.md`, `plan.md`, `tasks.md`) before implementation. This command MUST run only after `/sp.tasks` has successfully produced a complete `tasks.md`.

## Operating Constraints

**STRICTLY READ-ONLY**: Do **not** modify any files. Output a structured analysis report. Offer an optional remediation plan (user must explicitly approve before any follow-up editing commands would be invoked manually).

**Constitution Authority**: The project constitution (`.specify/memory/constitution.md`) is **non-negotiable** within this analysis scope. Constitution conflicts are automatically CRITICAL and require adjustment of the spec, plan, or tasks—not dilution, reinterpretation, or silent ignoring of the principle. If a principle itself needs to change, that must occur in a separate, explicit constitution update outside `/sp.analyze`.

## Execution Steps

### 1. Initialize Analysis Context

Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json -RequireTasks -IncludeTasks` once from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS. Derive absolute paths:

- SPEC = FEATURE_DIR/spec.md
- PLAN = FEATURE_DIR/plan.md
- TASKS = FEATURE_DIR/tasks.md

Abort with an error message if any required file is missing (instruct the user to run missing prerequisite command).
For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

### 2. Load Artifacts (Progressive Disclosure)

Load only the minimal necessary context from each artifact:

**From spec.md:**

- Overview/Context
- Functional Requirements
- Non-Functional Requirements
- User Stories
- Edge Cases (if present)

**From plan.md:**

- Architecture/stack choices
- Data Model references
- Phases
- Technical constraints

**From tasks.md:**

- Task IDs
- Descriptions
- Phase grouping
- Parallel markers [P]
- Referenced file paths

**From constitution:**

- Load `.specify/memory/constitution.md` for principle validation

### 3. Build Semantic Models

Create internal representations (do not include raw artifacts in output):

- **Requirements inventory**: Each functional + non-functional requirement with a stable key (derive slug based on imperative phrase; e.g., "User can upload file" → `user-can-upload-file`)
- **User story/action inventory**: Discrete user actions with acceptance criteria
- **Task coverage mapping**: Map each task to one or more requirements or stories (inference by keyword / explicit reference patterns like IDs or key phrases)
- **Constitution rule set**: Extract principle names and MUST/SHOULD normative statements

### 4. Detection Passes (Token-Efficient Analysis)

Focus on high-signal findings. Limit to 50 findings total; aggregate remainder in overflow summary.

#### A. Duplication Detection

- Identify near-duplicate requirements
- Mark lower-quality phrasing for consolidation

#### B. Ambiguity Detection

- Flag vague adjectives (fast, scalable, secure, intuitive, robust) lacking measurable criteria
- Flag unresolved placeholders (TODO, TKTK, ???, `<placeholder>`, etc.)

#### C. Underspecification

- Requirements with verbs but missing object or measurable outcome
- User stories missing acceptance criteria alignment
- Tasks referencing files or components not defined in spec/plan

#### D. Constitution Alignment

- Any requirement or plan element conflicting with a MUST principle
- Missing mandated sections or quality gates from constitution

#### E. Coverage Gaps

- Requirements with zero associated tasks
- Tasks with no mapped requirement/story
- Non-functional requirements not reflected in tasks (e.g., performance, security)

#### F. Inconsistency

- Terminology drift (same concept named differently across files)
- Data entities referenced in plan but absent in spec (or vice versa)
- Task ordering contradictions (e.g., integration tasks before foundational setup tasks without dependency note)
- Conflicting requirements (e.g., one requires Next.js while other specifies Vue)

### 5. Severity Assignment

Use this heuristic to prioritize findings:

- **CRITICAL**: Violates constitution MUST, missing core spec artifact, or requirement with zero coverage that blocks baseline functionality
- **HIGH**: Duplicate or conflicting requirement, ambiguous security/performance attribute, untestable acceptance criterion
- **MEDIUM**: Terminology drift, missing non-functional task coverage, underspecified edge case
- **LOW**: Style/wording improvements, minor redundancy not affecting execution order

### 6. Produce Compact Analysis Report

Output a Markdown report (no file writes) with the following structure:

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| A1 | Duplication | HIGH | spec.md:L120-134 | Two similar requirements ... | Merge phrasing; keep clearer version |

(Add one row per finding; generate stable IDs prefixed by category initial.)

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|

**Constitution Alignment Issues:** (if any)

**Unmapped Tasks:** (if any)

**Metrics:**

- Total Requirements
- Total Tasks
- Coverage % (requirements with >=1 task)
- Ambiguity Count
- Duplication Count
- Critical Issues Count

### 7. Provide Next Actions

At end of report, output a concise Next Actions block:

- If CRITICAL issues exist: Recommend resolving before `/sp.implement`
- If only LOW/MEDIUM: User may proceed, but provide improvement suggestions
- Provide explicit command suggestions: e.g., "Run /sp.specify with refinement", "Run /sp.plan to adjust architecture", "Manually edit tasks.md to add coverage for 'performance-metrics'"

### 8. Offer Remediation

Ask the user: "Would you like me to suggest concrete remediation edits for the top N issues?" (Do NOT apply them automatically.)

## Operating Principles

### Context Efficiency

- **Minimal high-signal tokens**: Focus on actionable findings, not exhaustive documentation
- **Progressive disclosure**: Load artifacts incrementally; don't dump all content into analysis
- **Token-efficient output**: Limit findings table to 50 rows; summarize overflow
- **Deterministic results**: Rerunning without changes should produce consistent IDs and counts

### Analysis Guidelines

- **NEVER modify files** (this is read-only analysis)
- **NEVER hallucinate missing sections** (if absent, report them accurately)
- **Prioritize constitution violations** (these are always CRITICAL)
- **Use examples over exhaustive rules** (cite specific instances, not generic patterns)
- **Report zero issues gracefully** (emit success report with coverage statistics)

## Context

Analyze for consistency: All original content covered across 52 chapters? Prices updated? No gaps in modules/weeks?

## Response snapshot

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| D1 | Duplication | MEDIUM | spec.md, plan.md | Hardware requirements mentioned in both spec and plan without clear distinction | Consolidate hardware requirements in one location and reference from the other |
| C1 | Coverage Gap | CRITICAL | tasks.md | Missing tasks for all 52 weekly chapters (only 48 out of 52 chapters have tasks) | Add tasks for missing chapters: 3-1-urdf-overview.md, 4-1-urdf-detailed.md, 4-2-ros2-launch-system.md, 4-3-ros2-parameter-system.md |
| C2 | Constitution Alignment | CRITICAL | tasks.md | Missing safety considerations in tasks despite "Safety-First Architecture" principle | Add safety-focused tasks throughout hardware and implementation content |
| A1 | Ambiguity | HIGH | spec.md | "Updated prices" mentioned but no specific date or source for updates | Clarify when prices were last updated and how often they should be refreshed |
| A2 | Ambiguity | MEDIUM | plan.md | "Fast loading of pages" lacks measurable criteria | Define specific performance metrics (e.g., <3s load time) |
| U1 | Underspecification | MEDIUM | tasks.md | Tasks reference files that are not defined in spec/plan | Ensure all referenced files and components are properly defined in spec/plan |
| I1 | Inconsistency | LOW | spec.md, plan.md | Week 3 has 4 chapters in spec but 5 tasks in plan | Align the number of chapters/tasks per week |
| I2 | Inconsistency | LOW | spec.md, tasks.md | Module content tasks don't fully align with spec requirements | Ensure all module requirements from spec are covered in tasks |
| I3 | Inconsistency | MEDIUM | tasks.md | Some hardware files referenced but not created in tasks | Create tasks for all hardware comparison and specification files |

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|
| provide-complete-13-week-curriculum | Yes | T038-T115 | All 52 chapters covered with 48 tasks (missing 4 chapters) |
| detailed-intro-sections | Yes | T011-T015, T028-T030 | Intro content properly covered |
| cover-modules-1-4 | Yes | T116-T124 | Module content covered |
| weekly-breakdown-52-chapters | No | T038-T115 | Missing 4 specific chapter files |
| hardware-requirements-updated-prices | Partial | T132-T141 | Price updates need clarification |
| assessment-options | Yes | T034, T147-T149 | Assessment content covered |
| lab-setup-options | Yes | T145-T151 | Lab setup covered |
| docusaurus-template | Yes | T001-T009 | Docusaurus setup covered |
| rich-mdx-content | Yes | T126-T130 | MDX content covered |
| educational-step-by-step | Yes | Throughout | Educational approach covered |
| automatic-deployment | Yes | T009 | Deployment covered |
| nvidia-isaac-content | Yes | T081-T090 | Isaac content covered |
| gazebo-unity-integration | Yes | T069-T078 | Gazebo/Unity content covered |
| humanoid-content | Yes | T099-T108 | Humanoid content covered |
| conversational-robotics | Yes | T111-T114 | Conversational robotics covered |

**Constitution Alignment Issues:**
- Safety-first architecture principle (III) not adequately reflected in tasks
- Educational excellence principle (VI) partially covered but could be strengthened

**Unmapped Tasks:**
- T164-T177 (Polish & Cross-Cutting Concerns) - These are valid general tasks but not directly mapped to specific spec requirements

**Metrics:**
- Total Requirements: 15 core functional requirements
- Total Tasks: 177 tasks
- Coverage %: ~90% (14 out of 15 requirements have >=1 task)
- Ambiguity Count: 2
- Duplication Count: 1
- Critical Issues Count: 2

## Next Actions

CRITICAL issues must be resolved before proceeding with `/sp.implement`. The missing chapters and safety considerations are fundamental to the project's constitution and core requirements.

1. Add tasks for the 4 missing weekly chapters (URDF and ROS2 content)
2. Integrate safety considerations throughout the hardware and implementation tasks as required by the constitution
3. Clarify the hardware pricing update schedule

## Remediation

Would you like me to suggest concrete remediation edits for the top 3 critical issues (missing chapter tasks, safety integration, and hardware pricing clarification)?

## Outcome

- ✅ Impact: Identified critical inconsistencies between spec, plan, and tasks; highlighted constitution alignment issues
- 🧪 Tests: none
- 📁 Files: specs/1-physical-ai-course/spec.md, specs/1-physical-ai-course/plan.md, specs/1-physical-ai-course/tasks.md, .specify/memory/constitution.md
- 🔁 Next prompts: Suggest concrete remediation edits for critical issues
- 🧠 Reflection: The analysis revealed critical gaps in chapter coverage and safety considerations that must be addressed before implementation.

## Evaluation notes (flywheel)

- Failure modes observed: Missing chapter tasks, safety considerations not adequately addressed
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): Add missing chapter tasks and safety-focused tasks