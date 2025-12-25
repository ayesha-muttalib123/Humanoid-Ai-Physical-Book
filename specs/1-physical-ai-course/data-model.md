# Data Model for Physical AI & Humanoid Robotics Course

## Course
- **title**: string (Physical AI & Humanoid Robotics: Bridging Digital AI to Embodied Intelligence)
- **description**: string (Comprehensive 13-week curriculum covering physical AI and humanoid robotics)
- **prerequisites**: string (Basic programming knowledge and introductory robotics concepts)
- **learningOutcomes**: list of strings (5 key learning outcomes from spec)
- **duration**: number (13 weeks)
- **totalChapters**: number (52 chapters)
- **assessmentTypes**: list of strings (Multiple choice quizzes, practical coding assignments)
- **completionRequirements**: list of strings (Complete all 52 chapters, pass assessments, complete capstone project)

## Module
- **id**: string (module-1, module-2, etc.)
- **title**: string (e.g., "Foundations of Physical AI")
- **description**: string (Overview of the module content)
- **weeks**: list of Week references
- **chapters**: list of Chapter references
- **learningObjectives**: list of strings (What students will learn in this module)

## Week
- **id**: string (week-1, week-2, etc.)
- **title**: string (e.g., "Introduction to Physical AI")
- **chapters**: list of 4 Chapter references
- **description**: string (Overview of the week's content)
- **learningGoals**: list of strings (What students will achieve by the end of the week)

## Chapter
- **id**: string (e.g., "1-1-foundations-physical-ai")
- **title**: string (e.g., "Foundations of Physical AI")
- **content**: string (MDX content with text, code, diagrams)
- **weekId**: string (Reference to parent week)
- **moduleId**: string (Reference to parent module)
- **prerequisites**: list of strings (What students should know before reading)
- **learningObjectives**: list of strings (What students will learn)
- **exercises**: list of Exercise objects
- **codeExamples**: list of CodeExample objects
- **diagrams**: list of Diagram objects

## Exercise
- **id**: string
- **type**: string (quiz, coding-assignment, discussion)
- **question**: string
- **options**: list of strings (for multiple choice)
- **correctAnswer**: string
- **explanation**: string (Explanation of the correct answer)
- **difficulty**: string (beginner, intermediate, advanced)

## CodeExample
- **id**: string
- **title**: string
- **language**: string (python, c++, etc.)
- **code**: string (The actual code)
- **explanation**: string (Explanation of the code)
- **relatedConcepts**: list of strings (Which concepts this example demonstrates)

## Diagram
- **id**: string
- **title**: string
- **type**: string (mermaid, svg, etc.)
- **content**: string (The diagram definition)
- **explanation**: string (What the diagram illustrates)
- **relatedConcepts**: list of strings (Which concepts this diagram demonstrates)

## Assessment
- **id**: string
- **type**: string (quiz, assignment, project)
- **title**: string
- **description**: string
- **moduleId**: string (Which module this assessment belongs to)
- **weekId**: string (Which week this assessment belongs to, if applicable)
- **questions**: list of Exercise objects
- **gradingCriteria**: string (How the assessment will be graded)
- **dueDate**: string (When the assessment is due, if applicable)

## Hardware
- **id**: string (e.g., "jetson-orin-nano-super")
- **name**: string (e.g., "Jetson Orin Nano Super")
- **description**: string (Overview of the hardware)
- **price**: number (Price in USD)
- **specifications**: object (Technical specifications)
- **useCases**: list of strings (How this hardware is used in the course)
- **compatibility**: list of strings (Which systems this hardware works with)
- **setupGuide**: string (Link to or content of setup guide)

## LabEnvironment
- **id**: string (on-premise, cloud)
- **type**: string (on-premise, cloud)
- **description**: string (Overview of the lab environment)
- **requirements**: list of strings (What's needed to set up this environment)
- **setupGuide**: string (Detailed instructions for setup)
- **pros**: list of strings (Advantages of this environment)
- **cons**: list of strings (Disadvantages of this environment)
- **cost**: string (Estimated cost of this environment)