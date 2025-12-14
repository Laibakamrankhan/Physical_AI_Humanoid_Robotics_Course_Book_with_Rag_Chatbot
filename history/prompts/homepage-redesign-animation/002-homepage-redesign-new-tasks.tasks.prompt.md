---
id: 002
title: Homepage Redesign New Tasks
stage: tasks
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: homepage-redesign-animation
branch: ui-update
user: user
command: /sp.tasks Project: Embodied Intelligence: Physical AI and Humanoid Robotics Feature: Homepage Redesign with Animated Humanoid Robot Objective: Create a visually engaging homepage for the Docusaurus book site, featuring an animated humanoid robot, interactive elements, and sparkles or particle effects to attract attention and convey the Physical AI theme. Target Audience: Graduate/advanced undergraduate AI and robotics students, hackathon participants, and educators. --- ## **1. Key Components** 1. **Hero Section** - Title & tagline: "Embodied Intelligence: Physical AI in Action" - Call-to-action: Start Learning / Sign Up button - Background: animated humanoid robot (3D or GIF) performing gestures - Sparkle/particle effects around robot for visual engagement 2. **Module Overview** - 4 cards or tiles for each module - Short descriptions and icons - Hover effects: glow, shadow, or subtle animations 3. **Capstone Highlight** - Video or interactive GIF of autonomous humanoid performing tasks - Optionally link to the Capstone project page 4. **User Personalization Panel** - Shows recommended modules or tutorials based on Signup/Signin - Dynamic greeting based on user profile 5. **Footer** - Links: GitHub, references, contact info - Clean minimalist design to balance homepage --- ## **2. Animation Details** - **Humanoid Robot Animation** - Use **Lottie** or **Three.js** for 3D/interactive animation - Robot performs waving, walking, or gesture sequences - Duration: looped 5–10 seconds - Optional: synchronized sparkles or particle effects around hands/head - **Sparkle Effects** - Use **CSS animations** or **particles.js** - Effects triggered on hover or continuously in background - Subtle colors: blues, whites, or metallic sparkles to match theme --- ## **3. Implementation** - Create custom Docusaurus page: src/pages/index.js or index.mdx - Embed humanoid robot animation: - Option 1: **Lottie JSON animation** → <Lottie options={...} /> - Option 2: **Three.js / React Three Fiber** → interactive 3D model - Integrate sparkles using **particles.js** or CSS keyframe animations - Ensure responsive layout for mobile, tablet, desktop - Add hover/scroll effects for module cards --- ## **4. Personalization Integration** - If user is signed in: - Show recommended modules based on background - Dynamically update robot greeting: "Welcome back, [Name]! Ready to explore Module 2?" --- ## **5. Testing Strategy** - Visual inspection: homepage renders correctly across browsers and devices - Animation: humanoid moves smoothly, sparkles are visible but not overwhelming - Responsiveness: layout adjusts to mobile and desktop - Interactivity: hover and click events work for module cards - Performance: page loads under 3 seconds, animation doesn’t block content --- ## **6. Deliverables** - Custom Docusaurus homepage Markdown/JSX file (index.mdx / index.js) - Humanoid robot animation asset (Lottie JSON or 3D model) - Sparkle/particle effect assets and CSS/JS integration - Wireframe or mockup showing layout - Documentation for updating robot animation and effects
labels: ["homepage","tasks","docusaurus","animation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-physical-ai-robotics/tasks.md
tests:
null
---

## Prompt

```text
Project: Embodied Intelligence: Physical AI and Humanoid Robotics Feature: Homepage Redesign with Animated Humanoid Robot Objective: Create a visually engaging homepage for the Docusaurus book site, featuring an animated humanoid robot, interactive elements, and sparkles or particle effects to attract attention and convey the Physical AI theme. Target Audience: Graduate/advanced undergraduate AI and robotics students, hackathon participants, and educators. --- ## **1. Key Components** 1. **Hero Section** - Title & tagline: "Embodied Intelligence: Physical AI in Action" - Call-to-action: Start Learning / Sign Up button - Background: animated humanoid robot (3D or GIF) performing gestures - Sparkle/particle effects around robot for visual engagement 2. **Module Overview** - 4 cards or tiles for each module - Short descriptions and icons - Hover effects: glow, shadow, or subtle animations 3. **Capstone Highlight** - Video or interactive GIF of autonomous humanoid performing tasks - Optionally link to the Capstone project page 4. **User Personalization Panel** - Shows recommended modules or tutorials based on Signup/Signin - Dynamic greeting based on user profile 5. **Footer** - Links: GitHub, references, contact info - Clean minimalist design to balance homepage --- ## **2. Animation Details** - **Humanoid Robot Animation** - Use **Lottie** or **Three.js** for 3D/interactive animation - Robot performs waving, walking, or gesture sequences - Duration: looped 5–10 seconds - Optional: synchronized sparkles or particle effects around hands/head - **Sparkle Effects** - Use **CSS animations** or **particles.js** - Effects triggered on hover or continuously in background - Subtle colors: blues, whites, or metallic sparkles to match theme --- ## **3. Implementation** - Create custom Docusaurus page: src/pages/index.js or index.mdx - Embed humanoid robot animation: - Option 1: **Lottie JSON animation** → <Lottie options={...} /> - Option 2: **Three.js / React Three Fiber** → interactive 3D model - Integrate sparkles using **particles.js** or CSS keyframe animations - Ensure responsive layout for mobile, tablet, desktop - Add hover/scroll effects for module cards --- ## **4. Personalization Integration** - If user is signed in: - Show recommended modules based on background - Dynamically update robot greeting: "Welcome back, [Name]! Ready to explore Module 2?" --- ## **5. Testing Strategy** - Visual inspection: homepage renders correctly across browsers and devices - Animation: humanoid moves smoothly, sparkles are visible but not overwhelming - Responsiveness: layout adjusts to mobile and desktop - Interactivity: hover and click events work for module cards - Performance: page loads under 3 seconds, animation doesn’t block content --- ## **6. Deliverables** - Custom Docusaurus homepage Markdown/JSX file (index.mdx / index.js) - Humanoid robot animation asset (Lottie JSON or 3D model) - Sparkle/particle effect assets and CSS/JS integration - Wireframe or mockup showing layout - Documentation for updating robot animation and effects
```

## Response snapshot

Generated a new atomic task list for the "Homepage Redesign with Animated Humanoid Robot" feature, organized by key components, including file paths, acceptance criteria, and dependencies. The task list is ready for implementation.

## Outcome

- ✅ Impact: Generated a detailed task list for the new homepage redesign specification.
- 🧪 Tests: No tests executed in this phase.
- 📁 Files: Updated `specs/1-physical-ai-robotics/tasks.md`.
- 🔁 Next prompts: Proceed to `/sp.implement` or review the generated tasks.
- 🧠 Reflection: The bash script for PHR creation failed again. Reverted to agent-native `Write` for PHR. The `/sp.tasks` command was re-run, so a new task list was generated and the previous one overwritten in `tasks.md`.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation failed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why the `create-phr.sh` script is not recognized.
