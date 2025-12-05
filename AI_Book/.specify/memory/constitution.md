<!-- Sync Impact Report:
Version change: 0.5.0 -> 0.6.0
Modified principles:
  - Added Principle 12: Module Content Structure
Added sections:
  - N/A
Removed sections:
  - N/A
Templates requiring updates:
  - .specify/templates/plan-template.md: Not Found
  - .specify/templates/spec-template.md: Not Found
  - .specify/templates/tasks-template.md: Not Found
  - .specify/templates/commands/*.md: Not Found
Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Needs to be set on initial ratification
-->
# Project Constitution: AI_Book

## Governance

- **Constitution Version**: 0.6.0
- **Ratification Date**: TODO(RATIFICATION_DATE): Needs to be set on initial ratification
- **Last Amended Date**: 2025-12-06
- **Amendment Procedure**: All amendments MUST be proposed as pull requests, reviewed, and approved by a consensus of core contributors. Minor (patch) updates may be fast-tracked; major (breaking) changes require a formal RFC process.
- **Versioning Policy**: This constitution follows Semantic Versioning. MAJOR versions for backward-incompatible changes, MINOR for new principles or sections, PATCH for clarifications or typo fixes.
- **Compliance Review**: Compliance with these principles will be reviewed quarterly during project retrospectives.

## Project Principles

### Principle 1: Clarity and Simplicity
**Description**: Solutions MUST prioritize clarity and simplicity. Code should be easy to understand, maintain, and extend. Avoid unnecessary complexity or over-engineering.
**Rationale**: Complex systems are harder to debug, more prone to errors, and slower to develop. Simplicity fosters collaboration and reduces cognitive load.

### Principle 2: Testability and Quality
**Description**: All new features and bug fixes MUST be accompanied by comprehensive automated tests. Code MUST meet defined quality gates (e.g., linting, static analysis) before merging.
**Rationale**: High test coverage and robust quality checks prevent regressions, ensure reliability, and build confidence in the codebase.

### Principle 3: Modularity and Reusability
**Description**: Components and modules SHOULD be designed with clear responsibilities and minimal dependencies, promoting reusability across the project.
**Rationale**: Modular design reduces duplication, improves maintainability, and allows for easier independent development and deployment of features.

### Principle 4: Performance and Efficiency
**Description**: Performance and resource efficiency SHOULD be considered throughout the development lifecycle, with critical paths optimized and monitored.
**Rationale**: Efficient systems provide a better user experience, reduce operational costs, and scale more effectively.

### Principle 5: Security by Design
**Description**: Security considerations MUST be integrated from the outset of design and implementation. All components handling sensitive data or exposed to external input MUST undergo security review.
**Rationale**: Proactive security measures prevent vulnerabilities, protect user data, and maintain trust.

### Principle 6: Documentation and Knowledge Sharing
**Description**: Key architectural decisions, API contracts, and complex functionalities MUST be documented clearly and kept up-to-date. Knowledge sharing among the team is encouraged.
**Rationale**: Good documentation reduces onboarding time for new contributors, ensures consistent understanding, and facilitates long-term project sustainability.

### Principle 7: Scientific Grounding and Currency
**Description**: All technical and theoretical claims MUST be backed by current research (ROS 2, Isaac Sim, VLA). Avoid speculative claims without academic citations. All concepts MUST be explained in a sequence that builds from fundamentals (Nodes \$\rightarrow\$ URDF \$\rightarrow\$ Simulation \$\rightarrow\$ Advanced AI).
**Rationale**: Strict adherence to peer-reviewed research and a structured pedagogical approach ensures factual accuracy, builds foundational understanding, and maintains the project\'s authority and relevance in rapidly evolving scientific fields.

### Principle 8: Content Formatting and Asset Management
**Description**: All chapter files MUST use MDX (.md or .mdx) format. All static assets (images, diagrams) MUST be stored in `static/img` and referenced with correct Docusaurus paths.
**Rationale**: Standardizing content and asset formats ensures consistency, improves maintainability, and leverages Docusaurus\'s optimized asset handling and rendering capabilities.

### Principle 9: Code Snippet and ROS 2 Framework Consistency
**Description**: All code snippets MUST be marked with the correct language (e.g., `python` or `bash`). ROS 2 code MUST exclusively use the `rclpy` (Python) framework for consistency and ease of understanding.
**Rationale**: Proper language marking enhances readability and syntax highlighting. Standardizing on `rclpy` for ROS 2 code ensures a uniform approach, simplifies examples, and reduces potential confusion for contributors and readers.

### Principle 10: Primary Theme Color
**Description**: The project\'s primary theme color MUST be `#007ACC` (a professional blue), set via the `--ifm-color-primary` CSS variable.
**Rationale**: Establishing a consistent brand color ensures a professional and unified visual identity across all documentation and interfaces, aligning with design guidelines.

### Principle 11: Hardware Relevance
**Description**: All modules MUST reference the Hardware Requirements defined in the project (e.g., the necessity of an RTX GPU, the role of the Jetson Orin) to maintain real-world relevance.
**Rationale**: Explicitly linking modules to specific hardware requirements ensures the practical applicability of the project\'s solutions, guides hardware-software co-design, and manages expectations regarding performance and compatibility.

### Principle 12: Module Content Structure
**Description**: All module content MUST follow a hierarchical structure: a main heading for the module, followed by subheadings for related content within that module. For example, a main heading 'Module ROS2' would have subheadings like 'Installation of ROS2'.
**Rationale**: A consistent content structure improves readability, navigability, and pedagogical effectiveness, making it easier for readers to follow complex topics and locate specific information.
