
# FRC Team 190 2k25 Robot Code - Technical Overview


This repository contains the complete software for the FRC Team 190 2k25 robot. This document outlines the core architectural decisions, subsystem interactions, and operational strategies.

![Robot Image](https://www.chiefdelphi.com/uploads/default/original/4X/9/c/2/9c2b01f10837312b45fb2918aae24ecef5c673ec.jpeg)

---

## 1. Code Philosophy and Structure

Our codebase is designed with a modular, scalable architecture that reflects best practices in FRC programming. The primary goal is to create clean, maintainable, and easily navigable code, adhering to the standards outlined at [team-190.github.io/190-Robot-Code-Standards/](https://team-190.github.io/190-Robot-Code-Standards/).

* **Command-Based Framework:** We utilize WPILib's command-based framework to structure robot actions. The `commands` package contains command classes that control one or more subsystems, allowing for complex sequences and parallel execution.
* **AdvantageKit Integration:** The main robot class (`Robot.java`) extends `LoggedRobot` to integrate AdvantageKit for advanced logging, telemetry, and replay. This is critical for diagnosing issues and optimizing performance.
* **Robot Container:** `RobotContainer.java` is the central hub for connecting joystick inputs to commands, configuring autonomous routines, and managing interactions between subsystems.
* **Constants Class:** The `Constants.java` file centralizes operational parameters like motor speeds and sensor thresholds, making it easy to switch between different robot versions and operating modes (e.g., `REAL`, `SIM`, `REPLAY`).

---

## 2. Multi-Robot Architecture & Hardware Abstraction

A key feature of our project is the support for multiple robot versions within a single codebase.

* **Shared Codebase:** This allows us to develop and test multiple robot designs (`v0_funky`, `v1_stackUp`, `v2_redundancy`) while keeping all code in one place. Shared components (like `drive`) are centralized, while robot-specific subsystems are isolated in their respective packages.
* **Hardware Abstraction Layer (IO Classes):** Each subsystem is paired with an `IO` interface (e.g., `ModuleIO`, `ElevatorIO`) that defines its capabilities. We have multiple implementations for this interface:
    * `ModuleIOTalonFX`: For CTRE motor controllers.
    * `ModuleIOSim`: For running simulations without hardware.
    This abstraction allows for development and testing in simulation before deploying to a physical robot, streamlining our workflow.

---

## 3. Deployment, Security, and Version Control

* **Automatic Deploy Verification:** To prevent deploying incorrect code to our competition robot, we use a "two-factor authentication" system. Each robot has a unique identifier string. During deployment, a script cross-references this ID with the desired robot defined in the code. The deployment only proceeds if they match.
* **GitHub Workflow:** We use a public GitHub repository for collaboration.
    * `main` branch: Hosts tested, stable code.
    * `development` branch: Hosts code currently being worked on.
    * `feature-*` branches: Used for developing new features. Work is merged into `development` via pull requests after review and testing.
    * **Issues:** We use GitHub Issues to track bugs, features, and requests from other sub-teams.

---

## 4. Superstructure State Machine (V2_Redundancy)

The `V2_Redundancy` robot utilizes a finite state machine (FSM) to manage the complex interactions between its scoring and intake mechanisms, preventing collisions and ensuring safe, repeatable movements. This was inspired by [FRC 6328](https://github.com/Mechanical-Advantage/RobotCode2025Public)

![Superstructure Diagram](https://i.postimg.cc/MGX6T6Gg/temp-Image3x-Kc5t.avif)

* **`V2_RedundancySuperstructure.java`:** The central class that orchestrates all other subsystems. It uses a `JGraphT` graph to represent all possible states and transitions.
* **`V2_RedundancySuperstructureStates.java`:** An enum defining every possible state of the robot (e.g., `STOW_DOWN`, `INTAKE_FLOOR`, `SCORE_L4`).
* **`V2_RedundancySuperstructurePose.java`:** Defines the physical setpoints (positions) for each subsystem (Elevator, Arm, Intake, Funnel) that correspond to a given state.
* **`V2_RedundancySuperstructureAction.java`:** Defines the active behaviors (roller speeds) for each subsystem in a given state.
* **`V2_RedundancySuperstructureEdges.java`:** This crucial class builds the state graph. It defines every valid transition (edge) between states and associates a `Command` with each one. The FSM uses a Breadth-First Search (BFS) on this graph to find the shortest path from the current state to the desired goal state, scheduling the necessary commands to execute the transition safely.

---

## 5. Subsystem Breakdown

### Drivetrain
* **Function:** Controls the 4-module swerve drive using Kraken X60 motors for both drive and steering.
* **Odometry:**
    * **Internal:** Uses motor feedback (position and velocity) to determine field-relative position. Prone to drift from wheel slip and collisions.
    * **External (Vision Correction):** A 3-camera Limelight system detects AprilTags to continuously correct for odometry drift, enhancing autonomous consistency.
* **Interaction:** Provides pose data to `RobotStateLL`. Controlled by `DriveCommands` in teleop and `Choreo` trajectories in auto.

### Vision (Limelight)
* **Function:** Provides precise robot localization.
* **Hardware:** Three Limelight cameras (one center, two angled on edges).
* **Logic:** Uses `MegaTag2` to get pose estimations from each camera. Tags that are closer are trusted more (lower standard deviation). This fused data corrects the internal odometry in `RobotStateLL`.

### Elevator
* **Function:** Manages the primary vertical lift for scoring.
* **Hardware:** Multiple CTRE Kraken X60 motors.
* **Interaction:** A core component of the `Superstructure`, moving to heights defined by the current `V2_RedundancySuperstructurePose`.

### Funnel
* **Function:** Controls the pivoting "Clap Daddy" mechanism to guide and secure game pieces.
* **Hardware:** CTRE Kraken X60 motor, CANcoder for absolute position.
* **Interaction:** Coordinated by the `Superstructure` to open for intake and close to secure pieces.

### Manipulator (V2)
* **Function:** Primary scoring and intake mechanism for "Coral" and "Algae".
* **Hardware:** CTRE Kraken X60 motors for arm rotation and roller control.
* **Interaction:** Moves to various angles (`ManipulatorArmState`) as directed by the `Superstructure`.

### Intake (V2)
* **Function:** A linear extending intake for floor pickup of game pieces.
* **Hardware:** CTRE Kraken X60 motors for extension and roller.
* **Interaction:** Extends and retracts based on the `Superstructure` state, primarily for `INTAKE_FLOOR`.

### Climber
* **Function:** Manages the endgame climb.
* **Hardware:** CTRE Kraken X60 motor, redundant digital inputs for state detection.
* **Interaction:** Sequenced by `CompositeCommands` to ensure other mechanisms are stowed before climbing.

### LEDs
* **Function:** Provides visual feedback to drivers.
* **Hardware:** Addressable LED strip.
* **Interaction:** Reads state from `RobotStateLL` (e.g., `isAutoAligning`, `hasAlgae`) to display patterns.

---

## 6. Autonomous and Teleop Control

### Autonomous
* **Path Generation:** We use **Choreo** to design smooth, efficient, and constraint-aware autonomous trajectories.
* **Path Execution:** The `LoggedAutoFactory` loads these trajectories. `AutonomousCommands.java` sequences them with other actions (like intake/scoring) to create full auto routines.
* **Auto-Alignment:** Before scoring, a vision-based command (`DriveCommands.autoAlignReefCoral`) uses Limelight data to precisely align the robot to the reef, compensating for any path inaccuracies.

### Teleoperated Control
* **Two-Driver System:** We use two Xbox controllers for intuitive control.
    * **Driver:** Manages robot movement (translation and rotation) and triggers high-level intake/scoring sequences.
    * **Operator:** Manages the end-effector functions (Elevator height, Funnel, Manipulator, Climber).
* **Game Piece Sensing (V2):**
    * **Coral:** Detected by monitoring current spikes on the intake motor when it's trying to maintain position against the coral piece. A `RobotState` variable ensures this logic only runs when an intake command is active.
    * **Algae:** Detected by monitoring the manipulator roller's velocity and acceleration. When algae is detected, a dynamic holding voltage is applied to the roller to prevent it from slipping, adjusting based on motor current.
