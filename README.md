# MASLAB 2026 Team 3 — Autonomous Robot Software

Collaborative software developed for **MIT MASLAB 2026** by a 6-person team. The robot was designed to autonomously navigate randomized courses, identify Pringles cans as scoring objects, and interact with scoring zones using onboard sensing and vision.

## My Role

I served as **Head Programmer** on the team. This was a shared codebase developed by several team members; my primary contributions were in **robot motion and autonomous navigation**, including PID-based control, encoder/IMU odometry, motion profiling, path-following experiments, and movement-system testing/debugging. I had smaller involvement in computer-vision integration and testing.

I do **not** claim authorship of the entire repository. Files outside my primary contribution areas may have been written primarily by other team members.

## My Primary Contributions

- Developed and tuned robot movement and heading-control logic using PID-based feedback
- Implemented and debugged encoder/IMU odometry for robot pose estimation
- Developed trapezoidal/triangular motion-profile logic for smoother acceleration and deceleration
- Worked on quadratic Bézier path generation and pure-pursuit-style lookahead/path-following experiments
- Integrated motion-control code with the team's autonomous behavior
- Assisted with testing and integration of parts of the camera/OpenCV pipeline

## Technical Highlights

### Motion & Controls

The repository includes:

- PID controller utilities (`pid.py`)
- Differential-drive odometry and heading control (`robot.py`)
- Motion-profile position generation (`motion_profile.py`)
- Experimental pure-pursuit/path-following code (`purepursuit/`)
- IMU integration (`imu.py`)

### Perception & Localization

The team codebase also includes:

- OpenCV-based can and scoring-zone detection
- Camera calibration and HSV/color-threshold tooling
- Homography-based pixel-to-real-world coordinate conversion
- Camera/debug utilities for perception testing

My involvement in these perception components was smaller than my work on motion/control.

## Repository Structure

```text
.
├── robot.py                  # Robot movement, odometry, heading control
├── pid.py                    # PID controller utility
├── motion_profile.py         # Motion-profile position generation
├── imu.py                    # IMU interface
├── purepursuit/              # Experimental Bézier/pure-pursuit path following
├── state_machine.py          # Team autonomous behavior/state logic
├── camera.py                 # Camera integration
├── camera_utils.py           # Vision utilities
├── can_detector_api.py       # Can detection interface
├── detect_zones.py           # Zone detection
├── get_rw_coord.py           # Homography / coordinate transforms
├── jackstuff/                # Additional team-developed localization/control experiments
└── debug_imgs/               # Example perception debug images
```

## Technologies

- Python
- OpenCV
- NumPy
- Raven robot-control library / Raven board
- BNO08X IMU
- Encoder-based odometry
- PID control
- Homography
- Bézier curves / pure-pursuit-style path following

## Notes on the Code

This repository is a snapshot of a competition codebase developed under time constraints. It contains experimental files, alternate approaches, debug utilities, and partially integrated prototypes. Those artifacts are intentionally preserved because they reflect the team's engineering iteration during MASLAB.

Some experimental path-following code was not necessarily part of the final competition behavior. The repository is presented as a record of the team's development process rather than as a polished standalone software package.

## Results

- **4th overall** in the 2026 MASLAB competition
- Recipient of the **Wilkens Family Design Award**

## Usage / Rights

This repository is shared for **portfolio and code-review purposes**. No license is granted for reuse, modification, or redistribution. Because this was a collaborative team project, rights to individual portions of the code may also belong to their respective contributors.
