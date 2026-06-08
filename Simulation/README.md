# Simulation

Deep inside the cold-storage warehouse, Glacio, Crystal, and Frostbite have completed their initial calibrations. Having mastered single-bot maneuvers in the simulation grid, it is now time to put their skills to the test. Crates must be picked, placed, and transported, but success requires more than individual precision.

This is Task 2, where teamwork, timing, and coordination take center stage. The simulation work commands multiple robots through ROS 2, executes pick-and-place operations, and synchronizes holonomic-drive robots in the Holo Battalion environment. Each sub-task builds toward smooth multi-bot operation under warehouse-like constraints.

> "A fleet of robots moves as one when coordination is perfect, and a swarm succeeds when every unit knows its place."

## Objective

Task 2 is designed to build familiarity with:

| Task | Focus |
| --- | --- |
| Task 2A | Manipulator control for pick-and-place operation with a single bot |
| Task 2B | Pick-and-place operation using multiple robots while demonstrating the 3C's of swarm robotics: communication, cooperation, and coordination |

## Branch Contents

This branch contains the simulation submissions and recorded run artifacts for the Holo Battalion swarm robotics work.

- `Single_bot_controller.py/` contains the Task 2A single-bot holonomic controller, metadata, and run database.
- `Multi_bot _controller.py/` contains the Task 2B multi-bot holonomic controller, metadata, and run database.
- Recorded `.db3` artifacts capture simulation runs for review and validation.
