# Hardware

With the robots assembled, tested, and calibrated in the previous task, this branch advances to real-world object manipulation. Movement alone is no longer enough; the robots must now interact with their surroundings with purpose and precision.

In Task 4, the hardware system performs pick-and-place operations using physical robots. This marks a major milestone where navigation, coordination, and controlled manipulation come together in a practical setting.

This stage builds confidence in controlling robot motion, handling objects, and executing reliable actions in a physical environment.

> "A robot's true capability is revealed not when it moves, but when it handles the world around it."

## Objective

Task 4 is designed to build familiarity with hardware-based pick-and-place execution.

| Task | Focus | Marks |
| --- | --- | --- |
| Task 4A | Single-bot pick and place: one box is placed at a pickup zone, and the bot must pick it and place it at the designated drop zone. | 50 |
| Task 4B | Three-bot pick and place: three robots coordinate independently for similar pick-and-place operations, testing fleet coordination, collision-free movement, and stable object handling. | 50 |

## Branch Contents

- `Hardware/1308#0_HB_1308_Task_4a/` contains Task 4A ESP32, perception, controller, metadata, and result files.
- `Hardware/Multi_bot_controller.py/Multi_bot_controller/` contains multi-bot controller, perception, MQTT simulation controller, and packaging artifacts.
- `Hardware/Multi_bot_stacking_controller/` contains the multi-bot stacking controller, feedback script, final plot, summary, and encrypted results.

## Focus Areas

| Area | Description |
| --- | --- |
| Embedded control | ESP32 firmware for robot actuation and communication |
| Perception | Camera-based marker and arena state processing |
| Motion control | Holonomic-drive controller logic for robot movement |
| Object manipulation | Gripper, lift, pickup, transport, and drop behavior |
| Multi-bot execution | Coordination and stacking behavior across multiple robots |
| Validation | Output files, plots, summaries, and run artifacts |

## Important Instructions

This task involves object manipulation using real hardware. Ensure that the gripper mechanism, movement control, and arena layout are well tested before attempting the final recording. Handle all components carefully and avoid overloading motors or servos during lift operations.

- Read all provided documents carefully before posting queries.
- Research basic concepts before raising doubts.
- Check whether a similar query has already been answered.
- Use clear and precise language when asking questions, and mention the team ID.
- Attach proper screenshots instead of photos of screens.
- When reporting an error, include relevant logs, screenshots, and a clear description.

## Notes

This branch is intended for hardware execution and hardware-oriented validation. Keep simulation-only work on the `simulation` branch and final media deliverables on the `video-demonstration` branch.
