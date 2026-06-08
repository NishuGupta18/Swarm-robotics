# Hardware

This branch contains the hardware-side implementation for the Holo Battalion swarm robotics work.

The files here bring the simulated holonomic control pipeline onto physical robot hardware using ESP32 firmware, perception scripts, controller logic, and recorded run artifacts.

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
| Multi-bot execution | Coordination and stacking behavior across multiple robots |
| Validation | Output files, plots, summaries, and run artifacts |

## Notes

This branch is intended for hardware execution and hardware-oriented validation. Keep simulation-only work on the `simulation` branch and final media deliverables on the `video-demonstration` branch.
