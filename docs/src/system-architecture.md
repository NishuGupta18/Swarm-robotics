# System Architecture

The system pipeline connects perception, planning, and control:

1. Capture the arena through an overhead camera.
2. Detect ArUco markers for robots and crates.
3. Estimate global positions and orientations.
4. Allocate tasks across robots.
5. Generate robot motion commands.
6. Execute movement through the low-level controller.
