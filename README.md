# Swarm-robotics

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314e?style=flat-square)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-e95420?style=flat-square)](https://releases.ubuntu.com/22.04/)
[![Platform](https://img.shields.io/badge/Platform-Holonomic%20Swarm-2f855a?style=flat-square)](#project-overview)
[![Docs](https://img.shields.io/badge/Docs-mdBook-6f42c1?style=flat-square)](#documentation)

Vision-based swarm robotics system for the e-Yantra Holo Battalion theme. The project coordinates multiple holonomic robots using overhead camera localization, ArUco marker detection, color-based crate classification, task allocation, and autonomous stacking in constrained drop zones.

## Project Overview

Holo Battalion models an agricultural cold-storage warehouse where three holonomic robots sort perishable produce crates before a global timer expires. Each crate is identified using ArUco markers and routed to a matching drop zone based on its color class.

The system focuses on:

- Multi-robot coordination for three holonomic-drive bots: Glacio, Crystal, and Frostbite
- Overhead-camera localization using ArUco markers
- Color-coded crate sorting for red bell peppers, limes, and blueberries
- Centralized planning, robot control, and task allocation
- Efficient placement and stacking inside constrained cold-storage zones
- Hardware execution using ESP32-based robot controllers

## Repository Structure

The repository is organized using separate branches so each deliverable stays clean and reviewable.

| Branch | Purpose | Contents |
| --- | --- | --- |
| `main` | Project landing page and documentation index | README, mdBook scaffold |
| `simulation` | Simulation tasks | Single-bot and multi-bot simulation controllers, metadata, run databases |
| `hardware` | Hardware tasks | ESP32 firmware, perception scripts, controllers, stacking results |
| `perception-control` | Camera and low-level control work | Camera calibration, camera tests, low-level controller structure |
| `video-demonstration` | Final media deliverables | Demonstration videos, submission links, presentation assets |

## System Architecture

```text
Overhead Camera
      |
      v
ArUco Detection and Pose Estimation
      |
      v
Global State: robots, crates, zones
      |
      v
Task Allocation and Motion Strategy
      |
      v
Robot Controllers
      |
      v
ESP32 Holonomic Drive Hardware
```

## Theme Constraints

| Area | Specification |
| --- | --- |
| Arena | 8 ft x 8 ft cold-storage warehouse layout |
| Robots | 3 holonomic-drive HoloBots |
| Localization | Overhead camera with ArUco markers |
| Robot ArUco IDs | `0`, `2`, `4` |
| Crate classification | `ArUcoID % 3` maps crates to red, green, or blue |
| Time limit | 250 seconds per run |
| Software stack | Ubuntu 22.04, ROS 2 Humble, Python/C++ |
| Hardware controller | ESP32 WROOM kit |

## Documentation

This repository includes an mdBook-ready documentation scaffold under [`docs/src`](docs/src/SUMMARY.md). The book outline is designed to grow with the project:

```text
Swarm Robotics Documentation
├── Introduction
├── Theme and Rules
├── System Architecture
├── Simulation
├── Hardware
├── Perception and Control
└── Demonstration
```

To preview the book locally after installing mdBook:

```bash
mdbook serve docs
```

## Branch Workflow

Use branches as independent deliverable tracks:

```bash
git switch simulation
git switch hardware
git switch perception-control
git switch video-demonstration
```

Keep `main` focused on documentation and project-level presentation. Merge only when you intentionally want the branch contents to become part of the main project history.

## Team Notes

This work was developed for the Holo Battalion theme, combining simulation, perception, embedded control, and real-world multi-robot coordination into a single swarm robotics pipeline.
