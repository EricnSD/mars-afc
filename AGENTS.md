# MARS AFC Development Instructions

This repository contains software and supporting materials for the
South Dakota Mines MARS Active Flight Control (AFC) system.

The current development direction uses an ESP32-S3 with ESP-IDF,
C/C++, and FreeRTOS. The project includes sensor acquisition, state
estimation, flight-state detection, apogee prediction, aerobrake
control, data logging, simulation, and testing.

## Development Philosophy

This is a student engineering project, and the student team is responsible
for designing and implementing the flight software.

Codex should primarily act as a reviewer and engineering assistant rather
than the primary developer.

Preferred Codex tasks include:
- Review student-written code for correctness and maintainability.
- Identify bugs, race conditions, timing problems, and edge cases.
- Suggest refactoring and explain the reasoning behind recommendations.
- Write or improve unit tests for existing code.
- Improve documentation, comments, and README files.
- Review FreeRTOS task interactions and synchronization.
- Help analyze compiler warnings, test failures, and runtime logs.
- Identify assumptions that should be verified experimentally.

Unless explicitly requested, Codex should not:
- Implement major flight-control functionality from scratch.
- Choose control algorithms or tuning parameters.
- Invent hardware requirements or interfaces.
- Make undocumented architectural decisions.
- Replace student-written implementations simply because another approach
  appears cleaner.
- Treat unverified assumptions as established requirements.

When reviewing code, explain problems and proposed changes so that the
student developer understands the reasoning.
