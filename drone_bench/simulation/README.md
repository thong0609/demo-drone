# Simulation

- `initialization.py`: deterministic validated 27-agent start layout.
- `dynamics.py`: translational plant and declared perturbation profiles.
- `runner.py`: common integration loop, terminal exits and recording.
- `metrics.py`: constraint measurements and summary.
- `interfaces.py`: structural contracts for interchangeable controllers.

The runner receives outer/inner controllers as arguments; it does not import a
particular controller. It never clips actual velocities or moves agents back
inside the tube to conceal violations.
