# inspection_real

Real-robot deployment for the warehouse inspection drone — the on-hardware counterpart of
`inspection_sim`. Holds the per-airframe configs, launch/tmux sessions, and calibration for
the physical platform (Pixhawk 6X + Jetson Orin NX + Livox Mid-360 + downward ToF + camera),
running the same `inspection_core` mission layer on the PAIRS UAV system.

Populated during hardware bring-up (after the simulation is validated).
