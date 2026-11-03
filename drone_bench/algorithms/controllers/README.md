# Controllers

- `paper_outer.py`: enhanced vector field, Eqs. 22–35.
- `potentials.py`: analytic attraction/repulsion functions and gradients.
- `paper_inner.py`: adaptive actor–critic, SPEM, LDO, saturation compensation.

Add each alternative controller as a separate module. Keep the environment, frozen
tube, initial states, dynamics and metrics shared. See simulation/interfaces.py.
