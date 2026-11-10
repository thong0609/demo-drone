# Experiment configurations

`paper_vb.json` contains the complete Section V-B baseline setup: shared scenario,
planner settings, paper gains, assumed physical/NN parameters, seed and integration
settings. Its provenance block distinguishes reported parameters from assumptions.
Keep separate config files for new algorithms/variants and separate output folders.

The current `paper_vb.json` uses continuous polynomial geometry, bounded NN
features with a checked Eq. 77 witness, and a velocity-only assumed uncertainty
model. These unspecified implementation choices are recorded in provenance;
they are not additional parameters reported by the authors. See
`docs/PAPER_FORMULA_FIXES.md`. `paper_vb_legacy.json` preserves the earlier config
used by the historical DMPC comparison. Historical outputs are not overwritten
by the new default output folder, `artifacts/paper_vb_continuous`.
