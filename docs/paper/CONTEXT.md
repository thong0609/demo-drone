# Curve virtual tube — paper context

Source: `/home/tts/Downloads/1. Curve_Virtual_Tube_Planning_and_Distributed_Control_for_Swarm_UAVs_With_Complex_Constraints_and_Disturbances (1).pdf`

Mengshun Yuan, Mou Chen, Mihai Lungu, Tongle Zhou. *Curve Virtual Tube Planning and Distributed Control for Swarm UAVs With Complex Constraints and Disturbances*. IEEE Transactions on Industrial Electronics, vol. 73, no. 2, February 2026, pp. 3037–3048. DOI: 10.1109/TIE.2025.3603043.

## Objective and model

Guide a UAV swarm through a common 3D tube from a start region to a goal region, with collision avoidance, boundary clearance, cohesion, speed limits, input saturation, uncertainty and disturbances. Attitude stabilization is assumed; the paper models translational dynamics: p_dot = v, v_dot = f + g*u + Delta + d (Eq. 1).

## Planning (Section III, Algorithms 1–3)

1. Compute an SDF map for nearest-obstacle distance and gradient queries.
2. Enhanced RRT*: edge cost combines Euclidean length and obstacle proximity penalties at interpolated edge samples (Eqs. 15–16). The penalty is (re-d)^3/(d^3+kappa) for 0 <= d < re, otherwise zero. Visual() searches ancestors for a collision-free, lower-cost parent.
3. Generate piecewise polynomial centerline pb(t) and radius rb(t) using minimum-snap QP (Eqs. 17–21). Objective includes squared fourth derivative and weighted deviation from the benchmark path. Constraints cover endpoint position/velocity/acceleration, continuity through acceleration, and corridor bounds.
4. Tube = union of circular disks normal to the centerline tangent (Eqs. 10–13).

## Distributed outer loop (Section IV-A)

Each UAV gets its own guidance curve rather than tracking the centerline. Eq. 22 uses p_guidance(t) = pb(t) + rb(t)*(p_i-pb(tq))/rb(tq). The offset vector in that equation is expressed in global coordinates; do not silently replace it with a transported local frame when reproducing the paper.

Eq. 34 combines guidance along its curve tangent, inter-UAV repulsion, tube-boundary repulsion, and swarm cohesion, with norm saturation at vm. Eqs. 25–35 define potentials and their derivatives. Safety target: pairwise distance >= 2*rs; boundary clearance >= rs; pairwise swarm distance <= lm. Avoidance acts over rs+ra; cohesion activates beyond ls. Eq. 40 establishes nonincreasing outer Lyapunov function in the virtual-velocity analysis.

## Adaptive inner loop (Section IV-B)

First-order command filter (41); compensated tracking error e = v-zeta-Lambda (42). Actor NN estimates lumped uncertainty; critic guides learning (45–56). Serial-parallel estimation model (SPEM, 49–51) supplies prediction error. Learning-based disturbance observer (LDO, 57) estimates composite disturbance. Auxiliary system (60) compensates input saturation, using smooth tanh approximation (5). Inner control law is Eq. 61; adaptive robust gain update follows Eq. 66. The inner Lyapunov bound is V_dot <= -P*V+G (76), i.e. practical boundedness rather than an unconditional zero-error claim.

## Reproduction parameters (Section V)

- 2D comparison: 30 UAVs; vm=3 m/s; rs=0.4 m; ra=0.8 m; ls=15 m; lm=22 m. P=diag(7.2,7.2); C=diag(1.8,1.8); Gamma_a=Gamma_c=0.22I; eta1=eta2=1.1; K=diag(1,1).
- 3D: map 500x500x300 m; start (50,140,50); goal (460,390,170); max tube radius re=45 m; 27 UAVs; vm=11 m/s; rs=5 m; ra=10 m; ls=100 m; lm=150 m. P=diag(9.2,9.2,10.2); C=diag(1.9,1.9,2); Gamma_a=Gamma_c=0.3I; eta1=eta2=1.1; K=diag(1.4,1.4,1.5). Reported centerline length 522.13 m, tube cost 653.02.
- Real experiment: indoor 5x5x3 m; start (0.5,0.9,1); goal (0.5,4.2,2); re=0.8 m; vm=0.37 m/s; rs=0.3 m; ra=0.9 m; ls=2.5 m; lm=3.5 m. MATLAB/Ubuntu control workstation, motion capture and WiFi commands.

## Implementation checks

Track minimum pairwise distance, minimum boundary clearance, maximum pairwise distance, speed, input magnitude, completion, and inner tracking error. Check smoothed tube clearance against obstacles explicitly; minimum snap alone does not certify it. Check geometric validity and cross-section association on tight curves. Verify PDF equations visually before coding: text extraction loses symbols, signs and matrix formatting. Do not treat virtual-command bounds as proof that actual velocities meet hard limits under tracking errors.

Extracted full text is currently available at `/tmp/curve_tube.txt`; the PDF is the source of truth.
