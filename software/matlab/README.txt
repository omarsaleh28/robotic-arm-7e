AL5D KINEMATICS - Robotic Arm 7E (Golf Ball Teeing Robot)
=========================================================

HOW TO RUN
1. Put all the .m files in one folder and open that folder in MATLAB.
2. Open al5d_main.m and press Run (F5).
   No toolbox is needed. If Peter Corke's Robotics Toolbox is on the path,
   an extra cross-check runs at the end.

FILES
al5d_params.m          ALL numbers (lengths, limits, calibration, masses). Edit only this file.
al5d_main.m            Runs all checks, prints results and draws the workspace figure.
al5d_fk.m              Forward kinematics (geometry).
al5d_fk_dh.m           Forward kinematics with Denavit-Hartenberg transforms (as taught).
al5d_ik.m              Closed-form inverse kinematics (elbow-up) + joint-limit check.
al5d_angles_to_ssc32.m Joint angles -> servo pulses -> SSC-32U command string.
al5d_static_torque.m   Gravity holding torque at shoulder/elbow/wrist (kg*cm).

WHAT TO UPDATE (in al5d_params.m), in this order
1. Link lengths: measure the real arm (d1, L2, L3, Lg). Values now are published nominal ones.
2. Joint limits and servo calibration: measure with the SSC-32U (pulse at two known angles).
   THE CALIBRATION IS PLACEHOLDER - do not send the generated commands to the arm until it is done.
3. Gripper length Lg and the pick pitch phi_pick once the gripper is chosen.
4. Extension length (ext_fore or ext_upper) once the mech team designs it.
5. Masses: weigh the parts.

RESULTS (CAD-measured lengths, updated 1 Nov 2026)
Geometry from the Lynxmotion AL5D CAD (AL5D_2017_12_17.STEP), joint-axis centre to centre:
  d1 = 71.2 mm, shoulder offset a1 = 11.9 mm in front of the base axis,
  L2 = 146.3 mm, L3 = 180.7 mm, Lg = 86 mm (claw position not fixed yet; stock
  fingertips are 93.8 mm from the wrist-pitch axis).
- FK geometry vs DH agree (difference ~1e-13 mm)            PASS
- IK round trip, 1000 targets (error ~1e-13)                PASS
- Max pick radius: ~334 mm from above, ~400 mm at -45 deg, ~418 mm from the side
- Forearm extension needed for 520 mm (elbow kept bent >= 20 deg):
  ~189 mm from above, ~126 mm at -45 deg, ~104 mm from the side
- Torque with a 189 mm extension holding a ball at 520 mm:
  shoulder ~51%, elbow ~55% of stall -> just above the 50% rule

ANGLE CONVENTION
th1 base yaw (0 = +x); shoulder axis is offset a1 forward of the base axis; th2 shoulder above horizontal; th3 elbow relative to upper arm
(negative = bends down); th4 wrist relative to forearm. Gripper pitch = th2+th3+th4.

TESTING ON THE REAL ARM
al5d_test_on_arm.m sends test points to the arm through the SSC-32U (USB serial,
9600 baud), asks you for the measured gripper position, and saves the errors to
arm_test_log.csv. Set the COM port at the top first. Calibrate before running.
