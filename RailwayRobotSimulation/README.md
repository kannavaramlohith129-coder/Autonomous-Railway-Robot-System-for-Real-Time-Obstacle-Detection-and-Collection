# Autonomous Railway Track Inspection and Obstacle Removal Robot
### 3D MATLAB Simulation (B.Tech Project) — pure MATLAB, no Simulink/Python/Unity

---

## 1. Folder Structure

```
RailwayRobotSimulation/
│
├── main.m                          <- RUN THIS FILE
├── createConfig.m                  <- all tunable parameters
│
├── Environment/
│   ├── createRailway.m             (rails, sleepers, ballast)
│   ├── createGround.m              (textured terrain plane)
│   ├── createRailwayObjects.m      (layered trees w/ shadows, poles, fences)
│   ├── createSky.m                 (distant hills + clouds, for depth)
│   └── DayNightController.m        (day/night lighting)
│
├── Robot/
│   ├── createRobot.m               (body, wheels, scanner, lights)
│   ├── updateRobotTransform.m      (applies position/heading)
│   ├── moveRobot.m                 (smooth accel/decel + movement)
│   ├── RobotController.m           (per-frame orchestrator + arm sequence)
│   └── RobotStateMachine.m         (pure state-transition logic)
│
├── Detection/
│   ├── detectObstacle.m            (range/FOV/corridor test)
│   ├── calculateObstacleDistance.m
│   └── ObstacleScanner.m           (fan of sensor rays + highlighted hit ray)
│
├── Obstacles/
│   ├── createObstacle.m
│   ├── spawnObstacles.m            (builds obstacles for a scenario)
│   ├── classifyObstacle.m          (rule-based REMOVABLE/HEAVY)
│   ├── setObstaclePosition.m       (reposition graphics - used to follow the gripper)
│   └── removeObstacle.m            (finalizes the drop, clear of the corridor)
│
├── Manipulator/
│   ├── createManipulator.m         (builds the arm's hgtransform chain)
│   ├── applyArmTransforms.m        (pushes joint angles into graphics)
│   ├── getGripperWorldPosition.m   (forward kinematics - world XYZ of gripper tip)
│   ├── controlArm.m                (smooth joint/gripper interpolation)
│   ├── moveArmToObstacle.m         (reach-and-lower target pose)
│   ├── graspObstacle.m             (closed-gripper target)
│   └── releaseObstacle.m           (open-gripper target)
│
├── Train/
│   ├── createTrain.m
│   ├── updateTrainTransform.m
│   ├── moveTrain.m
│   └── trainSafetyManager.m
│
├── Safety/
│   ├── safetyDecision.m            (CONTINUE/STOP/REMOVE/.../EMERGENCY_STOP)
│   ├── calculateTimeToCollision.m
│   └── trackClearanceCheck.m
│
├── UI/
│   ├── createDashboard.m           (uifigure dashboard)
│   ├── updateDashboard.m
│   └── eventLogger.m               (in-memory log + event feed)
│
└── Data/
    └── simulationLog.csv           <- created automatically when you run it
```

Save the whole `RailwayRobotSimulation/` folder as-is — the subfolder names matter
because `main.m` calls `addpath(genpath(...))` on startup, which adds every
subfolder to the MATLAB path automatically. You do not need to manually add
paths.

---

## 2. How to Run

1. Copy the entire `RailwayRobotSimulation` folder onto disk.
2. In MATLAB, set the **Current Folder** to `RailwayRobotSimulation/`.
3. Type:
   ```matlab
   main
   ```
4. A menu window pops up — pick one of six scenarios (see §7).
5. Two windows open:
   - **3D Simulation** — the railway, robot, train, obstacles.
   - **Dashboard** — live status panel (robot state, obstacle info, train
     distance, decision, safety status, event feed).
6. Close the 3D simulation window at any time to stop the simulation early.
   When it ends (or is closed), `Data/simulationLog.csv` is written
   automatically with every logged event.

### Required MATLAB version / toolboxes
- MATLAB **R2020a or later** is recommended (uses `uifigure`/`uilabel`/
  `uitextarea` for the dashboard, and string arrays for state names).
- **No additional toolboxes are required** — no Simulink, no Robotics
  System Toolbox, no Computer Vision Toolbox. Only base MATLAB graphics
  (`patch`, `surface`, `hgtransform`, `light`) and base MATLAB App Designer
  UI components are used.

---

## 3. Simulation Workflow

Each simulation frame (default `dt = 0.05 s`, set in `createConfig.m`):

1. **Sense** — `ObstacleScanner.m` draws the forward scan cone and checks
   every obstacle with `detectObstacle.m` / `calculateObstacleDistance.m`,
   returning the nearest detected obstacle (if any).
2. **Train safety check** — `trainSafetyManager.m` computes the train's
   distance/time to the robot and returns a `SAFE` / `WARNING` /
   `EMERGENCY` level.
3. **Think / Act** — `RobotController.m` runs the current state's actions
   (set target speed, drive the arm sequence, remove obstacles, etc.),
   then calls `RobotStateMachine.m` to get the next state:

   ```
   MOVING → OBSTACLE_DETECTED → CLASSIFYING → (REMOVABLE) → APPROACHING
          → STOPPED → REMOVING → CLEARANCE_CHECK → RESUMING → MOVING

   MOVING → OBSTACLE_DETECTED → CLASSIFYING → (HEAVY) → WARNING → WAITING

   (any state) → EMERGENCY_STOP  [forced whenever the train safety level
                                   becomes EMERGENCY]
   ```

4. **Move** — `moveRobot.m` and `moveTrain.m` advance positions; speed
   ramps smoothly using `config.robotAcceleration` rather than jumping.
5. **Arm sequence** (inside `RobotController.m`, while state = `REMOVING`):
   `REACH → GRASP → LIFT → SIDEWAYS → RELEASE → RETURN` — driven by
   `controlArm.m`, `moveArmToObstacle.m`, `graspObstacle.m`,
   `releaseObstacle.m`. From the moment the gripper closes (`GRASP`)
   until it fully opens again (`RELEASE`), the obstacle is re-parented
   to the gripper's actual world position every frame via
   `getGripperWorldPosition.m` (real forward kinematics through the
   robot → base → shoulder → elbow → wrist chain) and
   `setObstaclePosition.m`. That's what makes the pick-up read clearly:
   the rock visibly lifts, swings sideways, and is set down with the
   arm — it never just teleports. `removeObstacle.m` then finalizes the
   drop, snapping it onto the ground and guaranteeing it lands clear of
   the safety corridor.

   > **Approximation note:** the arm uses a fixed set of reach/lift/
   > sideways waypoint angles rather than a full inverse-kinematics
   > solver, since the obstacle is always roughly the same distance in
   > front of the robot whenever the arm deploys. This is clearly marked
   > in `moveArmToObstacle.m` and is easy to replace with real IK later
   > (the link lengths `arm.upperLen` / `arm.foreLen` are already stored
   > for that purpose).

6. **Camera & lighting** — the camera chases the robot in third-person
   view; the light is repositioned each frame so the scene stays lit.
7. **Dashboard & logging** — `updateDashboard.m` refreshes the panel;
   `eventLogger.m` appends a row whenever something notable happens
   (obstacle detected, arm activated, obstacle removed, state changes,
   train warnings, etc.).

---

## 4. Robot States

`MOVING, OBSTACLE_DETECTED, CLASSIFYING, APPROACHING, STOPPED, REMOVING,
CLEARANCE_CHECK, WARNING, WAITING, RESUMING, EMERGENCY_STOP, COMPLETED`

The current state is always shown above the robot in the 3D view and in
the dashboard's "Current State" field. The warning light on top of the
robot also changes color: green (moving), yellow (obstacle
detected/classifying/approaching), orange (stopped/removing/clearance
check), red (warning/waiting/emergency).

---

## 5. Changing Parameters

Everything you're likely to want to tune lives in **`createConfig.m`**:

```matlab
config.robotSpeed             = 2.0;   % m/s
config.scannerRange           = 30;    % m
config.scannerFOV             = 25;    % degrees
config.removableMassThreshold = 20;    % kg
config.trainSpeed             = 15;    % m/s
config.trainStartDistance     = 140;   % m
config.trackLength            = 220;   % m
config.numberOfObstacles      = 3;     % used in the "multiple" scenario
config.dayNightMode           = "DAY"; % "DAY" or "NIGHT"
config.simulationSpeed        = 1.0;   % time multiplier
```

---

## 6. Machine-Learning Extension Path

The project runs entirely without ML. `Obstacles/classifyObstacle.m` is
intentionally isolated and rule-based (mass threshold). To add ML later,
create `classifyObstacleML.m` with the same call signature
(`category = classifyObstacleML(obstacle, config)`, or extended with a
trained model / camera image input) and swap the one call to it inside
`Obstacles/createObstacle.m`. No other file needs to change.

---

## 7. Test Scenarios

Selected from the startup menu:

| # | Scenario | Expected behaviour |
|---|----------|---------------------|
| 1 | No obstacles | Robot moves continuously to the end of the track. |
| 2 | Single removable obstacle | Detect → slow → stop → arm reaches, grasps, lifts, moves it aside, releases, returns home → clearance check → resume. |
| 3 | Single heavy obstacle | Detect → classify HEAVY → stop → warning issued → robot remains stopped (arm never attempts removal). |
| 4 | Multiple obstacles | Each obstacle processed in sequence as the robot reaches it. |
| 5 | Heavy obstacle + approaching train | Heavy obstacle stops the robot; if the train closes to within `config.trainDangerTime` seconds, `EMERGENCY_STOP` fires and the train itself brakes. |
| 6 | Night operation | Night sky/lighting, robot headlights turn on automatically. |

---

## 8. Troubleshooting

- **"Undefined function" errors on startup** — make sure your Current
  Folder is `RailwayRobotSimulation/` itself (not a parent or a
  subfolder) before running `main`. `main.m` adds all subfolders to the
  path automatically via `genpath`, but only relative to its own location.
- **Dashboard fields don't update / errors about `uifigure`** — you need
  MATLAB R2020a+ for full `uitextarea`/`scroll` support. On older
  versions, remove the `scroll(dash.eventLog,'bottom')` line in
  `UI/eventLogger.m` (cosmetic only).
- **Simulation runs slowly** — increase `config.dt` slightly (e.g. 0.08)
  or reduce `config.numTrees`/`config.numPoles` in `createConfig.m`.
- **Robot appears to stop responding after an obstacle is removed** —
  check that `Data/simulationLog.csv` shows a `CLEARANCE_CHECK ->
  RESUMING` transition; if the obstacle wasn't actually pushed clear of
  `config.corridorWidth`, `trackClearanceCheck.m` will correctly hold the
  robot until it is.
- **Closing the 3D window early** — this is fine; the loop detects the
  closed figure, exits gracefully, and still writes the CSV log for
  whatever happened up to that point.
- **`menu` window doesn't appear / MATLAB is running headless** — `menu`
  requires a graphical MATLAB session (not `-nodisplay`/`-nodesktop`
  batch mode). Run MATLAB normally with its desktop UI.
