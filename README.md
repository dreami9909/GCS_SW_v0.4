# Tactical Ground Control — Qt 3D PLAN + MISSION MAP milestone

This Windows Python UI is an independent, QGroundControl-inspired ground-control
prototype. The current milestone provides PLAN mission configuration and a
PySide6 MISSION MAP tactical display on one shared Google Maps 3D WebEngine surface.

The tactical controls are display-only training simulations. The project does
not transmit launch, targeting, fuze or other weapon commands.

## Included

- dark utilitarian toolbar with Data, Plan, Mission Map and Details views
- top-bar 전체/LM-01 through LM-06 fleet selection controls
- mock vehicle status, GPS, link, battery and telemetry
- animated vehicle marker and attitude indicator
- two synthetic TEL/tank ground-target tracks; until telemetry integration,
  double-clicking a threat is treated as manual target detection
- symbology for LM, GCS, radar, launcher and mobile ground targets
- safe zone, predicted-intercept marker and map grid scale
- readiness, mission-stage and automatic SAFE/ARM indicators
- nominal-coverage Google satellite seeker map with a black target block and
  phase-synchronised yellow/red tracking boxes
- Data sections for General, Maps, Video, Data Report, Launch Controller,
  Data Link, Seeker, Motor, Battery, AVS and Fuze
- Data Link channel display for Telemetry, ADT, GDT, FANET, GPS and Radar
- synthetic MAVLink-format receive log
- mission route with draggable waypoint markers
- Takeoff, Waypoint, Loiter, Return and Land mission tools
- mission item list and detail editor
- JSON plan open/save
- mock mission upload/download status
- mock action confirmation for ARM, Takeoff, Pause, RTL and Land

## Qt 3D PLAN + MISSION MAP

The Qt application is the active implementation. The older Tkinter UI remains
available as a reference while DATA is ported.

```powershell
cd <cloned qgc_python_ui folder>
python -m venv .venv_qt
.\.venv_qt\Scripts\python.exe -m pip install -r requirements.txt
.\.venv_qt\Scripts\python.exe main_qt.py
```

You can also double-click `run_qt_windows.bat` after the environment is ready.

Current PLAN functions:

- select GCS, radar or launcher placement mode
- click the 3D map to load a latitude/longitude/altitude position
- configure GCS, radar, launcher and safe zones once as shared fleet data
- unlock the top-bar vehicle controls after the shared setup is complete
- select LM-01 through LM-06 and edit an independent ordered waypoint route;
  전체 is the shared/fleet overview
- show shared elements plus only the currently selected vehicle route
- use the center of the shared safe zone as the emergency-return destination
- define polygonal safe zones with point-delete/finalize controls
- render the connected mission route, zones and draft vertices on the 3D map
- edit coordinates and altitude manually
- move the PLAN map camera to an entered latitude, longitude and altitude
- delete or clear loaded mission elements
- save/open version 3 fleet JSON while retaining version 1/2 single-route support
- keep opened or edited PLAN data as a draft until `임무 장입` is pressed
- load an independent mission snapshot into MISSION MAP with `임무 장입`
- require explicit PLAN waypoints for all six vehicles before mission loading;
  Mission Map never creates missing routes
- open legacy `site_config.json` files
- Python/JavaScript synchronization through Qt WebChannel

Current MISSION MAP functions:

- reuse the live PLAN map and mission data without creating a screenshot
- show all six LM vehicles together or inspect one selected vehicle
- highlight the selected LM in fluorescent green while keeping the others blue
- model the common-rally ingress performance as 100 km in 8 minutes
  (208.33 m/s), fixed-sector search at 100 km/h, guidance at 160 km/h and
  terminal acceleration to 200 km/h
- place the demonstration Russian/Iranian TEL and tank tracks about 32–40 km
  from the launcher loaded from PLAN
- retain a persistent CV/CTRV/random-motion IMM particle belief for every
  active ground target and show its moving 95% uncertainty area
- update the probabilistic relative-motion intercept every second after target
  detection, with a maximum T+8-minute horizon and per-LM reach probability
- show PLAN GCS, radar, launcher, waypoints and safe zones
- select a threat from the table or its map symbol
- show vehicle and designated-target altitude, coordinates, speed and heading
- derive AVS/LC/RDR/DL/GCS readiness indicators from local simulation and PLAN
- use one Launch action to eject LM-01~06, reach the common staging point and
  spread into six fixed 60-degree circular-arc search sectors
- update the RHP particle belief every simulated second and evaluate
  RHP-FE-PF-PW-ARC candidates only at each 25-second decision epoch; the map
  displays committed prefixes as LM-colored dotted paths and moving RHP
  waypoints without a large update counter overlay
- show one warning `탐지 확인` popup with the detecting LM number and start ATR
  automatically without a Yes/No engagement-approval step
- run ATR initial/midcourse guidance at 160 km/h and smoothly accelerate from
  160 to 200 km/h during terminal guidance, without an initial circular orbit
- retain the IMM-PF relative-intercept marker as a prediction reference while
  terminal guidance pursues the live moving target rather than forcing the
  target to that prediction
- create and display the relative-motion prediction when detection starts,
  then remove it when the engagement ends or is stopped
- keep Launch single-purpose and disabled after the initial launch
- make Emergency pause the mission and return all six vehicles directly to the
  safe-zone center
- draw each LM planned route as its own colored dashed line and its completed
  flight track as a solid line; dim non-selected LM routes and tracks in
  individual view. `전체` hides LM waypoint markers, while LM-01~06 shows the
  selected vehicle's applied RHP waypoints with update/revision coordinates
- stage Mission Map waypoint add/delete operations from the right-click menu
  until the operator presses `임무 수정`; both loaded PLAN waypoints and the
  currently displayed automatic RHP waypoints are editable. Applying a live
  edit changes only the selected LM, preserves every vehicle's position and
  execution state, and protects the manual route from RHP overwrite for 50
  simulated seconds before automatic planning resumes
- provide a six-way seeker popup; double-click a seeker to activate it and use
  the popup-header `격추`/`중지` MITL controls for that vehicle
- show a Google satellite seeker map at the nominal 1.2 km ground-strip scale;
  render the ground target as a black block, use a yellow ACQUIRE/MIDCOURSE box,
  a red TERMINAL box, grow the target to about 98% of the seeker width, and
  overlay a large red X plus `SHUT DOWN` while displaying horizontal, altitude
  and slant separation
- model seeker parameters as altitude 600 m, Af 18°, Ag 45°, coverage 1200 m,
  cell 850 m/43.2 s and search area 89.4 km²/44 min
- display each time-varying seeker footprint and the RHP-PF target-belief
  ellipse on both Google 3D and the portable offline map
- pause tactical updates outside Mission Map and use one Google map load across
  PLAN/MISSION MAP

The visual simulation runs the LM and ground tracks on a shared 3× time scale so fleet movement is
easy to follow. Distances, speed and ETA labels still use the configured real
performance values.
Load `demo_intercept_mission.json` with `임무 열기`, press `임무 장입`, and move to
MISSION MAP to exercise the 30–40 km ground-target pursuit scenario. Press
`발사`; ATR search, detection popup and six-LM cooperative guidance proceed
automatically. Double-clicking a threat remains an optional MITL designation.

For the Saudi-desert C4I scenario, open `saudi_desert_mission.json`. Version 4
mission files can include `initial_targets`, containing the initial C4I GPS
point, uncertainty, speed/course and a deterministic motion profile. PLAN shows
the initial points as `C4I GPS TARGET`; pressing `임무 장입` initializes each
target belief, while pressing `발사` starts both LM and target scenario time.
The common WP001 is the target-motion/ingress-ETA-predicted search center. All
six LMs first rendezvous globally at that TP. After physical TP arrival, LM-01
through LM-06 own fixed 60-degree local sectors; every ARC prefix and radial
transition remains inside its owner's sector until detection. Cross-sector
reassignment belongs to the separate Global-Dynamic comparison model and is
not used by this runtime. The live search planner is `RHP-FE-PF-PW-ARC`: its
candidate library starts at half
the 152.05 m track spacing and continues outward in concentric 152.05 m steps.
Every one-second GCS cycle updates the persistent PF. At each 25-second RHP
epoch, 64 common future target samples are used to evaluate the highest-
probability radii (plus the vehicle's current/committed radius when needed) in
both ARC directions. A team-level greedy marginal-union selector assigns all
six LMs together, reserves different ARC radii while alternatives exist, and
uses separated execution endpoints as a tie-breaker. This prevents the six
vehicles from collapsing onto one independently selected search path;
only the next 25-second prefix is committed, after which the horizon recedes and
the action is selected again. A straight transit prefix is represented by its
actual 1/3, 2/3 and endpoint execution waypoints; the selected prefix is shown
only in the individual LM view and moved waypoints pulse at each applied update.
At launch, the first RHP prefix is cached against the CPP experiment's exact
TP/t=0 state and held as a pending route. This computation does not enable a
seeker footprint, apply negative observations, advance the PF, or consume the
25-second RHP clock. Physical TP arrival commits that prefix without changing
position, sets search time to zero, and starts later decisions at the nominal
TP+25/50/75-second epochs. Mission-plan geometry is sent to the browser
separately from high-rate telemetry, so a coalesced position frame cannot
discard TP or route data during that handoff.
The mission JSON retains compact seed arcs only so PLAN can show
a pre-launch search concept; the live RHP route supersedes them after WP001. The
included TEL continuously patrols a 3.0 km clockwise orbit around TP at
12 km/h, visibly inside the yellow 3.33 km search boundary throughout the
demonstration. The tank decoy remains at 25 km/h. Both use the shared 3×
visual time scale. The demonstration requires three RHP
route revisions before enabling ATR detection, providing time to inspect the
dotted route and waypoint relocation directly on Mission Map.
ATR seeker detection is automatic; double-clicking remains an optional MITL
designation. At detection the detecting LM is reported and all six immediately
stop RHP search and turn toward the shared target track. They proceed through
ACQUIRE, TRACK and LOCK; terminal guidance descends continuously from 600 m to
the live ground target at 0 m instead of stopping at an artificial 30 m floor.

## Portable rule-based planning runtime

Running `main_qt.py` starts the planner as part of the same Python process; no
file or binary outside this folder is required. The implementation is under
`qt_gcs/planning/` and is reconstructed locally with Python and NumPy, so
cloning this folder from GitHub on another Windows PC does not require the
separate `E:\\GCS_SW\\CPP` research workspace.

The runtime order is:

1. derive one seeker geometry/scan-law configuration from the 600 m, 18° FOV
   and ±45° gimbal specification
2. initialize a 3,000-particle moving-ring prior with stratified sampling and
   advance the isotropic Markov target process on its fixed 30-second clock
3. apply actual flown search segments as negative-observation evidence and use
   stratified resampling below the 50% effective-particle threshold
4. draw 64 equal-weight common future samples for every candidate comparison
   and score probability-weighted concentric ARC actions by encounter rate
5. update the PF once per wall-clock second, then evaluate and commit only the
   next 25 seconds of the best action at each RHP epoch; FE work runs outside
   the Qt display thread
6. stop RHP search on positive detection and hand the shared track to the
   existing IMM-based relative-intercept and ATR guidance sequence

The 1,200 m seeker value is treated as the full centre-line ground sweep from
-45° to +45° at 600 m, not as a 1,200 m instantaneous detection radius. The
instantaneous 18° swath is about 190 m and the nominal 20%-overlap track spacing
is about 152 m. Automatic ATR detection integrates only the actual FOV looks
swept between 1 Hz planning samples, clips centre-line reach to 600 m on either
side, and remains deterministic at PD=1 inside that geometry. The 850 m/43.2 s
and 89.4 km²/44 min figures remain displayed
equipment metadata until a detailed equipment scan-law definition is supplied.
The Rule-based 2 benchmark condition is a 3.33 km search circle generated from
a 40 km/h maximum target speed and a five-minute TP lead time.

This remains a deterministic training simulation, not a validated flight,
targeting or safety-critical command implementation. The legacy CV Kalman
predictor stays inside `fly_state.py` only as a fallback if the runtime planner
cannot provide a fresh solution.

Current DETAILS functions:

- four-panel seeker, contact-LM, side-profile and equipment-status layout
- top-bar-selected vehicle artificial-horizon HUD with heading, roll/pitch,
  speed, altitude, ARM state, battery and GNSS status; 전체 displays LM-01
- toned-down HUD colors and an operator legend explaining HDG, SPD, ALT,
  ROLL, PITCH, BAT and GPS values
- seeker-screen MP4/H.264 recording at 10 FPS with start/stop and Save controls
- distance-versus-altitude side view for LM-01 and the selected ground target
- vertically stacked, wide launcher/radar control-and-status panels
- launcher azimuth/elevation, six read-only umbilical indicators and
  canister/vehicle state
- read-only launcher position loaded from the active PLAN mission snapshot
- four radar detections and no-fire/jamming/radar reference coordinates
- a separate LOG DATA window for launcher and radar status history
- responsive launcher/radar sizing with compact coordinate tables that remain
  fully visible at the 1280×760 minimum window size

`임무 열기` loads a JSON file into the PLAN editor only. Review or edit it,
then press `임무 장입` to replace the mission snapshot used by MISSION MAP.
`임무 저장` writes the current PLAN draft as a versioned JSON document.

## Google Maps 3D

The Qt view uses the Google Maps JavaScript API `Map3DElement` in HYBRID mode
when a key is available and falls back to a local perspective-grid preview when
it is not. Enable Maps JavaScript API and billing, restrict the key, then define
the key outside the source tree:

```powershell
$env:GOOGLE_MAPS_API_KEY = "your-restricted-key"
.\.venv_qt\Scripts\python.exe main_qt.py
```

For a persistent Windows user environment variable:

```powershell
setx GOOGLE_MAPS_API_KEY "your-restricted-key"
```

Open a new terminal after using `setx`. Never commit an API key to this project.

## Legacy Tkinter UI

```powershell
cd <qgc_python_ui folder>
python main.py
```

If `python` is not registered in PATH, run the project with the Python
executable installed on the computer.

You can also double-click `run_windows.bat`.

## Controls

- `F1`: Mission Map
- `F2`: Plan
- `F3`: Details
- `F4`: Data
- mouse wheel: map zoom
- right-button drag: map pan
- double-click a threat row or map threat symbol: select the displayed target
- Plan view left click: add selected mission command
- drag a waypoint marker: move the mission item

## Next milestone

Planned order after this UI milestone:

1. Data view revision
2. receive-only MAVLink transport and telemetry integration
