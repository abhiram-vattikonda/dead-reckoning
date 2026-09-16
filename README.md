# SIH — Smartphone Intelligent Dead Reckoning (prototype v0.1.0)

Milestone 1: compare a baseline inertial dead-reckoning trajectory against GNSS
ground truth on a real Android phone, side-by-side on an OpenStreetMap map.

## 1. Maps: OpenStreetMap (no API key needed)

The map uses **osmdroid** (OpenStreetMap MAPNIK tiles). There is nothing to
configure and no key to obtain — tiles load over the network on first view and
are then cached in app-private storage. (For hackathon-scale testing this is
fine; for heavy use, respect the OSM tile usage policy.)

## 2. Build (terminal)

```bash
./gradlew assembleDebug
# APK: app/build/outputs/apk/debug/app-debug.apk
```

Requires JDK 17, Android SDK with platform android-37
(`sdk.dir` is set in `local.properties`).

## 3. Install on the physical phone

```bash
adb devices            # phone must show as "device" (USB debugging ON)
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

Grant location permission on first launch (precise location).

## 4. Run an experiment

1. Go outdoors, open the app, wait for a GPS fix (blue marker appears).
2. Fix the phone in a mount — do NOT move it mid-run (v1 assumes fixed mount).
3. Press **START** (DR initializes from the GPS fix, then runs on IMU alone).
4. Walk/drive. Blue = GPS ground truth, Red = DR estimate.
5. Toggle **GNSS Available To Navigation Engine OFF** to simulate a blackout
   (with Testing Mode ON, GPS keeps logging independently as reference).
6. Press **STOP** → final error / mean / max / drift stats.
7. Press **EXPORT CSV** → files land in
   `<storage>/Android/data/com.sih.idr/files/exports/`:
   - `session_<ts>_nav.csv` (GPS + matched DR + errors, for Jupyter)
   - `session_<ts>_imu.csv` (raw accel/gyro/mag/quat + linear-accel + gravity)
   - `session_<ts>_pdr.csv` (every step + 10 Hz: acc magnitude, step flag,
     stride, gyro/mag/fused headings, local x/y, lat/lon)

## 5. How the DR works: PDR BASELINE (not the final vehicle model)

`GitHubStylePdrEngine` (`navigation/`) is a pedestrian-style baseline ported
from nisargnp/DeadReckoning — step detection + heading fusion, NOT MEMS double
integration:

1. Linear-accel magnitude → `DynamicStepCounter` (moving-average hysteresis
   peak detector, sensitivity 1.0) → step events.
2. Gyro − bias → 0.0025 high-pass → ×dt → DCM orientation update → gyro heading,
   anchored once to the initial magnetic heading.
3. Gravity + magnetometer → absolute magnetic heading.
4. Complementary fusion: h = 0.02·mag + 0.98·gyro (wrap-aware).
5. Per step: dx = stride·cos(h), dy = stride·sin(h) with default stride 0.75 m,
   mapped x→East / y→North onto lat/lon from the seed fix.

All tunables live in `navigation/pdr/PdrConfig.kt` (stride, sensitivities,
biases, fusion weights, declination). GPS seeds the start position only — after
that the engine is IMU-only (`onGnssMeasurement` is a no-op).
The old double-integration `BaselineDeadReckoningEngine.kt` is retained but inactive.

## 6. Where the MMM/AI engine plugs in later

Implement `MMMDeadReckoningEngine : DeadReckoningEngine`
(`navigation/DeadReckoningEngine.kt`) and swap the **single construction site**
marked `>>> SINGLE SWAP SITE FOR ENGINES <<<` in `ui/TrackingViewModel.kt`.
UI, sensors, map, evaluation, CSV export all keep working unchanged.
Replaceable sub-modules for staged upgrades: `DynamicStepCounter`,
`StrideLengthProvider` (fixed → learned), `GyroscopeOrientationDcm`,
`MagnetometerHeadingProvider`, `ComplementaryHeadingFilter`.

## 7. Project structure

```
app/src/main/java/com/sih/idr/
  data/         Models.kt (ImuSample, GpsSample, NavigationState, TestSession…)
                CsvExporter.kt
  sensors/      ImuManager.kt (SensorManager → ImuSample, imu-thread)
                GpsManager.kt (FusedLocation → GpsSample, independent path)
                OrientationManager.kt (rotation-vector → quaternion/yaw)
  navigation/   DeadReckoningEngine.kt (replaceable interface + GNSS hook)
                BaselineDeadReckoningEngine.kt (simple strapdown baseline)
                CoordinateTransformer.kt (+ VehicleAlignmentModule)
  evaluation/   EvaluationManager.kt (haversine errors, drift %)
  utils/        GeoUtils.kt (haversine, ENU→lat/lon, drift formula)
  ui/           MainActivity.kt (permissions)
                MainScreen.kt (map + stats + controls)
                MapComponent.kt (GPS-blue / DR-red polylines, follow mode)
                TrackingViewModel.kt (wiring, session lifecycle, 5 Hz UI)
```

## 8. Automatic off-route recalculation

When a destination is set and **Auto-recalculate route** is ON (default), the
app watches the vehicle pose and rebuilds the route after a wrong turn:

- Pose source: fresh GPS fix if one exists (< 8 s old), otherwise the DR
  (inertial) estimate — detection and recalculation work with phone location
  OFF and with no WiFi/cellular data.
- Turn confirmation uses IMU-derived heading, not GPS bearing: > 35 m off the
  planned polyline sustained over ~4 checks (~8 s), or a > 30° heading change
  plus > 21 m deviation (fast path). Stationary (< 1 m/s) never triggers;
  20 s cooldown prevents thrash.
- Recalculation is offline-first (bundled Guntur/Vijayawada road graphs);
  OSRM online is tried only when a network is actually available. The green
  route, the fusion engine constraint, and the route cache all update together.
- Recalculated routes start from the road AHEAD, never with a U-turn
  instruction: the offline graph snaps to the directed edge matching the
  vehicle's compass heading (`OfflineRoadGraph.routeWithHeading`), and OSRM
  gets the same heading as a `bearings` hint. The only exception is a genuine
  dead-end, where doubling back is the only way out.
- Logic lives in `navigation/OffRouteDetector.kt` (pure Kotlin, no Android
  dependencies); tunables in `OffRouteConfig`.

## 9. Known limitations (v1)

- Baseline DR drifts fast (tens of meters/minute typical) — by design.
- Phone must stay fixed relative to the vehicle during a run.
- World frame uses the rotation-vector sensor (magnetic north on most phones).
- Session data is in-memory only; export CSV before RESET/restart.
- A location-type foreground service (`RecordingService`, started on START,
  stopped on STOP) keeps IMU/GPS delivery alive with the screen off or the app
  backgrounded. Sensors never depend on the phone location switch: turning
  location off pauses only the blue trail.
- The red DR trail is one dot per 10 Hz sample (`DotTrailOverlay`), not a
  line — gaps mean dropouts, clusters mean standstill. The Session info panel
  shows live `IMU … @ Hz` rate plus a `DR states` counter: if IMU samples grow
  but DR states don't, the engine (not the sensors) is stalled.
- Offline road routing is supported by placing `road_graph.json` in
  `app/src/main/assets/`; without that asset the destination route is a
  straight-line fallback. The graph is generated from an offline OSM XML
  extract by the companion `osm_to_roadgraph.py` script.
