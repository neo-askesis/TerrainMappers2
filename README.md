# PocketPath — TerrainMappers

[![Build & Release APK](https://github.com/neo-askesis/TerrainMappers2/actions/workflows/build-release.yml/badge.svg)](https://github.com/neo-askesis/TerrainMappers2/actions/workflows/build-release.yml)
[![Platform](https://img.shields.io/badge/platform-Android-3DDC84?logo=android&logoColor=white)](#)
[![Min SDK](https://img.shields.io/badge/minSdk-26-informational)](#)
[![Kotlin](https://img.shields.io/badge/language-Kotlin-7F52FF?logo=kotlin&logoColor=white)](#)

**PocketPath** reconstructs the path you walked using only your phone's motion
sensors — no GPS, no network, and the phone can stay in your pocket the whole time.

It records the gyroscope, gravity, accelerometer, and magnetometer, then runs those
readings through a dead-reckoning algorithm that detects turns and counts gait cycles
to rebuild your walked route and plot it on screen.

## 📲 Download

Scan the QR code to download and install the latest build directly on your Android phone:

<p align="center">
  <img src="assets/download-qr.png" alt="Scan to download PocketPath APK" width="220" />
</p>

<p align="center"><strong>
  <a href="https://github.com/neo-askesis/TerrainMappers2/releases/latest/download/PocketPath.apk">
    Or tap here to download PocketPath.apk
  </a>
</strong></p>

**Installing:** Android will block the install until you allow it. When prompted, tap
*Settings → allow installs from this source* (the exact wording depends on your browser
or file manager), then open the APK again to install.

> This is a debug build published automatically from the latest commit on `master` — it's
> meant for direct install/testing, not a Play Store release. Requires Android 8.0 (API 26) or newer.

The APK is rebuilt automatically on every push via [GitHub Actions](.github/workflows/build-release.yml),
so the link and QR code above always point to the newest version — no need to update them.

## ✨ How it works

1. **Record** — tap *Start recording* and walk with the phone in your pocket. A foreground
   service ([`RecorderService`](app/src/main/java/com/example/TerrainMappers/RecorderService.kt))
   keeps the sensors sampling even with the screen off.
2. **Reconstruct** — [`PathEngine`](app/src/main/java/com/example/TerrainMappers/PathEngine.kt)
   combines gyroscope + gravity to detect turns, gravity oscillation to count gait cycles, and
   accelerometer-based stride length (Weinberg model) to turn those cycles into distance — with
   a magnetometer cross-check on heading.
3. **Review** — the walked path is drawn live on screen
   ([`PathView`](app/src/main/java/com/example/TerrainMappers/PathView.kt)), and you can tweak
   stride length / start bearing and tap *Re-map* to recompute without re-recording.
4. **Export** — *Export CSV* writes Phyphox-style `Gyroscope.csv`, `Gravity.csv`,
   `Accelerometer.csv`, `Magnetometer.csv`, `path.csv`, and `report.txt` to the app's external
   files folder, so a recording can also be analyzed in your own scripts.

## 🔐 Permissions

| Permission | Why |
|---|---|
| `ACTIVITY_RECOGNITION` | Required on Android 10+ to keep motion sensors sampling in the background |
| `FOREGROUND_SERVICE` / `FOREGROUND_SERVICE_HEALTH` | Keeps recording running with the screen off |
| `POST_NOTIFICATIONS` | Shows the "recording in progress" notification on Android 13+ |
| `WAKE_LOCK` | Prevents the CPU from sleeping mid-recording |
| `HIGH_SAMPLING_RATE_SENSORS` | Allows sensor sampling above the default rate limit |

No location, camera, microphone, or network access is requested — everything runs on-device.

## 🛠 Building from source

```bash
git clone https://github.com/neo-askesis/TerrainMappers2.git
cd TerrainMappers2
./gradlew assembleDebug
```

The debug APK is written to `app/build/outputs/apk/debug/app-debug.apk`. You can also open
the project in Android Studio and run it directly on a device or emulator (a real device is
strongly recommended — the walking algorithm needs real motion sensor data).

**Requirements:** JDK 21, Android SDK with platform 37 / build-tools, Gradle (via the included
`./gradlew` wrapper — no separate install needed).

## 📂 Project structure

```
app/src/main/java/com/example/TerrainMappers/
├── MainActivity.kt      # UI: start/stop, stride & bearing inputs, live status, export
├── RecorderService.kt   # Foreground service sampling the sensors
├── Dsp.kt               # Signal processing helpers (filtering, peak/cycle detection)
├── PathEngine.kt         # Core dead-reckoning algorithm: turns, gait cycles, stride → path
└── PathView.kt           # Draws the reconstructed path on screen
```

## License

No license has been specified yet for this project.
