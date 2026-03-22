# Adaptive Clap Project

This folder contains the new adaptive trajectory-speed workflow:

1. Generate or provide a looping trajectory CSV
2. Estimate beat period from recent taps or microphone clap onsets
3. Retime playback with a phase integrator instead of wall-clock `elapsed * speed`

## 1) Generate a Simple Clap Trajectory

```powershell
python adaptive_clap/src/generate_clap_trajectory.py --out adaptive_clap/outputs/clap_trajectory.csv
```

## 2) Tap Tempo with Space Bar

This is the first development step. Start the player and tap the space bar in the PyBullet window.

```powershell
python adaptive_clap/src/adaptive_play_trajectory.py --csv adaptive_clap/outputs/clap_trajectory.csv --input-source tap --realtime
```

Controls:

- `Space`: register a beat
- `R`: reset phase and tempo controller
- `Q`: quit
- `Esc`: also quits on PyBullet builds that expose `B3G_ESCAPE`

The tempo estimator uses the median of the most recent `3..5` beat intervals.

## 3) Microphone Clap Onset Input

Install dependencies from `requirements.txt`, then run:

```powershell
python adaptive_clap/src/adaptive_play_trajectory.py --csv adaptive_clap/outputs/clap_trajectory.csv --input-source mic --realtime
```

Useful options:

```powershell
python adaptive_clap/src/adaptive_play_trajectory.py `
  --csv adaptive_clap/outputs/clap_trajectory.csv `
  --input-source mic `
  --beats-per-cycle 2 `
  --speed-min 0.5 `
  --speed-max 1.8 `
  --mic-threshold-scale 3.5
```

## Notes

- The adaptive player assumes the input trajectory is loopable.
- `beats-per-cycle` defines how many detected beats map to one full trajectory cycle.
- In `mic` mode, `sounddevice` must be available and a microphone must be accessible.
