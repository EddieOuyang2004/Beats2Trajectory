# Beats2Trajectory

This repository provides a minimal offline pipeline for music-driven 2DOF robot motion:

1. Extract `beat`, `RMS`, and `onset` features with `librosa`
2. Generate a continuous dance trajectory (Lissajous + beat keyframes + onset accents)
3. Save trajectory targets to `CSV`
4. Play the trajectory in `PyBullet`, with optional synchronized audio playback (Windows `wav`)

## 1) Environment Setup (Conda Recommended)

```powershell
# Create and activate env (first time only)
conda create -n librosa python=3.11 -y
conda activate librosa

# Project deps
pip install -r requirements.txt

# Player dependency
conda install -c conda-forge pybullet -y
```

If `conda` is not recognized in PowerShell:

```powershell
& "C:\Users\Eddie\miniconda3\Scripts\conda.exe" init powershell
```

Then restart PowerShell.

## 2) Generate Trajectory CSV

```powershell
python src/generate_trajectory.py .\audio\your_song.mp3 --out .\outputs\trajectory.csv
```

Optional tuning:

```powershell
python src/generate_trajectory.py .\audio\your_song.mp3 `
  --out .\outputs\trajectory.csv `
  --traj-hz 100 `
  --base-amp-deg 30 `
  --base-b-amp-deg 24 `
  --accent-duration 0.15 `
  --accent-gain-deg 14 `
  --onset-quantile 0.9
```

## 3) CSV Schema

Generated columns:

- `t`: time in seconds
- `yaw_rad`, `pitch_rad`: joint targets in radians
- `yaw_deg`, `pitch_deg`: same targets in degrees
- `phase`: beat phase in `[0, 1)`
- `amp_scale`: RMS-based amplitude scale (`~0.5..1.5`)
- `accent`: onset accent pulse term (radians)
- `tempo_bpm`: estimated BPM

## 4) Play Trajectory in PyBullet

```powershell
python src/play_trajectory_pybullet.py --csv outputs/trajectory.csv --realtime
```

Common options:

```powershell
# Loop playback at 1.2x speed
python src/play_trajectory_pybullet.py --csv outputs/trajectory.csv --realtime --loop --speed 1.2

# Use your own URDF and joint mapping
python src/play_trajectory_pybullet.py --csv outputs/trajectory.csv --urdf your_robot.urdf --joint-yaw 0 --joint-pitch 1 --fixed-base
```

## 5) Play Trajectory with Audio Sync

Audio sync in the current player uses `winsound`, so it supports Windows `.wav` files.

```powershell
python src/play_trajectory_pybullet.py --csv outputs/trajectory.csv --audio .\audio\your_song.wav --realtime
```

## 6) Convert MP3 to WAV

Install command-line `ffmpeg`:

```powershell
winget install -e --id Gyan.FFmpeg
```

Convert `mp3 -> wav`:

```powershell
ffmpeg -i .\audio\your_song.mp3 -ac 2 -ar 44100 .\audio\your_song.wav
```

If `ffmpeg` is not recognized, locate `ffmpeg.exe` and run it with an absolute path.

## 7) Troubleshooting

### Q1: `conda` is not recognized

- Cause: PowerShell has not been initialized for conda.
- Fix: run `conda init powershell`, then restart terminal.

### Q2: `numpy.dtype size changed` when importing `pandas`

- Cause: binary mismatch between `numpy` and `pandas` (often from mixed pip/conda installs).
- Fix: reinstall both from a single source inside one environment.

### Q3: No audio during playback

- Confirm `--audio` is provided.
- Confirm file extension is `.wav`.
- Confirm the audio path exists.
