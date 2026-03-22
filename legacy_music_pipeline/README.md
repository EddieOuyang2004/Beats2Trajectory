# Legacy Music Pipeline

This folder keeps the original offline pipeline unchanged:

1. Extract `beat`, `RMS`, and `onset` features with `librosa`
2. Generate a continuous dance trajectory
3. Save trajectory targets to `CSV`
4. Play the trajectory in `PyBullet`, with optional synchronized audio playback

## Generate Trajectory CSV

```powershell
python legacy_music_pipeline/src/generate_trajectory.py legacy_music_pipeline/audio/your_song.mp3 --out legacy_music_pipeline/outputs/trajectory.csv
```

Optional tuning:

```powershell
python legacy_music_pipeline/src/generate_trajectory.py legacy_music_pipeline/audio/your_song.mp3 `
  --out legacy_music_pipeline/outputs/trajectory.csv `
  --traj-hz 100 `
  --base-amp-deg 30 `
  --base-b-amp-deg 24 `
  --accent-duration 0.15 `
  --accent-gain-deg 14 `
  --onset-quantile 0.9
```

## Convert MP3 to WAV

```powershell
ffmpeg -i legacy_music_pipeline/audio/your_song.mp3 -ac 2 -ar 44100 legacy_music_pipeline/audio/your_song.wav
```

## Play Trajectory in PyBullet

```powershell
python legacy_music_pipeline/src/play_trajectory_pybullet.py --csv legacy_music_pipeline/outputs/trajectory.csv --realtime
```

Common options:

```powershell
python legacy_music_pipeline/src/play_trajectory_pybullet.py --csv legacy_music_pipeline/outputs/trajectory.csv --realtime --loop --speed 1.2
python legacy_music_pipeline/src/play_trajectory_pybullet.py --csv legacy_music_pipeline/outputs/trajectory.csv --urdf your_robot.urdf --joint-yaw 0 --joint-pitch 1 --fixed-base
```

## Play Trajectory with Audio Sync

```powershell
python legacy_music_pipeline/src/play_trajectory_pybullet.py --csv legacy_music_pipeline/outputs/trajectory.csv --audio legacy_music_pipeline/audio/your_song.wav --realtime
```
