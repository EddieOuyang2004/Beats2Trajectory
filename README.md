# Beats2Trajectory

This repository now contains two separate flows:

- `legacy_music_pipeline/`: the original offline music-to-trajectory pipeline, kept as-is and moved into its own folder
- `adaptive_clap/`: a new adaptive tempo project that retimes a looping trajectory from manual taps or microphone clap onsets

## Project Layout

```text
Beats2Trajectory/
  legacy_music_pipeline/
    audio/
    outputs/
    src/
    README.md
  adaptive_clap/
    outputs/
    src/
    README.md
```

## Start Here

All commands in the subproject READMEs are intended to be run from the repository root.

For the original music-driven workflow, see [legacy_music_pipeline/README.md](legacy_music_pipeline/README.md).

For the new adaptive trajectory-speed workflow, see [adaptive_clap/README.md](adaptive_clap/README.md).
