# Trossen Arm Scripts

This directory contains tools and scripts that can be used to configure, tune, and operate Trossen Arms.

## Joint Fine-Tuning Script

The [`tuning.py`](./tuning.py) script provides an interactive terminal interface for fine-tuning the joint characteristics of a Trossen Arm.

### dual-arm-vla fork changes

This branch (`dual-arm-vla`, based on `v1.10.0`) changes `tuning.py` for use inside the
dual-arm-vla thesis project:

- The inline `uv` script metadata is removed, so the script runs in the project's locked
  environment (`uv run third_party/trossen_arm/scripts/tuning.py`). The upstream metadata
  resolved the latest `trossen_arm`, which can mismatch the controller firmware.
- After connecting, it saves the current joint characteristics to
  `configs/joint_characteristics/wxai_v0_<ip>_<time>_before.json` before any change. On
  exit it saves `..._after.json` if any values changed.
- `--restore <file.json>` writes a backup back to the arm it was saved from. It asks for
  confirmation first and verifies the values by reading them back.
- It holds the project hardware lock (`outputs/lerobot_hardware.lock`) while it runs.
