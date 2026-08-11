# Screenshots

`architecture.svg` is checked in and already used by the root `README.md`.

The four files below aren't included yet (this environment has no display to
capture them) — drop them in here with these exact names and the **Demo**
section in the root `README.md` will pick them up automatically (the
`<!-- -->`-commented `<img>` tags are already in place, just uncomment them
once the files exist):

| File | Capture |
|---|---|
| `gazebo-sim.png` | `ros2 launch four_control_bringup follower_sim.launch.py` — Gazebo window with the rover and the walking actor both visible |
| `detection-feed.png` | The `viewer_node` OpenCV window (or the dashboard's camera panel) showing a bounding box + confidence label on the tracked person |
| `web-dashboard.png` | `http://localhost:8000` — the 3D pose view, camera feed, and telemetry panel together |
| `rviz.png` | The RViz window from `rover.launch.py`, showing the robot model and TF tree |

Keep them reasonably sized (~1280px wide is plenty) so the repo stays light.
