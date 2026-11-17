# OpenLMM ROS 2 adapter

`open_lmm_rosnode` owns one headless OpenLMM `RuntimeClient`. It exposes the
pipeline action, status service, execution events, and read-only RViz
presentation topics. It does not load the OpenLMM GUI or Iridescence plugin.

Run the node and the supplied RViz configuration with:

```bash
ros2 launch open_lmm_ros open_lmm_rviz.launch.py \
  config_path:=/absolute/path/to/config
```

From the repository root, the developer Makefile can build the installed core,
build the isolated ROS overlay, and launch the same configuration in one step:

```bash
make ros CONFIG=/absolute/path/to/config
```

Use `make ros-build` to build without launching, `make ros-run` to reuse the
existing artifacts, and `make ros ROS_USE_RVIZ=false` for the headless node.

The visualization topics are:

```text
~/visualization/a_<encoded-agent>/points  sensor_msgs/PointCloud2
~/visualization/a_<encoded-agent>/path    nav_msgs/Path
~/visualization/loops                     visualization_msgs/MarkerArray
```

The installed RViz configuration groups Map, Path, and Constraint displays for
agents 1 through 7. Agent punctuation is encoded without collisions: `_`
becomes `_5f`, `-` becomes `_2d`, and `.` becomes `_2e`.

The supplied layout also opens the **OpenLMM Control** panel. It can run the
whole pipeline, an individual stage or node, optimize through a selected agent,
cancel the active action goal, and show authoritative runtime/job/revision
status. Its default target namespace is `/open_lmm_ros`; the value is stored in
the RViz configuration when changed. The panel talks only to `~/execute`,
`~/status`, and `~/events` and does not own another OpenLMM runtime.

Interactive alignment and transactional configuration forms remain separate
follow-up work. The standalone `open_lmm_gui` remains available, but it owns a
different runtime and does not attach to a running ROS node.

## Online mode (experimental, phase 1)

`open_lmm_online_node` streams keyframes from one odometry front end per robot
and re-merges periodically. It is a leaf adapter: the runtime, stages and
plugins are unchanged.

- **Input per agent:** `<topic_prefix>/<agent>/cloud_registered_body`
  (body-frame `PointCloud2`) and `<topic_prefix>/<agent>/Odometry`, with
  matching stamps. These are FAST-LIO's outputs.
- **Keyframes:** one keyframe every `keyframe_translation_m` (1.0 m) or
  `keyframe_rotation_deg` (10°). Each keyframe merges the last
  `scans_per_keyframe` (5) body scans in the keyframe frame, then
  voxel-filters them (`keyframe_voxel_m`, 0.1 m). Keyframes are appended to
  `<work_dir>/session_<t>/live/<agent>/{Scans/NNNNNN.pcd,optimized_poses.txt}`.
- **Merge:** every `merge_period_s` (60 s). It runs once at least `min_agents`
  agents have `min_keyframes_per_agent` keyframes each and new keyframes have
  arrived since the last success. The merge hard-links a snapshot, re-opens the
  RuntimeClient on it and runs RunAll.
- **RViz:** the topics are the same as `open_lmm_ros`. They refresh only when
  a merge job completes, so a failed merge keeps the last good map.
- **Status:** status text is published on `~/status`.

```bash
# FAST-LIO per robot (thirdparty/FAST_LIO_ROS2, ROS2 branch @ cc3c9e8; needs livox_ros_driver2)
ros2 launch open_lmm_ros fast_lio_multi.launch.py agents:="agent1 agent2" \
  lidar_topics:="/r1/points /r2/points" imu_topics:="/r1/imu /r2/imu"
# online merger + RViz
ros2 launch open_lmm_ros open_lmm_online.launch.py config_path:=/path/to/config agents:="agent1 agent2"
```

Use a config with `enable_map_updater: false`; dynamic-object removal is too
slow to re-run every period.

**Limitation:** every merge recomputes everything from scratch. On bg2 (two
VLP-16 robots, FAST-LIO bag replayed at 1x, 60 s period), merge time grew from
4.2 s (77/89 keyframes) to 48.5 s (544/432 keyframes). Phase 2 should feed
keyframes into loop detection and iSAM2 incrementally.
