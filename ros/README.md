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
