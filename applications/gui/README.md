# OpenLMM standalone GUI

This package owns the OpenLMM v3 GUI host, presentation model, optional
Iridescence plugin, and the `open_lmm_gui` standalone executable. It builds only
against an exact installed OpenLMM core package.

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/path/to/open_lmm/prefix \
  -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=ON
cmake --build build --parallel
cmake --install build --prefix /path/to/combined/prefix
```

Run the installed application with:

```bash
open_lmm_gui <config_dir_path> [gui_plugin_path]
```

Without an explicit plugin path, the executable loads
`lib/libopen_lmm_iridescence_gui.so` from its installation prefix. A standalone
GUI owns a separate `RuntimeClient`; it does not share runtime state with a ROS
process using the same configuration.

The canonical CMake target is `open_lmm_gui::gui`. OpenLMM v3 combined packages
also provide the deprecated compatibility spelling `open_lmm::gui`.
