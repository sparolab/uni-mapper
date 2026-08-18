# Headers installed by the former recursive 1.x Development component which
# no longer exist in the source tree. Keep this list versioned and explicit so
# an in-place upgrade removes only paths that OpenLMM previously owned.
set(_open_lmm_legacy_installed_headers_1_x
  core/loop_detector/descriptor_factory/kdtree/database_base.hpp
  core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp
  core/loop_detector/scan_context_v2_adapter.hpp
  server/map_aligner/map_aligner.hpp
  server/map_updater/map_updater.hpp
)
