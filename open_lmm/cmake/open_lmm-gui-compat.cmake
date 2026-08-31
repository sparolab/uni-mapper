# OpenLMM v3 package compatibility for the extracted GUI artifact.
# The canonical implementation target is provided by OpenLmmGui. This file
# creates no library or runtime owner; it only restores the legacy target name.

set(_open_lmm_gui_requested FALSE)
if(NOT open_lmm_FIND_COMPONENTS)
  find_package(OpenLmmGui 3.0.0 EXACT CONFIG QUIET)
elseif("gui" IN_LIST open_lmm_FIND_COMPONENTS)
  set(_open_lmm_gui_requested TRUE)
  find_package(OpenLmmGui 3.0.0 EXACT CONFIG QUIET)
endif()

if(OpenLmmGui_FOUND AND TARGET open_lmm_gui::gui)
  if(NOT TARGET open_lmm::gui)
    add_library(open_lmm::gui ALIAS open_lmm_gui::gui)
  endif()
  set(open_lmm_gui_FOUND TRUE)
  if(_open_lmm_gui_requested)
    message(DEPRECATION
      "open_lmm::gui is a v3 compatibility alias; use open_lmm_gui::gui "
      "from find_package(OpenLmmGui 3.0.0 EXACT CONFIG)")
  endif()
elseif(_open_lmm_gui_requested)
  set(open_lmm_gui_FOUND FALSE)
  set(open_lmm_FOUND FALSE)
  set(open_lmm_NOT_FOUND_MESSAGE
    "open_lmm COMPONENTS gui requires the exact OpenLmmGui 3.0.0 artifact")
endif()

unset(_open_lmm_gui_requested)
set(_open_lmm_targets_loaded TRUE)

# ament's generated config has a package-wide early-return guard. Clear it so
# a later component-qualified find_package(open_lmm ...) call (the existing
# ROS client-then-GUI pattern) can evaluate the requested component.
unset(_open_lmm_CONFIG_INCLUDED)
