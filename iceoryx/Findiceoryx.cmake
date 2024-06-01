# Find the Iceoryx include directory
find_path(iceoryx_INCLUDE_DIRS
  NAMES iceoryx_posh
  HINTS /usr/local/include/iceoryx/v /usr/include/iceoryx/v
  DOC "iceoryx include directories")

# Find the Iceoryx library
find_library(iceoryx_LIBRARY
  NAMES iceoryx_posh
  HINTS /usr/local/lib
  DOC "iceoryx libraries")

# Set the Iceoryx libraries
set(iceoryx_LIBRARIES ${iceoryx_LIBRARY})

# Include the find package handle standard args module
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(iceoryx DEFAULT_MSG
  iceoryx_INCLUDE_DIRS iceoryx_LIBRARY)

# Mark the cache variables as advanced
mark_as_advanced(iceoryx_INCLUDE_DIRS iceoryx_LIBRARY)
