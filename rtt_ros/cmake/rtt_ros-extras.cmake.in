
macro(use_orocos)
  find_package(OROCOS-RTT REQUIRED 
    COMPONENTS rtt-marshalling rtt-scripting ${ARGN})
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endmacro()

set(catkin_FOUND True)

use_orocos()
