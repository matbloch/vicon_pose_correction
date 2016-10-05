## Include modules (header files)
INCLUDE_DIRECTORIES(
  .
)

## Set common link libraries
SET(COMMON_LIBS 
gtsam
gflags
)

## Main executables
ADD_EXECUTABLE(align_poses align_poses.cpp)
TARGET_LINK_LIBRARIES(align_poses
  ${COMMON_LIBS}		# 3rd party libraries
)
