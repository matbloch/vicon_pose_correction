## Include modules (header files)
INCLUDE_DIRECTORIES(
  .
)

## Set common link libraries
SET(COMMON_LIBS 
gtsam
)

## Main executables
ADD_EXECUTABLE(main_program main.cpp)
TARGET_LINK_LIBRARIES(main_program
  ${COMMON_LIBS}		# 3rd party libraries
)
