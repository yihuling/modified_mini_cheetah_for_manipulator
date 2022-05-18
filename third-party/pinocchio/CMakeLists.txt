SET(PINOCCHIO_INCLUDE /opt/openrobots/include)
SET(PINOCCHIO_LIB /opt/openrobots/lib)

INCLUDE_DIRECTORIES(${PINOCCHIO_INCLUDE})
LINK_DIRECTORIES(${PINOCCHIO_LIB})
ADD_EXECUTABLE(forwardKinematics forwardKinematics.cpp)
TARGET_LINK_LIBRARIES(forwardKinematics libeigenpy.so libhpp-fcl.so libpinocchio.so libpinocchio.so.2.6.5)

INCLUDE_DIRECTORIES(${PINOCCHIO_INCLUDE})
LINK_DIRECTORIES(${PINOCCHIO_LIB})
ADD_EXECUTABLE(inverseKinematics inverseKinematics.cpp)
TARGET_LINK_LIBRARIES(inverseKinematics libeigenpy.so libhpp-fcl.so libpinocchio.so libpinocchio.so.2.6.5)
