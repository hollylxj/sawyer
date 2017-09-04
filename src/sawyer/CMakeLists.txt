
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
FIND_PACKAGE(SWIG REQUIRED)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -I/System/Library/Frameworks/Python.framework/Versions/2.7/include/python2.7")

#ADD_LIBRARY (SawyerControllerShared SHARED
#        ${CS225A_COMMON_SOURCE}
#        SawyerController_wrap.cxx
#	SawyerController.cpp
#)

# and link the library against the executable
#TARGET_LINK_LIBRARIES (SawyerControllerShared
#        ${CS225A_COMMON_LIBRARIES}
#	${PYTHON_LIBRARIES})


INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_SWIG_FLAGS "")
SET_SOURCE_FILES_PROPERTIES(SawyerController.i PROPERTIES CPLUSPLUS ON)
#SET_SOURCE_FILES_PROPERTIES(SawyerController.i PROPERTIES SWIG_FLAGS "-includeall")


# Add swig module
SWIG_ADD_LIBRARY(SawyerController LANGUAGE python SOURCES SawyerController.i SawyerController.cpp ${CS225A_COMMON_SOURCE})
SWIG_LINK_LIBRARIES(SawyerController ${CS225A_COMMON_LIBRARIES} ${PYTHON_LIBRARIES})
execute_process(COMMAND python -c "from distutils.sysconfig import get_python_lib; print get_python_lib()" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

# Set output dir
SET(PYTHON_DEST ${PROJECT_SOURCE_DIR}/src/sawyer)
SET(CMAKE_SWIG_OUTDIR ${PYTHON_DEST})
SET_TARGET_PROPERTIES(_SawyerController PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PYTHON_DEST})

#execute_process(COMMAND python -c "from distutils.sysconfig import get_python_lib; print get_python_lib()" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

# Install
INSTALL(TARGETS _SawyerController DESTINATION ${PYTHON_DEST})
INSTALL(FILES SawyerController.py  DESTINATION ${PYTHON_DEST})

#==========================

# create an executable
#ADD_EXECUTABLE (sawyer 
#        ${CS225A_COMMON_SOURCE} 
#        sawyer.cpp
#        SawyerController.cpp
#)

# and link the library against the executable
#TARGET_LINK_LIBRARIES (sawyer
#        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/sawyer)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY sawyer.urdf world.urdf DESTINATION ${APP_RESOURCE_DIR})
