cmake_minimum_required(VERSION 3.0.0)
project(ModularSlamBackEnd)

include(CTest)
enable_testing()

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the library source files
set(SRC_FILES

    Transform.cpp
    LandMark.cpp
    WorldModelInterface.cpp
    PoseFactor.cpp
    RefFrame.cpp
    WorldModelPlotter.cpp
    PlotDataSet.cpp
    SFPlot.cpp
    WorldModel.cpp

)

# Set the SFML libraries
set(SFML_LIBRARIES

    sfml-graphics 
    sfml-window 
    sfml-system
)

set(LIBRARIES
	${SFML_LIBRARIES} 
)

# Find and include the Eigen library
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add the core library
add_library( myLib 
             ${SRC_FILES} 
           )
# Link the necesary external libraries to the core library
target_link_libraries( myLib 
                       tbb
                       pthread 
                       Eigen3::Eigen 
                       gtsam 
                       ${LIBRARIES} 
                     )
# Add the directory with all the headers to the core library
target_include_directories( myLib PRIVATE 
                            "${PROJECT_SOURCE_DIR}" 
                          )

# Add the main.cpp with the test so the executable is created
add_executable( mainTest 
                main.cpp 
                ${SRC_FILES}
              )
# Link the necesary external libraries to the test
target_link_libraries( mainTest 
                       tbb 
                       pthread 
                       Eigen3::Eigen 
                       gtsam 
                       ${LIBRARIES} 
                     )
# Add the directory with all the headers to the test
target_include_directories( mainTest PRIVATE 
                            "${PROJECT_SOURCE_DIR}" 
                          )


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
