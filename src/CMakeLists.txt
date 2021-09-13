# Set SOURCES variable
set(SOURCES
    
    Transform.cpp
    LandMark.cpp
    WorldModelInterface.cpp
    PoseFactor.cpp
    RefFrame.cpp
    WorldModelPlotter.cpp
    PlotDataSet.cpp
    SFPlot.cpp
    WorldModel.cpp
    UdpClient.cpp
    UdpServer.cpp
    Optimizer.cpp
)

# Set HEADERS_PUBLIC variable (public headers, included in the library)
set(HEADERS_PUBLIC
    
    Transform.h
    LandMark.h
    WorldModelInterface.h
    PoseFactor.h
    RefFrame.h
    WorldModelPlotter.h
    PlotDataSet.h
    SFPlot.h
    WorldModel.h
    KeyFrame.h
    UdpClientServer.h
    Optimizer.h
)

# Set HEADERS_PRIVATE variable, if needed.
# (these private headers won't be included in the library)
# set(HEADERS_PRIVATE
#     foo_private.h
# )

include(${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake)

# Set the SFML libraries
set(SFML_LIBRARIES
    sfml-graphics 
    sfml-window 
    sfml-system
)

# Set the GTSAM optimization libraries
set(GTSAM_LIBRARIES
    gtsam
)

set(LIBRARIES
	${SFML_LIBRARIES} 
	${GTSAM_LIBRARIES} 
)

# Find and include the Eigen library
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries( ${PROJECT_NAME} 
                       tbb
                       pthread 
                       Eigen3::Eigen  
                       ${LIBRARIES} 
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
                       ${LIBRARY_NAME}
                     )
# Add the directory with all the headers to the test
target_include_directories( mainTest PUBLIC 
                            ${LIBRARY_NAME} 
                          )
                     
# Create the executable for the server
add_executable( WorldModelServer 
                WorldModel.cpp 
                ${SRC_FILES}
              )
# Link the necesary external libraries to the test
target_link_libraries( WorldModelServer 
                       tbb 
                       pthread 
                       Eigen3::Eigen 
                       gtsam 
                       ${LIBRARIES} 
                       ${LIBRARY_NAME}
                     )
# Add the directory with all the headers to the test
target_include_directories( WorldModelServer PUBLIC 
                            ${LIBRARY_NAME} 
                          )
