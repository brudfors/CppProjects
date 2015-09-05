cmake_minimum_required (VERSION 2.8.11)
project (PoissonSurfaceReconstruction)

find_package (VTK REQUIRED)
if (VTK_FOUND)
  include (${VTK_USE_FILE})
endif (VTK_FOUND)

# PointSetProcessing
find_library(POINTSETPROCESSING_LIBRARY PointSetProcessing
          PATHS "H:/_B/VTK-filters/PointSetProcessingR/Release"
          DOC "Path to the PointSetProcessing library." )  
find_path(POINTSETPROCESSING_INCLUDE_DIR vtkPointSetNormalEstimation.h
          PATHS "H:/_S/VTK-filters/PointSetProcessing"
          DOC "Path to PointSetProcessing include directory." )                        

# PoissonReconstruction
find_path(POISSONRECONSTRUCTION_BINARY_DIR vtkPoissonReconstructionConfigure.h
          PATHS "H:/_B/VTK-filters/PoissonReconstructionR"
          DOC "Path to PoissonReconstruction binary directory." )
find_path(POISSONRECONSTRUCTION_INCLUDE_DIR vtkPoissonReconstruction.h
          PATHS "H:/_S/VTK-filters/PoissonReconstruction/source"
          DOC "Path to PoissonReconstruction include directory." )
find_library(POISSONRECONSTRUCTION_LIBRARY vtkPoissonReconstruction
          PATHS "H:/_B/VTK-filters/PoissonReconstructionR/lib/Release"
          DOC "Path to the PoissonReconstruction library." )
               
if (NOT POINTSETPROCESSING_LIBRARY)
  message( FATAL_ERROR "PointSetProcessing library not found." )
elseif (NOT POINTSETPROCESSING_INCLUDE_DIR)
  message( FATAL_ERROR "PointSetProcessing include directory not found." )          
elseif (NOT POISSONRECONSTRUCTION_LIBRARY)
  message( FATAL_ERROR "PoissonReconstruction library not found." )
elseif (NOT POISSONRECONSTRUCTION_BINARY_DIR)
  message( FATAL_ERROR "PoissonReconstruction binary directory not found." )
elseif (NOT POISSONRECONSTRUCTION_INCLUDE_DIR)
  message( FATAL_ERROR "PoissonReconstruction include directory not found." )
endif()

add_executable(PoissonSurfaceReconstruction main.cxx )

message("----------------")
message("${POINTSETPROCESSING_INCLUDE_DIR}")
message("${POISSONRECONSTRUCTION_INCLUDE_DIR}")
message("${POISSONRECONSTRUCTION_BINARY_DIR}")
message("${POINTSETPROCESSING_LIBRARY}")
message("${POISSONRECONSTRUCTION_LIBRARY}")
message("----------------")

target_include_directories(PoissonSurfaceReconstruction PUBLIC
  "${POINTSETPROCESSING_INCLUDE_DIR}"
  "${POISSONRECONSTRUCTION_INCLUDE_DIR}"
  "${POISSONRECONSTRUCTION_BINARY_DIR}"
)

target_link_libraries(PoissonSurfaceReconstruction 
  "${VTK_LIBRARIES}"
  "${POINTSETPROCESSING_LIBRARY}"
  "${POISSONRECONSTRUCTION_LIBRARY}"
)

add_subdirectory (Data)