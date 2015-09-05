set(FILES_TO_COPY
    points.vtk
    SpherePoints.vtp
    RecordedModel_2015-09-04.vtk
    )

foreach(FILENAME ${FILES_TO_COPY})
  configure_file(${FILENAME} ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
  message(STATUS "COPY: " ${FILENAME} " copied to " ${CMAKE_CURRENT_BINARY_DIR})
endforeach(FILENAME ${FILES_TO_COPY})