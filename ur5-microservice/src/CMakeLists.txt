
# mark-start::declare-library
SET(LIBRARY_SOURCES "ur5-microservice/library/ur5-rsb-adapter.cpp")
SET(LIBRARY_HEADERS "ur5-microservice/library/ur5-rsb-adapter.h")
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES VERSION ${SO_VERSION})
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${RSC_LIBRARIES} ${RSB_LIBRARIES} ${RST_LIBRARIES})

# mark-end::declare-library

# mark-start::declare-executable
ADD_EXECUTABLE(${BINARY_NAME_SERVICE} "ur5-microservice/ur5-microservice.cpp")
TARGET_LINK_LIBRARIES(${BINARY_NAME_SERVICE} ${LIBRARY_NAME} ${RSB_LIBRARIES} ${RST_LIBRARIES})
# mark-end::declare-executable

# mark-start::declare-executable
ADD_EXECUTABLE(${BINARY_NAME_CLIENT} "ur5-microservice/ur5-microservice-client.cpp")
TARGET_LINK_LIBRARIES(${BINARY_NAME_CLIENT} ${LIBRARY_NAME} ${RSB_LIBRARIES} ${RST_LIBRARIES})
# mark-end::declare-executable

# mark-start::installation
INSTALL(TARGETS ${LIBRARY_NAME} ${BINARY_NAME_SERVICE} ${BINARY_NAME_CLIENT}
        EXPORT ${EXPORT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
INSTALL_FILES_RECURSIVE("include" LIBRARY_HEADERS)
# mark-end::installation
