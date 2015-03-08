# mark-start::cmake-project-declaration
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT("ur5-microservice")
# mark-end::cmake-project-declaration

# mark-start::user-options
OPTION(BUILD_TESTS "Decides whether the unit tests will be built." ON)
# mark-end::user-options

# mark-start::rpath
SET(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib:\$ORIGIN/")
# mark-end::rpath

# mark-start::upstream-dependencies
FIND_PACKAGE(RSC 0.11 REQUIRED)
FIND_PACKAGE(RSB 0.11 REQUIRED)
FIND_PACKAGE(RST 0.11 REQUIRED)

INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RST_INCLUDE_DIRS} ${RSB_INCLUDE_DIRS})
ADD_DEFINITIONS(${RST_CFLAGS})

LIST(INSERT CMAKE_MODULE_PATH 0 ${RSC_CMAKE_MODULE_PATH})
# mark-end::upstream-dependencies

# mark-start::include-cmake-code
INCLUDE(InstallFilesRecursive)
INCLUDE(DefineProjectVersion)
INCLUDE(GenerateDoxygen)
INCLUDE(PedanticCompilerWarnings)
# mark-end::include-cmake-code

# mark-start::version-variables
DEFINE_PROJECT_VERSION("UR5_MICROSERVICE_" 0 1 0 "archive")
SET(SO_VERSION "${UR5_MICROSERVICE_VERSION_MAJOR}.${ESSENTIALS_VERSION_MINOR}")
# mark-end::version-variables

# mark-start::define-target-names
SET(LIBRARY_NAME "ur5-microservice")
SET(BINARY_NAME_SERVICE "rsb-ur5-microservice")
SET(BINARY_NAME_CLIENT "rsb-ur5-microservice-client")
SET(EXPORT_NAME "ur5-microservice-exports")
# mark-end::define-target-names

# mark-start::include-src-directory
ADD_SUBDIRECTORY(src)
# mark-end::include-src-directory
# mark-start::include-test-directory
ENABLE_TESTING()
IF(BUILD_TESTS)
    INCLUDE(ProvideGoogleMock)
    IF(GMOCK_AVAILABLE)
        FIND_PACKAGE(Threads REQUIRED)
        ADD_SUBDIRECTORY(test)
    ELSE()
        MESSAGE(WARNING "Could not build unit tests even though desired because Google Mock could not be installed.")
    ENDIF()
ENDIF()
# mark-end::include-test-directory

# mark-start::cmake-config-files
CONFIGURE_FILE(ur5-microservice-config.cmake.in
               "${CMAKE_BINARY_DIR}/ur5-microservice-config.cmake"
               @ONLY)
CONFIGURE_FILE(ur5-microservice-config-version.cmake.in
               "${CMAKE_BINARY_DIR}/ur5-microservice-config-version.cmake"
               @ONLY)
CONFIGURE_FILE(ur5-microservice-build-tree-settings.cmake.in
               "${CMAKE_BINARY_DIR}/ur5-microservice-build-tree-settings.cmake"
               @ONLY)
               
INSTALL(FILES "${CMAKE_BINARY_DIR}/ur5-microservice-config.cmake"
              "${CMAKE_BINARY_DIR}/ur5-microservice-config-version.cmake"
        DESTINATION "share/ur5-microservice")

EXPORT(TARGETS ${LIBRARY_NAME} FILE "${CMAKE_BINARY_DIR}/${EXPORT_NAME}.cmake")
INSTALL(EXPORT ${EXPORT_NAME}
        DESTINATION "share/ur5-microservice")
# mark-end::cmake-config-files

# mark-start::documentation
GENERATE_DOXYGEN(VERSION "${ESSENTIALS_VERSION}")
# mark-end::documentation
