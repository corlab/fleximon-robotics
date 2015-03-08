
# mark-start::test-output-folder
SET(TEST_RESULT_DIR "${CMAKE_BINARY_DIR}/testresults")
# mark-end::test-output-folder

# mark-start::test-includes
INCLUDE_DIRECTORIES(BEFORE SYSTEM ${GMOCK_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(BEFORE "${CMAKE_SOURCE_DIR}/src")
# mark-end::test-includes

# mark-start::test-declaration
SET(TEST_NAME "${LIBRARY_NAME}-test")

SET(TEST_SOURCES "ur5-microservice/library/ur5-rsb-adapter.cpp")
ADD_EXECUTABLE(${TEST_NAME} ${TEST_SOURCES})

TARGET_LINK_LIBRARIES(${TEST_NAME}
                      ${LIBRARY_NAME}
                      ${GMOCK_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})
                      
ADD_TEST(${TEST_NAME} ${TEST_NAME} "--gtest_output=xml:${TEST_RESULT_DIR}/")
# mark-end::test-declaration
