if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

if(ENABLE_tests)

    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)
    include_directories(${GTestSources_INCLUDE_DIR}})

    if(ENABLE_CheckSelfCollisionLibrary)
        add_executable(testCheckSelfCollisionLibrary testCheckSelfCollisionLibrary.cpp)

        target_link_libraries(testCheckSelfCollisionLibrary ROBOTICSLAB::ColorDebug
                                                 CheckSelfCollisionLibrary
                                                 gtest_main)

        gtest_add_tests(testCheckSelfCollisionLibrary "" AUTO)
    endif()

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()
