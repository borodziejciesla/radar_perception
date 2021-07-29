# Additional target to perform cppcheck run, requires cppcheck

# Get all project files
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.hpp)

# Remove files from 3rd party libraries
foreach (SOURCE_FILE ${ALL_SOURCE_FILES})

    message(STATUS "${SOURCE_FILE}")

    set(EXCUDED_DIR "test")
    string(FIND ${SOURCE_FILE} ${EXCUDED_DIR} PROJECT_TRDPARTY_DIR_FOUND)
    if (NOT ${PROJECT_TRDPARTY_DIR_FOUND} EQUAL -1)
        list(REMOVE_ITEM ALL_SOURCE_FILES ${SOURCE_FILE})
    endif ()

    set(EXCUDED_DIR "CMakeCXXCompilerId.cpp")
    string(FIND ${SOURCE_FILE} ${EXCUDED_DIR} PROJECT_TRDPARTY_DIR_FOUND)
    if (NOT ${PROJECT_TRDPARTY_DIR_FOUND} EQUAL -1)
        list(REMOVE_ITEM ALL_SOURCE_FILES ${SOURCE_FILE})
    endif ()
endforeach ()


# Run CppCheck
find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)

add_custom_target(
    cppcheck
    COMMAND ${CMAKE_CXX_CPPCHECK}
    --enable=all#warning,performance,portability,information,missingInclude
    --std=c++20
    --inconclusive
    --library=qt.cfg
    --template="[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)"
    --verbose
    --inline-suppr
    --force
    --output-file=${CMAKE_SOURCE_DIR}/suppressions.txt
    ${ALL_SOURCE_FILES}
)