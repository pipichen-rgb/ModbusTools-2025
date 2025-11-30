# doxygen Configuration

cmake_minimum_required(VERSION 3.13) # 2.2 - case insensitive syntax
                                     # 3.13 included policy CMP0077

find_program(MBTOOLS_DOXYGEN_EXECUTABLE
    NAMES doxygen doxygen.exe
    PATHS
        "$ENV{ProgramFiles}\\Doxygen\\bin"
        "$ENV{ProgramFiles\(x86\)}\\Doxygen\\bin"
    DOC "Doxygen documentation generator"
)

if(MBTOOLS_DOXYGEN_EXECUTABLE)
    message(STATUS "MBTOOLS-DOC: Found Doxygen: ${MBTOOLS_DOXYGEN_EXECUTABLE}")

    project("mbtools-doc" VERSION ${MBTOOLS_VERSION})

    set(MBTOOLS_DOC_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}")
    set(MBTOOLS_DOC_DIR "${CMAKE_BINARY_DIR}/doc")
    set(MBTOOLS_DOC_CLIENT_FILE_NAME "DoxyfileClient")
    set(MBTOOLS_DOC_CLIENT_FILE "${MBTOOLS_DOC_DIR}/${MBTOOLS_DOC_CLIENT_FILE_NAME}")
    set(MBTOOLS_DOC_SERVER_FILE_NAME "DoxyfileServer")
    set(MBTOOLS_DOC_SERVER_FILE "${MBTOOLS_DOC_DIR}/${MBTOOLS_DOC_SERVER_FILE_NAME}")

    # Ensure documentation output directories exist before running Doxygen
    file(MAKE_DIRECTORY "${MBTOOLS_DOC_DIR}")
    file(MAKE_DIRECTORY "${MBTOOLS_DOC_DIR}/output")
    file(MAKE_DIRECTORY "${MBTOOLS_DOC_DIR}/output/client")
    file(MAKE_DIRECTORY "${MBTOOLS_DOC_DIR}/output/server")

    message(STATUS "MBTOOLS-DOC: Create '${MBTOOLS_DOC_CLIENT_FILE}'")
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${MBTOOLS_DOC_CLIENT_FILE_NAME}.in
                   ${MBTOOLS_DOC_CLIENT_FILE})

    message(STATUS "MBTOOLS-DOC: Create '${MBTOOLS_DOC_SERVER_FILE}'")
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${MBTOOLS_DOC_SERVER_FILE_NAME}.in
                   ${MBTOOLS_DOC_SERVER_FILE})

    add_custom_target(${MBTOOLS_DOC_CLIENT_FILE_NAME} ALL
            WORKING_DIRECTORY "${MBTOOLS_DOC_DIR}"
            COMMAND "${MBTOOLS_DOXYGEN_EXECUTABLE}" "${MBTOOLS_DOC_CLIENT_FILE}"
            COMMENT "Build documentation"
            )

    add_custom_target(${MBTOOLS_DOC_SERVER_FILE_NAME} ALL
            WORKING_DIRECTORY "${MBTOOLS_DOC_DIR}"
            COMMAND "${MBTOOLS_DOXYGEN_EXECUTABLE}" "${MBTOOLS_DOC_SERVER_FILE}"
            COMMENT "Build documentation"
            )
else()
    message(WARNING "MBTOOLS-DOC: Doxygen not found. Documentation will not be generated.")
    message(STATUS "MBTOOLS-DOC: To install Doxygen, visit: https://www.doxygen.nl/download.html")
endif()

