set(IGRAPH_INCLUDE_DIR_HINTS "" CACHE PATH "igraph include directory")
set(IGRAPH_LIBRARY_DIR_HINTS "" CACHE PATH "igraph library directory")

unset(IGRAPH_FOUND)
unset(IGRAPH_INCLUDE_DIRS)
unset(IGRAPH_LIBRARIES)

list(APPEND IGRAPH_CHECK_INCLUDE_DIRS
    /usr/include 
    /usr/local/include
    /usr/local/homebrew/include
    /opt/local/val/macports/software
    /opt/local/include)

list(APPEND IGRAPH_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/lib
    /usr/local/homebrew/lib
    /opt/local/lib)

find_path(IGRAPH_INCLUDE_DIRS 
          NAMES
          igraph/igraph.h
          PATHS
          ${IGRAPH_INCLUDE_DIR_HINTS}
          ${IGRAPH_CHECK_INCLUDE_DIRS})

find_library(IGRAPH_LIBRARIES
            NAMES
            igraph
            libigraph
            PATHS
            ${IGRAPH_LIBRARY_DIR_HINTS}
            ${IGRAPH_CHECK_LIBRARY_DIRS})

if (IGRAPH_INCLUDE_DIRS AND IGRAPH_LIBRARIES)
    set(IGRAPH_FOUND TRUE)
    message(STATUS "Found igraph")
    message(STATUS "  Includes : ${IGRAPH_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${IGRAPH_LIBRARIES}")
else()
    if(igraph_FIND_REQUIRED)
        message(FATAL_ERROR "Cound not find igraph")
    endif(igraph_FIND_REQUIRED)
endif()
    