include(GitInfo) # YCM

# Define current version.
set(TEO_SHARON_VERSION_SHORT ${TEO_SHARON_VERSION})

# Retrieve current revision from Git working tree.
git_wt_info(SOURCE_DIR "${CMAKE_SOURCE_DIR}"
            PREFIX TEO_SHARON)

if(DEFINED TEO_SHARON_GIT_WT_HASH)
    if(TEO_SHARON_GIT_WT_TAG_REVISION GREATER 0)
        set(TEO_SHARON_VERSION_REVISION ${TEO_SHARON_GIT_WT_TAG_REVISION})
        string(REPLACE "-" "" _date ${TEO_SHARON_GIT_WT_AUTHOR_DATE})
        set(TEO_SHARON_VERSION_SOURCE
            "${_date}.${TEO_SHARON_GIT_WT_DATE_REVISION}+git${TEO_SHARON_GIT_WT_HASH_SHORT}")
    endif()

    if(TEO_SHARON_GIT_WT_DIRTY)
        set(TEO_SHARON_VERSION_DIRTY "dirty")
    endif()
endif()

if(DEFINED TEO_SHARON_VERSION_SOURCE)
    if(NOT "${TEO_SHARON_GIT_WT_TAG}" STREQUAL "v${TEO_SHARON_VERSION_SHORT}")
        set(TEO_SHARON_VERSION
            "${TEO_SHARON_VERSION_SHORT}+${TEO_SHARON_VERSION_SOURCE}")
    else()
        set(TEO_SHARON_VERSION
           "${TEO_SHARON_VERSION_SHORT}+${TEO_SHARON_VERSION_REVISION}-${TEO_SHARON_VERSION_SOURCE}")
    endif()
elseif(NOT "${TEO_SHARON_GIT_WT_TAG}" STREQUAL "v${TEO_SHARON_VERSION_SHORT}")
    set(TEO_SHARON_VERSION "${TEO_SHARON_VERSION_SHORT}~dev")
else()
    set(TEO_SHARON_VERSION "${TEO_SHARON_VERSION_SHORT}")
endif()

if(DEFINED TEO_SHARON_VERSION_DIRTY)
    set(TEO_SHARON_VERSION "${TEO_SHARON_VERSION}+${TEO_SHARON_VERSION_DIRTY}")
endif()

# Print version.
message(STATUS "TEO_SHARON version: ${TEO_SHARON_VERSION_SHORT} (${TEO_SHARON_VERSION})")
