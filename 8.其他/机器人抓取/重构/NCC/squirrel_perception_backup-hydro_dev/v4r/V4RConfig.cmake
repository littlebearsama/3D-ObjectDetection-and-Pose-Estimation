
# ======================================================
# Include directories to add to the user project:
# ======================================================

SET(V4R_INCLUDE_DIR ${V4R_DIR})
include_directories(${V4R_INCLUDE_DIR})


# ======================================================
# Link directories to add to the user project:
# ======================================================

SET(V4R_LIB_DIR ${V4R_DIR}/lib)
LINK_DIRECTORIES(${V4R_LIB_DIR})


# ====================================================================
# Link libraries:
# ====================================================================

SET(AK_LIBS akshm)