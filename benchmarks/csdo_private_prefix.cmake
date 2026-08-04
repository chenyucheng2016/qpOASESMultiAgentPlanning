if(NOT CSDO_PRIVATE_PREFIX)
    message(FATAL_ERROR "Set CSDO_PRIVATE_PREFIX to the isolated CSDO dependency prefix")
endif()

# CSDO includes <eigen3/Eigen/...> and <osqp/osqp.h>, while the imported
# targets export the nested include directories. Put the prefix roots first
# without modifying the external CSDO checkout.
include_directories(BEFORE
    "${CSDO_PRIVATE_PREFIX}/include"
    "${CSDO_PRIVATE_PREFIX}/include/eigen3"
)
