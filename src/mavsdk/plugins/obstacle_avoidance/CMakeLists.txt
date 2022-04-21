add_library(mavsdk_obstacle_avoidance
    obstacle_avoidance.cpp
    obstacle_avoidance_impl.cpp
)

target_link_libraries(mavsdk_obstacle_avoidance
    mavsdk
)

set_target_properties(mavsdk_obstacle_avoidance
    PROPERTIES COMPILE_FLAGS ${warnings}
    VERSION ${MAVSDK_VERSION_STRING}
    SOVERSION ${MAVSDK_SOVERSION_STRING}
)

target_include_directories(mavsdk_obstacle_avoidance PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(TARGETS mavsdk_obstacle_avoidance
    EXPORT mavsdk-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

install(FILES
    include/plugins/obstacle_avoidance/obstacle_avoidance.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/obstacle_avoidance
)