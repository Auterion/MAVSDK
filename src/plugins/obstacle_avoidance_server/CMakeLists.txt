add_library(mavsdk_obstacle_avoidance_server
    obstacle_avoidance_server.cpp
    obstacle_avoidance_server_impl.cpp
)

target_link_libraries(mavsdk_obstacle_avoidance_server
    mavsdk
)

set_target_properties(mavsdk_obstacle_avoidance_server
    PROPERTIES COMPILE_FLAGS ${warnings}
)

target_include_directories(mavsdk_obstacle_avoidance_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(TARGETS mavsdk_obstacle_avoidance_server
    EXPORT mavsdk-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

install(FILES
    include/plugins/obstacle_avoidance_server/obstacle_avoidance_server.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/obstacle_avoidance_server
)