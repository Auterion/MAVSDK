target_sources(mavsdk
    PRIVATE
    obstacle_avoidance_server.cpp
    obstacle_avoidance_server_impl.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(FILES
    include/plugins/obstacle_avoidance_server/obstacle_avoidance_server.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/obstacle_avoidance_server
)