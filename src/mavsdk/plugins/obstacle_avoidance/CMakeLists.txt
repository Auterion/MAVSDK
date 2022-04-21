target_sources(mavsdk
    PRIVATE
    obstacle_avoidance.cpp
    obstacle_avoidance_impl.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

install(FILES
    include/plugins/obstacle_avoidance/obstacle_avoidance.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/obstacle_avoidance
)
