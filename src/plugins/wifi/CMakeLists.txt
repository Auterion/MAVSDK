add_library(mavsdk_wifi
    wifi.cpp
    wifi_impl.cpp
)

target_link_libraries(mavsdk_wifi
    mavsdk
)

set_target_properties(mavsdk_wifi
    PROPERTIES COMPILE_FLAGS ${warnings}
)

target_include_directories(mavsdk_wifi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(TARGETS mavsdk_wifi
    EXPORT mavsdk-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

install(FILES
    include/plugins/wifi/wifi.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/wifi
)
