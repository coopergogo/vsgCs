set(SOURCES
    cesium_client_offscreen.cpp
)

add_executable(cesium_client_offscreen ${SOURCES})

target_link_libraries(cesium_client_offscreen vsg::vsg)

if (vsgXchange_FOUND)
    target_compile_definitions(cesium_client_offscreen PRIVATE vsgXchange_FOUND)
    target_link_libraries(cesium_client_offscreen vsgXchange::vsgXchange)
endif()

install(TARGETS cesium_client_offscreen RUNTIME DESTINATION bin)
