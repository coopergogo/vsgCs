set(SOURCES
    cesium_client_offscreen.cpp
    UI.cpp
)

INCLUDE_DIRECTORIES(${Vulkan_INCLUDE_DIR})

add_executable(cesium_client_offscreen ${SOURCES})


target_link_libraries(cesium_client_offscreen vsgCs CsApp vsg::vsg)

target_link_libraries(cesium_client_offscreen vsgImGui::vsgImGui)

if (vsgXchange_FOUND)
    target_compile_definitions(cesium_client_offscreen PRIVATE vsgXchange_FOUND)
    target_link_libraries(cesium_client_offscreen vsgXchange::vsgXchange)
endif()

install(TARGETS cesium_client_offscreen
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)