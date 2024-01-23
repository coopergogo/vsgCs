set(SOURCES
  cesium_client.cpp
  UI.cpp
)

SET(TARGET_SRC ${SOURCES})

INCLUDE_DIRECTORIES(${Vulkan_INCLUDE_DIR})

add_executable(cesium_client ${SOURCES})

target_link_libraries(cesium_client vsgCs CsApp vsg::vsg)

target_link_libraries(cesium_client vsgImGui::vsgImGui)

if (vsgXchange_FOUND)
  target_compile_definitions(cesium_client PRIVATE vsgXchange_FOUND)
  target_link_libraries(cesium_client vsgXchange::vsgXchange)
endif()

install(TARGETS cesium_client
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
