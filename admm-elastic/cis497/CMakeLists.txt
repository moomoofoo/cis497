include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_executable( rotate_lower_down ${CMAKE_CURRENT_SOURCE_DIR}/rotate_lower_down.cpp )
target_link_libraries( rotate_lower_down admmelastic ${RENDER_LIBS} )

add_executable( rotate ${CMAKE_CURRENT_SOURCE_DIR}/rotate.cpp )
target_link_libraries( rotate admmelastic ${RENDER_LIBS} )

add_executable( translate_lower_down ${CMAKE_CURRENT_SOURCE_DIR}/translate_lower_down.cpp )
target_link_libraries( translate_lower_down admmelastic ${RENDER_LIBS} )

add_executable( translate_down ${CMAKE_CURRENT_SOURCE_DIR}/translate_down.cpp )
target_link_libraries( translate_down admmelastic ${RENDER_LIBS} )

add_executable( translate_up ${CMAKE_CURRENT_SOURCE_DIR}/translate_up.cpp )
target_link_libraries( translate_up admmelastic ${RENDER_LIBS} )

add_executable( testing ${CMAKE_CURRENT_SOURCE_DIR}/testing.cpp )
target_link_libraries( testing admmelastic ${RENDER_LIBS} )

add_executable( testing2 ${CMAKE_CURRENT_SOURCE_DIR}/testing2.cpp )
target_link_libraries( testing2 admmelastic ${RENDER_LIBS} )

add_executable( testing3 ${CMAKE_CURRENT_SOURCE_DIR}/testing3.cpp )
target_link_libraries( testing3 admmelastic ${RENDER_LIBS} )
