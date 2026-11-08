add_library(kinova_robot kinova_robot.cpp)
target_include_directories(kinova_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kinova_robot PUBLIC kortex_api matrix)
