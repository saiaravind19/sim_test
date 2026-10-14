add_library(fleet_core STATIC src/map.cpp src/grid.cpp src/planner.cpp src/scene.cpp)
target_include_directories(fleet_core PUBLIC include)
target_link_libraries(fleet_core PUBLIC rvo2 nlohmann_json::nlohmann_json)
