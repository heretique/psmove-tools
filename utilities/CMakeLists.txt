find_package(OpenCV CONFIG REQUIRED)
find_package(rttr CONFIG REQUIRED)
find_package(raylib CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(PSMoveConfigurator 
    imgui/imgui_impl_raylib.cpp
    imgui/imgui_impl_raylib.h
    imgui/RttrImguiInspector.cpp
    imgui/RttrImguiInspector.h
    PSMoveConfiguratorMain.cpp 
    PSMoveConfigurator.cpp
    PSMoveConfigurator.h)

target_include_directories(PSMoveConfigurator PRIVATE imgui ${RAYLIB_INCLUDE_DIRS})
target_link_libraries(PSMoveConfigurator RTTR::Core ${RAYLIB_LIBRARIES} opencv_world imgui::imgui psmove psmove_tracker)