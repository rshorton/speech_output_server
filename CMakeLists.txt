cmake_minimum_required(VERSION 3.5)
project(speech_output_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(speech_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

set(client_library_name speech_output_client)

add_library(${client_library_name} SHARED
  src/speech_action_client.cpp)

target_include_directories(${client_library_name} PRIVATE
  <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  <INSTALL_INTERFACE:include>)

ament_target_dependencies(${client_library_name}
  "speech_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(${client_library_name} PLUGIN "speech_output_action_server::SpeakActionClient" EXECUTABLE speech_output_action_client)
install(TARGETS
  ${client_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

set(server_library_name speech_output_server)

add_library(${server_library_name} SHARED
  src/speech_action_server.cpp
  src/speech_output_proc.cpp)

target_include_directories(${server_library_name} PRIVATE
  <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  <INSTALL_INTERFACE:include>)

# Fix
include_directories(/home/ubuntu/ms_voice/speechsdk/include/cxx_api
  /home/ubuntu/ms_voice/speechsdk/include/c_api)
# Fix
target_link_libraries(${server_library_name}
  /home/ubuntu/ms_voice/speechsdk/lib/arm64/libMicrosoft.CognitiveServices.Speech.core.so
  -lcrypto
  -l:libasound.so.2)

ament_target_dependencies(${server_library_name}
  "speech_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(${server_library_name} PLUGIN "speech_output_action_server::SpeechOutputActionServer" EXECUTABLE speech_output_action_server)

install(TARGETS
  ${server_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()