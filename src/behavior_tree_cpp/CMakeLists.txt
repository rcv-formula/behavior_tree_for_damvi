cmake_minimum_required(VERSION 3.8)
project(behavior_tree_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# 공용 코드 라이브러리
add_library(bt_nodes src/bt_nodes.cpp)  # add_library는 실행파일이 아니라, 재사용하는 코드 덩어리(라이브러리)를 만든다는 뜻
# 여러 실행파일이 같은 코드를 쓰면, 중복 컴파일 대신 이걸 공유한다. => bt_nodes라는 라이브러리가 생김.
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(bt_nodes PUBLIC cxx_std_17)
ament_target_dependencies(bt_nodes
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
) # ament_target_dependencies는 패키지들을 target에 연결(include/link)해줍니다. 여기서 target는 저 bt_nodes 자리!

# bt_main 실행 파일
add_executable(bt_main src/bt_main.cpp) # add executable은 실행파일 만들라는 명령어(ros2 run ... 그놈)
target_link_libraries(bt_main bt_nodes) # bt_main 실행파일이 bt_nodes 라이브러리를 붙여서(link) 사용하게 함. 즉 bt_main에서 bt_nodes 안의 함수/클래스를 쓸 수 있음.
target_compile_features(bt_main PUBLIC cxx_std_17)
ament_target_dependencies(bt_main
  rclcpp
  behaviortree_cpp_v3
)

# health monitor 실행 파일
add_executable(health_monitor src/health_monitor.cpp)
target_compile_features(health_monitor PUBLIC cxx_std_17)
ament_target_dependencies(health_monitor
  rclcpp
  std_msgs
  sensor_msgs
)

# tester code 실행 파일 만들기
add_executable(behavior_tree_tester src/behavior_tree_tester.cpp)
target_compile_features(behavior_tree_tester PUBLIC cxx_std_17)
ament_target_dependencies(behavior_tree_tester
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# 이 아래 add executable은 원래 있던놈.
add_executable(behavior_tree src/behavior_tree.cpp)
target_include_directories(behavior_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(behavior_tree PUBLIC cxx_std_17)  # Require C++17
ament_target_dependencies(behavior_tree
  rclcpp
)

# install
install(TARGETS 
  bt_nodes
  bt_main
  health_monitor
  behavior_tree_tester
  behavior_tree
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
) # install(TARGETS ...)는 colcon build는 "빌드"만 하고, ros2 run은 install 공간에 있는 실행파일을 찾아 실행합니다.
  # 따라서 install 폴더 안에 넣어주는 역할임!

install(DIRECTORY include/
  DESTINATION include/
)

# (나중에 launch 만들면 추가)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
