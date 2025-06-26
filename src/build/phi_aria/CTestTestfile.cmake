# CMake generated Testfile for 
# Source directory: /home/jumombach/ros2_ws/src/phi_aria
# Build directory: /home/jumombach/ros2_ws/src/build/phi_aria
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/cppcheck.xunit.xml" "--package-name" "phi_aria" "--output-file" "/home/jumombach/ros2_ws/src/build/phi_aria/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/cppcheck.xunit.xml" "--include_dirs" "/home/jumombach/ros2_ws/src/phi_aria/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/jumombach/ros2_ws/src/phi_aria")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/lint_cmake.xunit.xml" "--package-name" "phi_aria" "--output-file" "/home/jumombach/ros2_ws/src/build/phi_aria/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/jumombach/ros2_ws/src/phi_aria")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/uncrustify.xunit.xml" "--package-name" "phi_aria" "--output-file" "/home/jumombach/ros2_ws/src/build/phi_aria/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/jumombach/ros2_ws/src/phi_aria")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/xmllint.xunit.xml" "--package-name" "phi_aria" "--output-file" "/home/jumombach/ros2_ws/src/build/phi_aria/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/jumombach/ros2_ws/src/build/phi_aria/test_results/phi_aria/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/jumombach/ros2_ws/src/phi_aria")
