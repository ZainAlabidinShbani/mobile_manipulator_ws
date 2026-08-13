---
trigger: always_on
---

1. Workspace root is always ~/mobile_manipulator_ws.
2. NEVER use the word 'husky' in package names or variables. Use 'mobile_manipulator_*'.
3. Always verify generated ROS 2 nodes and C++/Python packages with 'colcon build'.
4. Shell is Zsh on ROS 2 Humble (Ubuntu 22.04 LTS).
5. When modifying code, always update the relevant CMakeLists.txt and package.xml dependencies.