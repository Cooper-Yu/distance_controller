# Task6 精确起点位置容差（2026-10-25）

用户本次授权：初始化位置容差可放宽至1cm。此变更只作用于precise_start的右/后距位置门，不修改普通航点容差、路线、速度、航向、反馈失鲜、4cm、微调守卫、初始化预算，也不增加P02动态修正。

## 参数与兼容

- ROS参数precise_start_position_tolerance：double，默认0.005m，必须finite且在[0.005,0.010]；非法值在初始化配置阶段抛错。
- precise_start=true时，centering_tolerance_=min(既有centering_tolerance, precise_start_position_tolerance)。既有默认centering_tolerance为0.01m，更严格的显式设置仍然保留。
- heading_tolerance仍为min(既有值,0.008726646rad)，centering_max_speed仍保留0.02m/s上限；其它参数保持原实现。
- task6_route_runner新增--start-position-tolerance，默认0.005m，同范围finite校验。请求1cm需显式传--start-position-tolerance 0.010。
- prepare_and_run在supervised参数后新增尾部start_position_tolerance=0.005，直接调用也校验后才操作runner；session使用getattr(args,'start_position_tolerance',0.005)兼容旧SimpleNamespace。
- prepare_options追加-p precise_start_position_tolerance:=<value>；preparation_started审计字段precise_start_position_tolerance保存请求值。有效容差仍受更严格既有centering_tolerance限制，不能把请求值冒称所有场景的实际门槛。

## 本地检查与边界

Ubuntu-22.04，仓库/home/cooper/ros2_ws/src/distance_controller。

已实际执行并通过：

- ruff check tools/task6_route_runner.py
- ruff format --check tools/task6_route_runner.py
- clang-format --dry-run --Werror src/initial_centering.cpp
- python3 -m py_compile tools/task6_route_runner.py
- git diff --check

只对修改文件格式化；未批量改其它源码。初次clang-format检查提示新增片段布局，按新增行范围格式化后复验通过。

构建由Main统一执行，避免共享workspace并发colcon；独立边界/真实C++隔离验证由B负责，不在本文预先声明通过。此修改不构成真机初始化通过或Task6路线验收。未stage/commit/push、未云端部署、未操作机器人。