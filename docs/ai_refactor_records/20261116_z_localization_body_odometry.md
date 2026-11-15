# Z-Localization 机器人 Odometry 改造记录

## User Intent
在 Z-Localization 后端增加机器人 Odometry。用户确认 pose 与 twist 均在 map 系表达，保持 navigation 和原有输出不变。

## Scope
后端 ROS Publisher、接口说明、现有 ROS 集成测试。

## Out of Scope
不修改 navigation、机械外参、既有 TF、定位算法、launch、QoS 默认配置；不构建、不提交 Git。

## Explorer Findings
### Files inspected
后端 publisher.hpp/publisher.cpp/publisher_legacy.cpp/publisher_state.cpp、body.cpp、covariance.hpp、test_map_service.py、README.md；导航 ros2_node.cpp 的速度输入约定。
### Active logic path
publishPrediction 在 publication_mutex_ 内核验 projectionIsCurrent 后发布预测输出。legacy 路径已有车体外参与世界系杆臂速度计算。
### Data flow
前端传感器 Odometry + 后端 map 投影 → common.body 外参 → 车体位姿、map 系速度 → /localization/body_odometry。
### Risk notes
新接口 twist 在 header/map 系而非 child/body 系，是已确认的导航专用约定。停发不等于停车，导航消费者仍需超时与可信状态门控。速度协方差条件于当前姿态与固定外参，不额外计入姿态/外参误差。
### Recommended modification boundary
复用现有 Publisher 和车体速度逻辑，不增加适配节点，不改变原有传感器里程计。

## Modifier Changes
### Files changed
- src/location/z-localization/src/backend/apps/publisher.hpp
- src/location/z-localization/src/backend/apps/publisher.cpp
- src/location/z-localization/src/backend/apps/publisher_legacy.cpp
- src/location/z-localization/src/backend/README.md
- src/location/z-localization/src/backend/test/ros/test_map_service.py
- 本记录。
### Key changes
新增 reliable/volatile depth 200 机器人 Odometry，使用 common.map_frame/common.body.frame 和前端时间戳。只在可信且全局位姿可用时发布，与 legacy 开关无关。将原有速度计算提取为共享函数，原兼容速度发布复用该函数。位姿协方差传播到车体参考点；速度协方差通过旋转和杆臂 Jacobian 传播，保留交叉项。
### Behavior preserved
原 /localization/odometry、兼容 pose/twist/path/danger、TF 的开关和语义保持不变。navigation 未修改。
### Behavior intentionally adjusted
增加新话题；初始化或不可信状态不发布新话题。
### Notes
任务开始时已有根 README、navigation 子模块和 tf 约定文档的工作区变更；未触碰。根 AGENTS.md 在执行期间由工作区新增并已读取，遵守其不构建和不提交限制。首次 ls 检查因当时不存在根/中间 AGENTS.md 返回 2，非构建失败。

## Auditor Review
单人静态自查，未使用多 Agent，不宣称独立审计。
### Checks performed
- [x] 发布调用链与 legacy 行为差异核对
- [x] 任务范围 diff 检查
- [x] git diff --check -- src/location/z-localization：退出 0
- [x] Python ast.parse：新增测试语法通过
- [x] 独立纯 Python 数值检查：yaw=90°、非零杆臂时速度 (-1,2,0)、速度协方差对角/交叉项符合预期
- [ ] C++ 编译及 ROS 集成测试执行（未授权构建）

ROS 测试已增加初始化不发布、legacy 关闭仍发布、frame/时间戳、非零 yaw/杆臂、速度协方差、原传感器速度未改变的断言；未运行。SUSPECT/LOST 的实际 ROS 状态转移停发未动态覆盖，只检查实现门控条件。

### Issues found
未发现静态阻断问题，但静态检查不能证明 C++ 编译和真实 ROS 发布正确。
### Final result
PASS（仅静态检查）；整体运行验收 needs-verification。未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
