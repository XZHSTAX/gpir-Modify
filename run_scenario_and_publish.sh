#!/bin/zsh

# 切换到scenario_runner目录并在后台运行python脚本
echo "Starting scenario runner..."
cd /home/xzh2/code/scenario_runner_XZH
python scenario_runner.py --scenario XZHScenario_4 --timeout 1000 --waitForEgo --output --randomize &

# 获取python进程的PID
PYTHON_PID=$!
echo "Scenario runner started with PID: $PYTHON_PID"

# 等待5秒让python脚本初始化
echo "Waiting 5 seconds for scenario runner to initialize..."
sleep 7
echo "Wait completed, proceeding with ROS operations..."

# 切换到ROS工作空间
echo "Switching to ROS workspace..."
cd /home/xzh2/ros1/gpir_Modify

# Source ROS环境
echo "Sourcing ROS environment..."
source devel/setup.zsh

# 发布PoseStamped话题
echo "Publishing PoseStamped message..."
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{
  header: {
    seq: 0,
    stamp: 'now',
    frame_id: 'map'
  },
  pose: {
    position: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }
  }
}" --once

echo "PoseStamped message published successfully"

# 等待python脚本结束
echo "Waiting for scenario runner to complete..."
wait $PYTHON_PID
echo "Scenario runner completed with exit code: $?"

echo "Script execution completed"