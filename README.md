# UR3 Robotic Arm Control with LLM Planning & ROS 2

Dự án điều khiển cánh tay robot công nghiệp Universal Robots UR3 trong môi trường mô phỏng Gazebo Classic và MoveIt 2, sử dụng Large Language Model (LLM) để lập kế hoạch tác vụ từ câu lệnh ngôn ngữ tự nhiên (tiếng Việt / tiếng Anh).

## Tính năng chính
- **Natural Language Task Planning:** Tự động giải mã câu lệnh ngôn ngữ tự nhiên tiếng Việt/Anh thành chuỗi kỹ năng thao tác (pick, place, home).
- **Conflict Resolution (Deadlock Handling):** Tự động phát hiện và giải quyết xung đột khi vùng đích đã bị chiếm bằng cách dọn tạm vật thể sang điểm đỗ tạm (park_1..3).
- **Motion Planning & Collision Avoidance:** Quy hoạch quỹ đạo qua MoveIt 2 (OMPL RRTConnect), đảm bảo an toàn động học và giới hạn khớp.
- **Gazebo Dynamic Visual Sync:** Đồng bộ vị trí vật thể theo tay gắp ảo trong điều kiện mô phỏng thời gian thực.

## Yêu cầu hệ thống
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Classic 11 & MoveIt 2
- Python 3.10+

## Cài đặt & Build
```bash
cd ~/ur3_llm_ws/src
git clone https://github.com/Hiep-h/Ur3_LLM.git

cd ~/ur3_llm_ws
colcon build --packages-select ur3_llm_control
source install/setup.bash
```

## Khởi chạy
**Terminal 1 (Mô phỏng Gazebo & MoveIt 2):**
```bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

**Terminal 2 (Bộ điều phối tác vụ LLM):**
```bash
source install/setup.bash
ros2 run ur3_llm_control skill_executor --ros-args -p use_sim_time:=true
```

## Ví dụ câu lệnh
- di chuyen khoi mau do toi zone a
- sap xep cac khoi ve dung zone
- Arrange all objects according to my student ID.
