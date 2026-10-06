# UR3 Pick & Place điều khiển bằng LLM (Bài 03)

Hệ thống mô phỏng robot UR3 + gripper 2 ngón trong **Gazebo Classic 11**, lập quỹ đạo bằng **MoveIt 2**, nhận diện khối bằng **camera trên cao**. **LLM chỉ lập kế hoạch kỹ năng** (`pick`, `place`, `home`); kế hoạch được kiểm tra trước khi thực thi và không có kỹ năng nào dịch chuyển vật thể bằng phép "dịch chuyển tức thời".
## 1. Luồng xử lý

```
Lệnh người dùng ──► LLM (llm_planner.py) ──► kế hoạch JSON
                                              │
                          task_validator.py ◄─┘  (chỉ cho phép pick/place/home, vật & zone hợp lệ)
                                              │
 Camera ──► object_detector/perception ──► ZoneManager (trạng thái zone từ camera,
                                              chèn bước dọn khối khi zone đích bị chiếm)
                                              │
                                  skill_executor.py chạy từng bước
                                              │
                      robot_skills.py: pick / place / home
                                              │
                    MoveIt 2 (OMPL) + IK riêng (ur3_kin.py) ──► controller ──► UR3 + gripper
```

## 2. Cấu trúc package `ur3_llm_control`

| File | Vai trò |
|---|---|
| `llm_planner.py` | Prompt + gọi LLM (API tương thích OpenAI), trả về kế hoạch JSON. **Prompt nằm ở file này.** |
| `task_validator.py` | Kiểm tra kế hoạch: skill hợp lệ, vật thể/zone tồn tại. Kế hoạch sai bị từ chối, robot không chạy. |
| `object_detector.py`, `perception.py` | Nhận diện 5 khối theo màu (HSV), tính tọa độ từ `camera_info` + TF; publish `/detected_objects`. |
| `zone_manager.py` | Trạng thái zone từ camera; nếu zone đích đang có khối thì chèn "pick khối đó → place ra điểm tạm `park_*`". |
| `robot_skills.py` | Kỹ năng `pick`, `place`, `home`: đóng/mở gripper, kiểm tra kẹp, hạ/nhấc thẳng đứng, kiểm tra kết quả. |
| `ur3_kin.py` | FK/IK giải tích-số cho UR3 (hạ/nhấc thẳng chính xác). |
| `scene_publisher.py` | Đưa bàn/vật cản vào MoveIt planning scene. |
| `skill_executor.py` | Điều phối: nhận lệnh → LLM → validate → ZoneManager → thực thi → in kết quả. |
| `config/` | `student_config.yaml` (MSSV), `scene.yaml` (zone, điểm tạm, vị trí khối), controller yaml. |
| `launch/llm_robot.launch.py` | Mở Gazebo, robot, MoveIt, camera, controller. |
| `urdf/`, `worlds/` | Robot + gripper, thế giới Gazebo. |
| `scripts/analyze_contacts.py` | Phân tích lực tiếp xúc ngón–khối (dùng khi gỡ lỗi). |

Vị trí (frame `base_link`): zone_a (0.40, 0.15), zone_b (0.40, 0.00), zone_c (0.40, −0.15); điểm tạm park_1 (0.30, 0.30), park_2 (0.30, −0.30), park_3 (0.22, 0.37).

## 3. Cài đặt

Yêu cầu: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, MoveIt 2, `gazebo_ros2_control`, Python 3.10+, `pip install openai pyyaml`.

```bash
mkdir -p ~/ur3_llm_ws/src && cd ~/ur3_llm_ws/src
git clone https://github.com/Hiep-h/Ur3_LLM_Gripper.git ur3_llm_control
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git   # nhánh humble, dùng để giữ khối (xem mục 7)
cd ~/ur3_llm_ws
colcon build --packages-select linkattacher_msgs ros2_linkattacher ur3_llm_control --symlink-install
source install/setup.bash
```

Cấu hình LLM (API tương thích OpenAI; mặc định là gateway cục bộ):
```bash
export LLM_BASE_URL=http://localhost:20128/v1   # hoặc https://api.openai.com/v1 ...
export LLM_API_KEY=<khoá của bạn>               # KHÔNG commit khoá lên repo
export LLM_MODEL=<tên model>
```

## 4. Chạy

**Terminal 1 – mô phỏng (chờ Gazebo + MoveIt lên hẳn, ~30 s):**
```bash
cd ~/ur3_llm_ws && source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

**Terminal 2 – nhập lệnh tương tác:**
```bash
cd ~/ur3_llm_ws && source install/setup.bash
ros2 run ur3_llm_control skill_executor --ros-args -p use_sim_time:=true
```
Gõ lệnh ở dấu nhắc, `exit` để thoát. Hoặc chạy một lệnh rồi thoát:
```bash
ros2 run ur3_llm_control skill_executor --ros-args -p use_sim_time:=true -p command:="Put the red cube in Zone B."
```
Kiểm tra riêng robot (không qua LLM): `-p test_skill:=home` (hoặc `open`, `close`).

> Chỉ chạy **một** bản launch. Nếu chạy lại, tắt hết tiến trình cũ trước: `pkill -9 -f "gzb|gzclient|gzserver|move_group|ros2"`.

## 5. Kịch bản kiểm thử theo đề

**Demo bắt buộc – không thể pick→place trực tiếp:** zone_b đang có `blue_cube`.
```
Put the red cube in Zone B.
```
Luồng: camera thấy zone_b bị chiếm → chọn điểm tạm trống → pick blue → place park → pick red → place zone_b → home.

Đầu ra mẫu:
```
USER COMMAND: Put the red cube in Zone B.
LLM PLAN:
1. pick(red_cube)
2. place(red_cube, zone_b)
3. home()
RESOLVED PLAN (ZoneManager bo sung buoc don):
1. pick(blue_cube)
2. place(blue_cube, park_1)
3. pick(red_cube)
4. place(red_cube, zone_b)
5. home()
EXECUTION:
pick(blue_cube) ............ SUCCESS
place(blue_cube, park_1) ... SUCCESS
pick(red_cube) ............. SUCCESS
place(red_cube, zone_b) .... SUCCESS
home() ..................... SUCCESS
TASK SUCCESS
```

**Mức nâng cao – sắp xếp theo MSSV:**
```
Arrange all objects according to my student ID.
```
Kết quả: red→zone_a, yellow→zone_b, blue→zone_c; green và purple không có zone nên được đưa tới điểm tạm `temp` (park_1..3). Khi zone đích đang bị chiếm, ZoneManager tự chèn bước dọn khối sang điểm tạm.

**Kế hoạch không hợp lệ** (robot không chuyển động, báo `VALIDATION FAILED` / `TASK FAILED`):
```
Put the orange cube in Zone D.
```

Đã chạy trong mô phỏng: demo Zone B thành công (khối xanh → park_1 lệch ~0.2 cm, khối đỏ → zone_b lệch ~0.1 cm); sắp xếp theo MSSV thành công; lệnh sai bị từ chối; `test_skill:=home` thành công. Mỗi pick/place mất ~55 s thời gian thực.

## 6. Quy tắc an toàn đã cài
- LLM chỉ được chọn 3 skill; kế hoạch bị `task_validator` kiểm tra trước khi chạy.
- Trạng thái khối/zone lấy từ **camera**, không dùng toạ độ đặt cứng khi quyết định.
- `pick` chỉ tiếp tục khi độ mở hai ngón ≈ bề rộng khối (kẹp được vật); nếu sai số vị trí tool > 2 cm thì dừng với `FAILED`.
- Gặp bước lỗi thì dừng, in `TASK FAILED` và về home.

## 7. Giới hạn đã biết
- Giữ khối chỉ bằng ma sát (không attach) **không** thành công trong Gazebo Classic ở cấu hình này: ngón kẹp đúng bề rộng nhưng khối trượt khi nhấc, dù đã thử gain lực, ma sát 5–100, đường thẳng, tham số solver ODE và đo lực tiếp xúc (`scripts/analyze_contacts.py`). Vì vậy sau khi **xác nhận kẹp bằng gripper**, khối được gắn bằng fixed joint của IFRA LinkAttacher (đặt `UR3_USE_ATTACH=0` để tắt). Gripper vẫn đóng/mở và kẹp được kiểm tra.
- Kiểm tra vị trí sau khi nhấc/thả dùng `/gazebo/model_states` (chỉ có trong mô phỏng, chỉ để xác minh kết quả, không dùng để lập kế hoạch).
