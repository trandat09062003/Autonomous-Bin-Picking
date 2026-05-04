# Autonomous Bin-Picking with Franka Panda & RLBench

Dự án này triển khai hệ thống gắp vật thể tự động (Bin-Picking) sử dụng cánh tay robot Franka Emika Panda trong môi trường mô phỏng RLBench. Hệ thống kết hợp Deep Learning cho thị giác máy tính và thuật toán lập kế hoạch quỹ đạo để tối ưu hóa quá trình gắp thả vật thể giữa các thùng chứa.

## 🌟 Các cải tiến chính so với bản gốc
So với phiên bản gốc, hệ thống này đã được tối ưu hóa đáng kể về độ tin cậy:
- **Cơ chế thử lại thông minh (Smart Retry):** Tự động thay đổi góc tiếp cận (Orientation) và độ sâu gắp (Depth) nếu lần gắp đầu tiên thất bại.
- **Xác thực đa tầng (Multi-layer Verification):** Kiểm tra trạng thái vật thể ngay sau khi kẹp và sau khi nhấc lên để đảm bảo không bị rơi giữa chừng.
- **Tối ưu hóa quỹ đạo:** Sử dụng RRT với tham số tinh chỉnh để tránh va chạm với thành thùng chứa.

---

## 🛠️ Quy trình hoạt động của dự án

### 1. Thu thập dữ liệu (Data Collection)
Hệ thống sử dụng file `data_collector.py` để tự động hóa việc thu thập dữ liệu huấn luyện:
- Robot di chuyển đến các vị trí quan sát phía trên thùng chứa.
- Camera gắn trên cổ tay (Wrist Camera) chụp ảnh RGB và ảnh độ sâu (Depth).
- Dữ liệu được gán nhãn tự động dựa trên trạng thái của thùng (trống hoặc có vật thể) để tạo Dataset cho mạng phân loại.

### 2. Tự học và Nhận diện (Learning & Perception)
Quá trình "tự học" của Agent diễn ra qua hai thành phần chính:
- **Phân loại trạng thái (Container Detection):** Sử dụng mạng **ResNet-18** (trong `object_detector.py`) được huấn luyện để nhận diện thùng chứa có trống hay không. Điều này giúp Agent quyết định khi nào cần gắp và khi nào hoàn thành nhiệm vụ.
- **Kế hoạch gắp (Grasp Planning):** Sử dụng **GQCNN 2.0** (Grasp Quality Convolutional Neural Network). Đây là một Agent đã được học từ hàng triệu mẫu gắp giả lập để dự đoán "Chất lượng điểm gắp" (Grasp Quality) từ ảnh độ sâu. Agent sẽ chọn điểm có điểm số cao nhất để thực hiện.

### 3. Thực thi (Execution)
- **Perception:** Agent quan sát hiện trường từ Wrist Camera.
- **Planning:** Nếu thùng không trống, GQCNN tính toán tọa độ gắp tối ưu.
- **Control:** Robot sử dụng thuật toán **RRT (Rapidly-exploring Random Tree)** để lập kế hoạch quỹ đạo di chuyển từ vị trí hiện tại đến điểm gắp mà không va chạm.
- **Robustness:** Sau khi gắp, hệ thống kiểm tra cảm biến lực/proximity để xác nhận vật đã nằm trong ngàm kẹp trước khi di chuyển sang thùng đích.

---

## 📁 Cấu trúc thư mục chính
- `auto_grasp.py`: Script chính điều khiển toàn bộ quy trình gắp thả tự động.
- `main.py`: Chứa lớp `GraspController` điều phối môi trường và robot.
- `grasp_planner.py`: Wrapper cho model GQCNN để dự đoán điểm gắp.
- `object_detector.py`: Định nghĩa và huấn luyện mạng ResNet phân loại thùng chứa.
- `data_collector.py`: Công cụ thu thập dữ liệu ảnh từ mô phỏng.
- `models/`: Chứa các trọng số đã huấn luyện (GQCNN, ResNet).

---

## 🚀 Hướng dẫn cài đặt

1. **Yêu cầu hệ thống:** Python 3.6, CoppeliaSim, PyRep, RLBench.
2. **Cài đặt dependencies:**
   ```bash
   pip install -r requirements.txt
   ```
3. **Cài đặt thư viện Perception & GQCNN:** Theo hướng dẫn từ Berkeley Automation Lab.
4. **Chạy hệ thống:**
   ```bash
   python auto_grasp.py
   ```

---
*Dự án được phát triển và tối ưu hóa bởi trandat09062003.*
