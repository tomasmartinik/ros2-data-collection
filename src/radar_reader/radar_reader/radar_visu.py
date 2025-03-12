import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from collections import deque


class RadarVisu(Node):
    def __init__(self):
        super().__init__('radar_visualisation_live')

        # Subscriptions
        self.radar_subscriber = self.create_subscription(Float32MultiArray, 'radar_data', self.radar_callback, 10)
        self.image_subscriber = self.create_subscription(CompressedImage, 'image_raw/compressed', self.image_callback, 10)
        self.session_info_subscriber = self.create_subscription(String, 'session_info', self.session_info_callback, 10)
        


        # Parameters
        self.history_duration = 5.0  # History length (seconds)
        self.update_interval = 0.05  # Update interval (seconds)
        self.num_frames_to_keep = int(self.history_duration / self.update_interval)

        # Buffers
        self.radar_frames = deque(maxlen=self.num_frames_to_keep)
        self.image_frames = deque(maxlen=self.num_frames_to_keep)

        # 🔹 **Otevřeme okna až po detekci dat**
        self.data_type = None  # Zatím nevíme, jestli Sparse nebo Envelope

        self.session_info_text = "Waiting for session info..."


        # Timer for updating the visualization
        self.create_timer(self.update_interval, self.display_live_data)

    def radar_callback(self, msg):
        """Processes incoming radar data and detects type (sparse/envelope)."""
        frame = np.array(msg.data, dtype=np.float32)

        if frame.size < 2:
            self.get_logger().error(f"Radar frame too small! Size: {frame.size}")
            return

        #cv2.namedWindow("Session Info", cv2.WINDOW_NORMAL)

        # 🔹 **Detekce typu dat při první zprávě**
        if self.data_type is None:
            sweeps_per_frame = int(frame[0]) if frame[0] > 0 and frame[0] < 100 else None

            if sweeps_per_frame in [16, 32]:  # 🔹 Pokud odpovídá sparse formátu
                self.data_type = "sparse"
                cv2.namedWindow("Radar Heatmap", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Radar Variance Heatmap", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Radar Sweeps", cv2.WINDOW_NORMAL)
            else:
                self.data_type = "envelope"
                cv2.namedWindow("Radar Envelope", cv2.WINDOW_NORMAL)  # ✅ Otevřeme jen pro envelope

            cv2.namedWindow("Live Video", cv2.WINDOW_NORMAL)  # 📹 Společné okno pro video

            self.get_logger().info(f"Detected radar data type: {self.data_type}")

        # 🔹 **Uložení dat podle typu**
        if self.data_type == "sparse":
            sweeps_per_frame = int(frame[0])  # První hodnota určuje počet sweepů
            frame = frame[1:]

            if frame.size % sweeps_per_frame != 0:
                self.get_logger().error(f"Incorrect sparse data size: {frame.size} not divisible by {sweeps_per_frame}")
                return

            points_per_sweep = frame.size // sweeps_per_frame
            reshaped_frame = frame.reshape((sweeps_per_frame, points_per_sweep))
            self.radar_frames.append(reshaped_frame)

        else:  # 🔹 Envelope data
            self.radar_frames.append(frame)

    def session_info_callback(self, msg):
        """Receives session info and updates the text display."""
        self.session_info_text = msg.data  # Uložíme přijatý JSON string


    def image_callback(self, msg):
        """Processes incoming image data"""
        img_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Detect image format (JPEG/PNG)
        start_index = img_data.tobytes().find(b'\xff\xd8')  # JPEG header
        if start_index == -1:
            start_index = img_data.tobytes().find(b'\x89PNG')  # PNG header

        if start_index != -1:
            img_data_clean = img_data[start_index:]
            image = cv2.imdecode(img_data_clean, cv2.IMREAD_COLOR)

            if image is not None:
                self.image_frames.append(image)

    def display_live_data(self):
        """Updates OpenCV windows based on detected data type."""
        if not self.radar_frames:
            return

        last_radar_idx = len(self.radar_frames) - 1
        last_image_idx = len(self.image_frames) - 1 if self.image_frames else -1

        info_img = np.ones((300, 600, 3), dtype=np.uint8) * 255  # Bílé pozadí
        y0, dy = 20, 25  # Počáteční pozice textu

        for i, line in enumerate(self.session_info_text.split("\n")):
            cv2.putText(info_img, line, (10, y0 + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # cv2.imshow("Session Info", info_img)


        # 🔹 **Rozhodnutí podle typu dat**
        if self.data_type == "sparse":
            # --- Radar Heatmap (Scrolling) ---
            heatmap_data = np.array([frame.mean(axis=0) for frame in self.radar_frames])

            if heatmap_data.shape[0] > self.num_frames_to_keep:
                heatmap_data = heatmap_data[-self.num_frames_to_keep:, :]

            heatmap_norm = cv2.normalize(heatmap_data.T[::-1], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            heatmap_color = cv2.applyColorMap(heatmap_norm, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Radar Heatmap", heatmap_color)

            # --- Radar Variance Heatmap ---
            variance_data = np.array([np.var(frame, axis=0) for frame in self.radar_frames])

            if variance_data.shape[0] > self.num_frames_to_keep:
                variance_data = variance_data[-self.num_frames_to_keep:, :]

            variance_norm = cv2.normalize(variance_data.T[::-1], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            variance_color = cv2.applyColorMap(variance_norm, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Radar Variance Heatmap", variance_color)

             # --- Radar Sweeps (jeden okamžik) ---
            if last_radar_idx >= 0:
                current_sweeps = self.radar_frames[last_radar_idx]
                sweep_img = np.zeros((300, 600, 3), dtype=np.uint8)

                num_sweeps, points_per_sweep = current_sweeps.shape
                sweep_min, sweep_max = 0, 60000
                normalized_sweeps = np.clip(current_sweeps, sweep_min, sweep_max)

                for i, sweep in enumerate(normalized_sweeps):
                    x = np.linspace(50, 550, points_per_sweep).astype(np.int32)
                    y = 280 - ((sweep - sweep_min) / (sweep_max - sweep_min) * 250).astype(np.int32)

                    color = (255 - int(i * 255 / num_sweeps), 255, int(i * 255 / num_sweeps))
                    for j in range(len(x)):
                        cv2.circle(sweep_img, (x[j], y[j]), 2, color, -1)

                cv2.imshow("Radar Sweeps", sweep_img)


        else:
            # --- Envelope Data Heatmap ---
            envelope_data = np.array(self.radar_frames)

            if envelope_data.shape[0] > self.num_frames_to_keep:
                envelope_data = envelope_data[-self.num_frames_to_keep:, :]

            envelope_norm = cv2.normalize(envelope_data.T[::-1], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            envelope_color = cv2.applyColorMap(envelope_norm, cv2.COLORMAP_VIRIDIS)
            envelope_color = cv2.resize(envelope_color, (600, 300))
            cv2.imshow("Radar Envelope", envelope_color)

        # --- Live Video ---
        if last_image_idx >= 0:
            cv2.imshow("Live Video", self.image_frames[last_image_idx])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = RadarVisu()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down radar visualization node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
