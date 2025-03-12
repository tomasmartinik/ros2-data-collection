import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
import numpy as np
import matplotlib.pyplot as plt
import cv2
from collections import deque


class RadarVisu(Node):
    def __init__(self):
        super().__init__('radar_visualisation_live')

        # Subscribers for radar and image data
        self.radar_subscriber = self.create_subscription(
            Float32MultiArray, 'radar_data', self.radar_callback, 10)

        self.image_subscriber = self.create_subscription(
            CompressedImage, 'image_raw/compressed', self.image_callback, 10)

        # Parameters
        self.history_duration = 5.0  # History length in seconds
        self.update_interval = 0.05  # Graph update interval (seconds)
        self.num_frames_to_keep = int(self.history_duration / self.update_interval)

        # Buffers for radar and image frames
        self.radar_frames = deque(maxlen=self.num_frames_to_keep)
        self.image_frames = deque(maxlen=self.num_frames_to_keep)

        # Initialize Matplotlib figure
        self.fig, (self.ax_movement, self.ax_radar, self.ax_scalogram, self.ax_image) = plt.subplots(4, 1, figsize=(14, 16))
        plt.subplots_adjust(bottom=0.1, hspace=0.4)
        self.init_graphs()

        # Timer for updating the visualization
        self.create_timer(self.update_interval, self.display_live_data)

    def init_graphs(self):
        """Initialize graphs"""
        self.ax_movement.set_title('Movement Variance')
        self.ax_radar.set_title('Current Radar Frame')
        self.ax_scalogram.set_title('Radar History (Average)')
        self.ax_image.set_title('Live Video Frame')
        self.ax_image.axis('off')

        # Initialize empty plots
        self.movement_img = self.ax_movement.imshow(np.zeros((10, 10)), cmap='inferno', aspect='auto', origin='lower')
        self.radar_scatter = self.ax_radar.scatter([], [])
        self.scalogram_img = self.ax_scalogram.imshow(np.zeros((10, 10)), cmap='viridis', aspect='auto', origin='lower')
        self.image_display = self.ax_image.imshow(np.zeros((100, 100, 3), dtype=np.uint8))

    def radar_callback(self, msg):
        """Processes incoming radar data"""
        frame = np.array(msg.data, dtype=np.float32)

        if frame.size < 2:
            self.get_logger().error(f"Radar frame too small! Size: {frame.size}")
            return

        # Extract sweeps_per_frame if present
        sweeps_per_frame = int(frame[0]) if frame[0] > 0 and frame[0] < 100 else 32  # Reasonable default
        frame = frame[1:]

        if frame.size % sweeps_per_frame != 0:
            self.get_logger().error(f"Incorrect data size: {frame.size} not divisible by {sweeps_per_frame}")
            return

        points_per_sweep = frame.size // sweeps_per_frame
        reshaped_frame = frame.reshape((sweeps_per_frame, points_per_sweep))
        self.radar_frames.append(reshaped_frame)

    def image_callback(self, msg):
        """Processes incoming image data"""
        img_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Try to detect the image header (JPEG or PNG)
        start_index = img_data.tobytes().find(b'\xff\xd8')  # JPEG header
        if start_index == -1:
            start_index = img_data.tobytes().find(b'\x89PNG')  # PNG header

        if start_index != -1:
            img_data_clean = img_data[start_index:]  # Remove metadata
            image = cv2.imdecode(img_data_clean, cv2.IMREAD_COLOR)

            if image is not None:
                self.image_frames.append(image)

    def display_live_data(self):
        """Updates the plots in real-time"""
        if not self.radar_frames:
            return

        last_radar_idx = len(self.radar_frames) - 1
        last_image_idx = len(self.image_frames) - 1 if self.image_frames else -1

        # --- 1️⃣ Movement Variance Heatmap ---
        if len(self.radar_frames) > 1:
            movement_data = np.var(np.array(self.radar_frames), axis=1)

            # Scroll by shifting left: old data is removed, new data is added
            if movement_data.shape[0] > self.num_frames_to_keep:
                movement_data = movement_data[-self.num_frames_to_keep:, :]

            self.movement_img.set_data(movement_data.T)
            self.ax_movement.set_xlim(0, movement_data.shape[0])

        # --- 2️⃣ Radar Frame (Fixed Scale) ---
        self.ax_radar.clear()
        current_radar_frame = self.radar_frames[last_radar_idx]

        for sweep in current_radar_frame:
            self.ax_radar.scatter(np.arange(sweep.size), sweep, color='blue', s=1)

        self.ax_radar.set_title(f'Current Radar Frame {last_radar_idx + 1}')
        self.ax_radar.set_xlim(0, current_radar_frame.shape[1] - 1)
        self.ax_radar.set_ylim(0, 60000)  # 🔹 Fixed scale to prevent auto-scaling

        # --- 3️⃣ Radar Scalogram Heatmap (Live Scrolling) ---
        scalogram_data = np.array([frame.mean(axis=0) for frame in self.radar_frames])

        # Scroll by shifting left: old data is removed, new data is added
        if scalogram_data.shape[0] > self.num_frames_to_keep:
            scalogram_data = scalogram_data[-self.num_frames_to_keep:, :]

        self.scalogram_img.set_data(scalogram_data.T)
        self.ax_scalogram.set_xlim(0, scalogram_data.shape[0])

        # --- 4️⃣ Video Frame ---
        if last_image_idx >= 0:
            self.image_display.set_data(cv2.cvtColor(self.image_frames[last_image_idx], cv2.COLOR_BGR2RGB))

        plt.pause(0.001)  # Small delay for smoother updates


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


if __name__ == '__main__':
    main()
