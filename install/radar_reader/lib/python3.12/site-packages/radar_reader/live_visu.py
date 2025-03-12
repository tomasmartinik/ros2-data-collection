import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribe to the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Change this if your topic is different
            self.image_callback,
            10
        )

        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        """ Callback function to process and display the received image """
        try:
            # Convert message data to numpy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8)

            # Search for the JPEG/PNG header in case of metadata
            start_index = img_data.tobytes().find(b'\xff\xd8')  # JPEG header
            if start_index == -1:
                start_index = img_data.tobytes().find(b'\x89PNG')  # PNG header

            if start_index != -1:
                img_data_clean = img_data[start_index:]  # Extract the actual image
                frame = cv2.imdecode(img_data_clean, cv2.IMREAD_COLOR)  # Decode image

                if frame is not None:
                    cv2.imshow("Live Video Stream", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
                        rclpy.shutdown()
            else:
                self.get_logger().warn("No valid image header found!")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
