import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import acconeer.exptool as et


class RadarPublisher(Node):
    def __init__(self):
        super().__init__('radar_publisher')

        # Vytvoření publisheru na topic 'radar_data'
        self.publisher_ = self.create_publisher(Float32MultiArray, 'radar_data', 10)

        # Inicializace radaru
        self.get_logger().info("Spouštím radar s pevnými parametry...")
        args = et.a111.ExampleArgumentParser().parse_args(["-u", "/dev/ttyUSB0"])
        self.client = et.a111.Client(**et.a111.get_client_args(args))

        self.config = et.a111.EnvelopeServiceConfig()
        self.config.sensor = [1]
        self.config.range_interval = [0.2, 0.3]
        self.config.update_rate = 10

        self.client.connect()
        session_info = self.client.setup_session(self.config)
        self.get_logger().info(f"Session info:\n{session_info}\n")

        self.client.start_session()

        # Nastavení timeru pro periodické čtení dat
        self.timer = self.create_timer(1.0 / self.config.update_rate, self.read_and_publish_data)

    def read_and_publish_data(self):
        try:
            data_info, data = self.client.get_next()
            msg = Float32MultiArray()
            msg.data = data.tolist()
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published radar data: {data}")
        except Exception as e:
            self.get_logger().error(f"Error reading radar data: {e}")

    def destroy_node(self):
        self.client.stop_session()
        self.client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    radar_publisher = RadarPublisher()

    try:
        rclpy.spin(radar_publisher)
    except KeyboardInterrupt:
        radar_publisher.get_logger().info("Ukončuji radar publisher...")
    finally:
        radar_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
