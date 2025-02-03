import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import acconeer.exptool as et
import numpy as np
import json


class RadarPublisher(Node):
    def __init__(self):
        super().__init__('radar_publisher')

        # Publisher pro data a metadata
        self.amplitude_publisher = self.create_publisher(Float32MultiArray, 'radar_amplitude', 10)
        self.metadata_publisher = self.create_publisher(String, 'radar_metadata', 1)

        # Načtení parametrů
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('range_interval', [0.24, 1.2])
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('service_type', 'sparse')  # envelope', 'iq', nebo 'sparse'
        self.declare_parameter('sweeps_per_frame', 16)
        self.declare_parameter('measurement_info', 'Radar measurement in progress')

        # Získání parametrů
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        range_interval = self.get_parameter('range_interval').get_parameter_value().double_array_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        service_type = self.get_parameter('service_type').get_parameter_value().string_value
        sweeps_per_frame = self.get_parameter('sweeps_per_frame').get_parameter_value().integer_value
        measurement_info = self.get_parameter('measurement_info').get_parameter_value().string_value

        # Inicializace klienta
        self.get_logger().info(f"Spouštím radar na portu {serial_port} s typem služby '{service_type}'...")
        args = et.a111.ExampleArgumentParser().parse_args(["-u", serial_port])
        self.client = et.a111.Client(**et.a111.get_client_args(args))

        # Konfigurace služby
        if service_type == 'envelope':
            self.config = et.a111.EnvelopeServiceConfig()
        elif service_type == 'iq':
            self.config = et.a111.IQServiceConfig()
            self.config.profile = self.config.Profile.PROFILE_2
        elif service_type == 'sparse':
            self.config = et.a111.SparseServiceConfig()
            self.config.sweeps_per_frame = sweeps_per_frame
            self.config.profile = self.config.Profile.PROFILE_3
        else:
            self.get_logger().error("Neplatný typ služby! Použij 'envelope', 'iq' nebo 'sparse'.")
            exit(1)

        self.config.sensor = [1]
        self.config.range_interval = range_interval
        self.config.update_rate = update_rate

        # Připojení klienta
        self.client.connect()
        session_info = self.client.setup_session(self.config)
        self.get_logger().info(f"Session info:\n{session_info}\n")

        # Publikování metadat
        metadata = {
            "service_type": service_type,
            "serial_port": serial_port,
            "range_interval": range_interval,
            "update_rate": update_rate,
            "sweeps_per_frame": sweeps_per_frame if service_type == 'sparse' else None,
            "measurement_info": measurement_info,
            "session_info": str(session_info),
        }
        metadata_msg = String()
        metadata_msg.data = json.dumps(metadata)
        self.metadata_publisher.publish(metadata_msg)
        self.get_logger().info("Published radar metadata.")

        # Zahájení měření
        self.client.start_session()
        self.timer = self.create_timer(1.0 / update_rate, self.read_and_publish_data)

    def read_and_publish_data(self):
        try:
            _, data = self.client.get_next()

            if isinstance(self.config, et.a111.IQServiceConfig):
                amplitude = np.abs(data)
                msg_data = amplitude.flatten().tolist()
            elif isinstance(self.config, et.a111.SparseServiceConfig):
                msg_data = data.flatten().tolist()
            else:  # Envelope service
                msg_data = data.tolist()

            # Publikace dat
            msg = Float32MultiArray()
            msg.data = msg_data
            self.amplitude_publisher.publish(msg)

            self.get_logger().info(f"Published radar data of type '{type(self.config).__name__}'.")

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
