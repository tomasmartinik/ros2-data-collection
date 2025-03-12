import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import acconeer.exptool as et
import numpy as np
import json

class RadarPublisher(Node):
    def __init__(self):
        super().__init__('radar_publisher')

        # Publisher for data
        self.data_publisher = self.create_publisher(Float32MultiArray, 'radar_data', 10)
        #self.metadata_publisher = self.create_publisher(String, 'radar_metadata', 1)

        # nastveni qos proflu, abycho mohli pouzit transient local - subscriber, ktery se pripoji pozde, obdrzi "sticky" message
        qos_profile_tl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.session_info_publisher = self.create_publisher(String, 'radar_session_info', qos_profile_tl)


        # Default parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('service_type', "envelope")  # envelope' nebo 'sparse'
        self.declare_parameter('range_interval', [0.24, 0.9])
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('profile', "PROFILE_3")
        self.declare_parameter('mur', "MUR_6")
        self.declare_parameter('running_average_factor', 0.0)
        self.declare_parameter('repetition_mode', "HOST_DRIVEN")
        self.declare_parameter('downsampling_factor', 1)
        self.declare_parameter('hw_accelerated_average_samples', 10)
        self.declare_parameter('gain', 0.5)
        self.declare_parameter('maximize_signal_attenuation', False)
        self.declare_parameter('noise_level_normalization', True)
        self.declare_parameter('tx_disable', False)
        self.declare_parameter('power_save_mode', "ACTIVE")
        self.declare_parameter('asynchronous_measurement', True)
        self.declare_parameter('sweeps_per_frame', 32)

        self.declare_parameter('measurement_info', 'Radar measurement in progress')

        # Get launchfile parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        range_interval = self.get_parameter('range_interval').get_parameter_value().double_array_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        service_type_short = self.get_parameter('service_type').get_parameter_value().string_value.lower() 
        service_type = "envelope" if service_type_short == "e" else "sparse" if service_type_short == "s" else service_type_short
        profile = self.get_parameter('profile').get_parameter_value().string_value
        mur = self.get_parameter('mur').get_parameter_value().string_value
        running_average_factor = self.get_parameter('running_average_factor').get_parameter_value().double_value
        repetition_mode = self.get_parameter('repetition_mode').get_parameter_value().string_value
        downsampling_factor = self.get_parameter('downsampling_factor').get_parameter_value().integer_value
        hw_accelerated_average_samples = self.get_parameter('hw_accelerated_average_samples').get_parameter_value().integer_value
        gain = self.get_parameter('gain').get_parameter_value().double_value
        maximize_signal_attenuation = self.get_parameter('maximize_signal_attenuation').get_parameter_value().bool_value
        noise_level_normalization = self.get_parameter('noise_level_normalization').get_parameter_value().bool_value
        tx_disable = self.get_parameter('tx_disable').get_parameter_value().bool_value
        power_save_mode = self.get_parameter('power_save_mode').get_parameter_value().string_value
        asynchronous_measurement = self.get_parameter('asynchronous_measurement').get_parameter_value().bool_value
        sweeps_per_frame = self.get_parameter('sweeps_per_frame').get_parameter_value().integer_value


        measurement_info = self.get_parameter('measurement_info').get_parameter_value().string_value

        # Inicializace klienta
        self.get_logger().info(f"Spouštím radar na portu {serial_port} s typem služby '{service_type}'...,")
        args = et.a111.ExampleArgumentParser().parse_args(["-u", serial_port])
        self.client = et.a111.Client(**et.a111.get_client_args(args))

        # Config
        if service_type == "sparse":
            self.config = et.a111.SparseServiceConfig()
            self.config.sweeps_per_frame = sweeps_per_frame
        elif service_type == "envelope":
            self.config = et.a111.EnvelopeServiceConfig() 
            self.config.running_average_factor = running_average_factor
            self.config.noise_level_normalization = noise_level_normalization

        self.config.sensor = [1]
        self.config.range_interval = range_interval
        self.config.update_rate = update_rate
        self.config.profile = profile
        self.config.mur = mur
        self.config.repetition_mode = repetition_mode
        self.config.downsampling_factor = downsampling_factor
        self.config.hw_accelerated_average_samples = hw_accelerated_average_samples
        self.config.gain = gain
        self.config.maximize_signal_attenuation = maximize_signal_attenuation
        self.config.tx_disable = tx_disable
        self.config.power_save_mode = power_save_mode
        self.config.asynchronous_measurement = asynchronous_measurement

        self.get_logger().info(f"Config: {self.config}")


        # Připojení klienta
        self.client.connect()
        session_info = self.client.setup_session(self.config)
        self.get_logger().info(f"Session info:\n{session_info}\n")


        # Zahájení měření
        self.client.start_session()
        self.timer = self.create_timer(1.0 / update_rate, lambda: self.read_and_publish_data(service_type))

        self.publish_session_and_config_info(session_info, service_type, serial_port)

    def read_and_publish_data(self, service_type):
        try:
            data_info, data = self.client.get_next()

            if service_type == "sparse":

                # Ověření a logování tvaru dat
                #self.get_logger().info(f"Received sparse data shape: {data.shape}")
                #self.get_logger().info(f"Received sparse data : {data.tolist()}")            
                self.publish_frame_data(frame=data, sweeps_per_frame=self.config.sweeps_per_frame)

            elif service_type == "envelope":
                 # Ověření a logování tvaru dat
                #self.get_logger().info(f"Received envelope data shape: {data.shape}")
                #self.get_logger().info(f"Received envelope data : {data.tolist()}")            
                self.publish_data(data)

                # msg = Float32MultiArray() 
                # msg.data = data.tolist() 
                # self.publisher_.publish(msg) 
                # self.get_logger().info(f"Published radar data: {data}") 
            

        except Exception as e:
            self.get_logger().error(f"Error reading radar data: {e}")

    def publish_session_and_config_info(self, session_info, service_type, serial_port):
        session_info_msg = String() 

        full_info = {
            "session_info": session_info,
            "config": {
                "sensor": self.config.sensor,
                "serial_port": serial_port,
                "service_type": service_type,
                "range_interval": self.config.range_interval.tolist(),
                "update_rate": self.config.update_rate,
                "profile": str(self.config.profile),
                "mur": str(self.config.mur),
                "repetition_mode": str(self.config.repetition_mode),
                "downsampling_factor": self.config.downsampling_factor,
                "hw_accelerated_average_samples": self.config.hw_accelerated_average_samples,
                "gain": self.config.gain,
                "maximize_signal_attenuation": self.config.maximize_signal_attenuation,
                "tx_disable": self.config.tx_disable,
                "power_save_mode": str(self.config.power_save_mode),
                "asynchronous_measurement": self.config.asynchronous_measurement,
            }
        }

        if service_type == "sparse":
            full_info["config"]["sweeps_per_frame"] = self.config.sweeps_per_frame

        if service_type == "envelope":
            full_info["config"]["noise_level_normalization"] = self.config.noise_level_normalization
            full_info["config"]["running_average_factor"] = self.config.running_average_factor

        session_info_msg.data = json.dumps(full_info, indent=4)  
        self.session_info_publisher.publish(session_info_msg) 
        self.get_logger().info("Published radar session info with conditional config.")


    def publish_frame_data(self, frame, sweeps_per_frame):
        msg = Float32MultiArray()
        frame_flattened = np.array(frame).flatten().tolist()

        # Přidání sweeps_per_frame jako první prvek
        msg.data = [sweeps_per_frame] + frame_flattened

        # Publikace zprávy
        self.data_publisher.publish(msg)





    def publish_data(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.data_publisher.publish(msg)
        #self.get_logger().info("Published radar data.")

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


