import sys  # Přidání chybějícího importu
import acconeer.exptool as et

def main():
    print("Final sys.argv:", sys.argv)  # Debug pro kontrolu argumentů

    args = et.a111.ExampleArgumentParser().parse_args()
    et.utils.config_logging(args)

    client = et.a111.Client(**et.a111.get_client_args(args))
    config = et.a111.EnvelopeServiceConfig()
    config.sensor = args.sensors
    config.range_interval = [0.2, 0.3]
    config.update_rate = 10

    client.connect()
    session_info = client.setup_session(config)
    print("Session info:\n", session_info, "\n")

    client.start_session()

    for i in range(1000):
        data_info, data = client.get_next()
        print(f"Sweep {i + 1}:\n", data_info, "\n", data, "\n")

    client.stop_session()
    client.disconnect()

if __name__ == "__main__":
    main()
