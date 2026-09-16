from digi.xbee.devices import XBeeDevice
from telemetry import parse_telemetry
from plotter import TelemetryPlotter


PORT = "COM6"
BAUD_RATE = 9600


device = XBeeDevice(PORT, BAUD_RATE)
plotter = TelemetryPlotter()


try:
    device.open()

    print("XBee connected.")
    print("Waiting for telemetry...")

    while True:

        message = device.read_data()

        if message is None:
            continue

        # XBee payload: bytes → string
        packet = message.data.decode("utf-8")

        print("Received:", packet)

        try:
            telemetry = parse_telemetry(packet)

            plotter.update(
                telemetry["latitude"],
                telemetry["longitude"],
                telemetry["altitude"]
            )

        except (ValueError, UnicodeDecodeError) as error:
            print("Invalid telemetry:", error)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    if device.is_open():
        device.close()
