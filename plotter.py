import matplotlib.pyplot as plt


class TelemetryPlotter:

    def __init__(self):
        self.latitudes = []
        self.longitudes = []
        self.altitudes = []

        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_zlabel("Altitude")
        self.ax.set_title("Live XBee Telemetry")

    def update(self, latitude, longitude, altitude):
        self.latitudes.append(latitude)
        self.longitudes.append(longitude)
        self.altitudes.append(altitude)

        self.ax.clear()

        self.ax.plot(
            self.longitudes,
            self.latitudes,
            self.altitudes,
            marker="o"
        )

        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_zlabel("Altitude")
        self.ax.set_title("Live XBee Telemetry")

        plt.draw()
        plt.pause(0.01)
