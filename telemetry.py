def parse_telemetry(packet):
    fields = packet.strip().split(",")

    if len(fields) != 10:
        raise ValueError(f"Expected 10 fields, got {len(fields)}")

    return {
        "timestamp": int(fields[0]),
        "state": int(fields[1]),
        "temperature": float(fields[2]),
        "pressure": float(fields[3]),
        "altitude": float(fields[4]),
        "battery_voltage": float(fields[5]),
        "battery_current": float(fields[6]),
        "latitude": float(fields[7]),
        "longitude": float(fields[8]),
        "prev_cmd_echo": fields[9]
    }
