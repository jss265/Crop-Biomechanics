import serial

COM_PORT = "COM25"
BAUD_RATE = 115200


def main():
	with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
		while True:
			line = ser.readline().decode("utf-8", errors="ignore").strip()
			if line:
				print(line)


if __name__ == "__main__":
	main()
