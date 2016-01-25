import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import serial

class SerialInterfaceArduinoSonar(object):
    NEW_DISTANCE_METHOD = "ND"
    STANDARD_DISTANCE = "SD"
    IS_ALIVE = "AL"

    def __init__(self, *args, **kwargs):
        self._serial = self.start_serial_(*args, **kwargs)

    @staticmethod
    def start_serial_(port='/dev/ttyCOM0', baudrate=9600):
        parity = serial.PARITY_NONE
        stopbits = serial.STOPBITS_ONE
        bytesize = serial.EIGHTBITS
        timeout = 10  # secs

        return serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout
        )

    def _read_line(self):
        ret = []
        try:
            r = self._serial.read()
        except (OSError, serial.serialutil.SerialException):
            r = self._serial.read()
        while r != "\n":
            ret.append(r)
            try:
                r = self._serial.read()
            except (OSError, serial.serialutil.SerialException):
                r = self._serial.read()
        return ''.join(ret)

    def is_arduino_alive(self):
        self._serial.write(self.IS_ALIVE)
        self._serial.flush()
        got = self._serial.read(3)
        return got == "YES"

    def run_standard_distance_method(self):
        self._serial.reset_input_buffer()
        self._serial.write(self.STANDARD_DISTANCE.encode("utf-8"))
        self._serial.flush()
        status = self._serial.read(2)
        if status != "OK":
            raise ValueError(
                    "Unexpected output before starting new distance %s", status)
        return int(self._read_line())

    def run_new_distance_method(self):
        before_echo_fall = []
        after_echo_fall = []
        self._serial.reset_input_buffer()
        self._serial.write(self.NEW_DISTANCE_METHOD.encode("utf-8"))
        self._serial.flush()
        status = self._serial.read(2)
        if status != "OK":
            raise ValueError(
                    "Unexpected output before starting new distance %s", status)
        while True:
            rcvd = int(self._read_line())
            if rcvd < 0:
                if rcvd == -1:  # This is the sentinel when the echo falls.
                    break
                else:
                    raise ValueError("Received an unexpected value %d" % rcvd)
            else:
                before_echo_fall.append(rcvd)

        while True:
            rcvd = int(self._read_line())
            if rcvd < 0:
                if rcvd == -2:  # This is the sentinel when the output ends.
                    break
                else:
                    raise ValueError("Received an unexpected value %d" % rcvd)
            else:
                after_echo_fall.append(rcvd)

        return before_echo_fall, after_echo_fall


def get_serial_port():
    path = "/dev"
    all_devs = os.listdir(path)
    ttys = [x for x in all_devs if x.startswith("ttyAC")]
    if len(ttys) == 1:
        return os.path.join(path, ttys[0])
    else:
        raise ValueError("More than one port available: %s" % ttys)


def myplot(points):
    def update_line(num, data, line):
        line.set_data(data[..., :num])
        return line

    plt.plot(points)
    plt.show()


def main():
    arduino_interface = SerialInterfaceArduinoSonar(
            port=get_serial_port())
    if not arduino_interface.is_arduino_alive():
        raise ValueError("Error comunicating with the arduino")

    print("Distance %d" % arduino_interface.run_standard_distance_method())
    b, a = arduino_interface.run_new_distance_method()
    all_points = []
    all_points.extend(b)
    all_points.extend(a)
    myplot(all_points)

    return 0

if __name__ == "__main__":
	main()
