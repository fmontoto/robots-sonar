import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import serial

import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("arduino_connection")

class SerialInterfaceArduinoSonar(object):
    NEW_DISTANCE_METHOD = "ND"
    RISE_FALL_METHOD = "RF"
    STANDARD_DISTANCE = "SD"
    IS_ALIVE = "AL"

    def __init__(self, *args, **kwargs):
        self._serial = self.start_serial_(*args, **kwargs)
        import time
        # TODO There should be a way to wait until the serial connection
        # is ready without the sleep.
        time.sleep(2)

    @staticmethod
    def start_serial_(port='/dev/ttyCOM0', baud_rate=9600):
        parity = serial.PARITY_NONE
        stop_bits = serial.STOPBITS_ONE
        byte_size = serial.EIGHTBITS
        timeout = 10  # secs

        return serial.Serial(
            port=port,
            baudrate=baud_rate,
            parity=parity,
            stopbits=stop_bits,
            bytesize=byte_size,
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
        logging.debug("Arduino alive: %s", got)
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

    def run_rise_fall_echo(self):
        before_rise = []
        after_rise = []
        after_fall = []
        results = (before_rise, after_rise, after_fall)
        self._serial.reset_input_buffer()
        self._serial.write(self.RISE_FALL_METHOD.encode("utf-8"))
        self._serial.flush()
        status = self._serial.read(2)
        if status != "OK":
            raise ValueError(
                    "Unexpected output before starting new distance %s", status)
        for l in results:
            while True:
                rcvd = int(self._read_line())
                if rcvd < 0:
                    if rcvd == -1:  # This is the sentinel when the echo falls.
                        logging.debug("-1 received, %d values got before it",
                                      len(l))
                        break
                    if rcvd == -2:
                        if l is not results[len(results) - 1]:
                            raise ValueError("Received end of data before expected.")
                        break
                    else:
                        raise ValueError("Received an unexpected value %d" % rcvd)
                else:
                    l.append(rcvd)

        return results

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
        import time
        time.sleep(2)
        while True:
            rcvd = int(self._read_line())
            if rcvd < 0:
                if rcvd == -1:  # This is the sentinel when the echo falls.
                    logging.debug("Echo received, %d values got before it",
                                  len(before_echo_fall))
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
        logger.info("Trying to connect with arduino at %s", ttys[0])
        return os.path.join(path, ttys[0])
    else:
        raise ValueError("More than one port available: %s" % ttys)


def plot_signal(before_echo, after_echo):

    points = []
    points.extend(before_echo)
    points.extend(after_echo)

    plt.plot(points)
    print(len(before_echo))
    print(len(after_echo))
    plt.axvline(len(before_echo), linestyle='--')
    plt.show()


def plot_signal_vertical_separations(tuple_of_point_lists, distance):
    plt.title("Using the standard method the sonar implements.", fontsize=10)
    plt.suptitle("Distance: %d" % distance, fontsize=18)
    all_points = [x for l in tuple_of_point_lists for x in l]

    plt.plot(all_points)
    acumulator = 0
    for l in tuple_of_point_lists[:-1]:
        acumulator = acumulator + len(l)
        plt.axvline(acumulator, linestyle='--')
        #plt.text(acumulator + 0.1,8, "Echo bla", rotation=270)
    plt.show()


def main():
    arduino_interface = SerialInterfaceArduinoSonar(
            port=get_serial_port())
    if not arduino_interface.is_arduino_alive():
        raise ValueError("Error comunicating with the arduino")

    distance = arduino_interface.run_standard_distance_method()
    print("Distance %d" % distance)
    plot_signal_vertical_separations(arduino_interface.run_rise_fall_echo(), distance)

    return 0

if __name__ == "__main__":
	main()
