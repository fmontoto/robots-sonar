import os
import serial
import struct

class SerialInterfaceArduinoSonar(object):
    NEW_DISTANCE_METHOD = "ND"
    STANDARD_DISTANCE = "SD"
    IS_ALIVE = "AL"

    def __init__(self, *args, **kwargs):
        self._serial = self.start_serial_(*args, **kwargs)

    @staticmethod
    def start_serial_(port='/dev/ttyCOM0', baudrate=230400):
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
        r = self._serial.read()
        while r != "\n":
            ret.append(r)
            try:
                r = self._serial.read()
            except OSError:
                r = self._serial.read()
        return ''.join(ret)

    def is_arduino_alive(self):
        self._serial.write(self.IS_ALIVE.encode("utf-8"));
        self._serial.flush()
        got = self._serial.read(3)
        return got == "YES"

    def run_new_distance_method(self):
        before_echo_fall = []
        after_echo_fall = []
        self._serial.write(self.NEW_DISTANCE_METHOD.encode("utf-8"))
        self._serial.flush()
        status = self._serial.read(2)
        if status != "OK":
            raise ValueError(
                    "Unexpected output before starting new distance %s", status)
        while True:
            rcvd = self._read_line()
            print(rcvd)
            rcvd = int(rcvd)
            if(rcvd < 0):
                if(rcvd == -1):  # This is the sentinel when the echo falls.
                    break
                else:
                    raise ValueError("Received an unexpected value %d" % rcvd)
            else:
                before_echo_fall.append(rcvd)

        while True:
            rcvd = self._read_line()
            print(rcvd)
            rcvd = int(rcvd)
            if(rcvd < 0):
                if(rcvd == -2):  # This is the sentinel when the output ends.
                    break
                else:
                    raise ValueError("Received an unexpected value %d" % rcvd)
            else:
                after_echo_fall.append(rcvd)

        print(before_echo_fall)
        print(after_echo_fall)




def get_serial_port():
    path = "/dev"
    all_devs = os.listdir(path)
    ttys = [x for x in all_devs if x.startswith("ttyAC")]
    if len(ttys) == 1:
        return os.path.join(path, ttys[0])
    else:
        raise ValueError("More than one port available: %s" % ttys)

def main():
    arduino_interface = SerialInterfaceArduinoSonar(
            port=get_serial_port())
    print(arduino_interface.is_arduino_alive())
    arduino_interface.run_new_distance_method()
    return 0;

if __name__ == "__main__":
	main()
