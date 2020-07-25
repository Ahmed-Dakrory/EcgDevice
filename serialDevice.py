import sys
import glob
import serial
import serial.tools.list_ports


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    ports = []
    if sys.platform.startswith('win'):
        portsDev = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(portsDev):
            ports.append(port)
    else:
        raise EnvironmentError('Unsupported platform')

    
    return ports

if __name__ == '__main__':
    print(serial_ports())