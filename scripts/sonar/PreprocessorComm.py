import threading
import serial
import Queue
import time
import os
import yaml


class PreprocessorComm(threading.Thread):
    def __init__(self, port, param_path, ping_queue):
        self.serial = serial.Serial(port, timeout=0.5)

        self.shutdown_flag = threading.Event()
        self.buf = bytearray()
        self.ping_queue = ping_queue
        self.write_response = None
        self.read_response = None

        if self.__load_params(param_path):
            print('Loaded initial params')
        else:
            raise RuntimeError('ERROR: Unable to load initial params')

        threading.Thread.__init__(self)

    def run(self):
        while True:
            # this is a blocking call. Waits for a full line to be published
            i = max(1, min(2048, self.serial.in_waiting))
            data = self.serial.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                r = r.decode("utf-8")
                self.buf[0:] = data[i + 1:]
                parsed_type, parsed_data = self.__parse_input(
                    r, self.sonar_params)
                if parsed_type == 'ping':
                    self.ping_queue.put(parsed_data)
                if parsed_type == 'write_response':
                    self.write_response = parsed_data
                if parsed_type == 'read_response':
                    self.read_response = parsed_data
            else:
                self.buf.extend(data)

    def send_param(self, command, data, timeout=0.2):
        if command in self.sonar_params:
            self.__write('$set {} {}\n'.format(command, data))

            current_timeout = timeout
            while timeout == 0 or (timeout != 0 and current_timeout > 0):
                if self.write_response is not None:
                    if self.write_response['cmd'] == command and \
                                    self.write_response['val'] == data:
                        self.sonar_params[command] = self.write_response['val']
                        print('wrote {}={}'.format(command,
                                                   self.write_response['val']))
                        self.write_response = None
                        return True
                    else:
                        self.write_response = None
                        return False
                time.sleep(0.1)
                current_timeout = current_timeout - 0.1
        return False

    def read_param(self, command, timeout=0.2):
        if command in self.sonar_params:
            self.__write('$get {}\n'.format(command))

            current_timeout = timeout
            while timeout == 0 or (timeout != 0 and current_timeout > 0):
                if self.read_response is not None:
                    if self.read_response['cmd'] == command:
                        self.sonar_params[command] = self.read_response['val']
                        self.read_response = None
                        return True
                    else:
                        self.read_response = None
                        return False
                time.sleep(0.1)
                current_timeout = current_timeout - 0.1
            return False

    def write_current_params(self):
        status = False
        for param in self.sonar_params:
            status = self.send_param(param, self.sonar_params[param])
        return status

    def __load_params(self, path):
        if os.path.exists(path):
            params = yaml.load(open(path))
            self.sonar_params = params
            return True
        else:
            return False

    def __write(self, data):
        if self.serial.is_open:
            self.serial.write(data=data)
        else:
            raise RuntimeError("Cannot write. Serial port is closed.")

    @staticmethod
    # returns command type, data
    def __parse_input(data, command_dict):
        if data is not None:
            data = data.strip()
            split_string = data.split()

            if len(split_string) > 1:
                # proper command ?
                if split_string[0][0] == '$':
                    command = split_string[0][1:]
                    if command == 'ping':
                        return 'ping', PreprocessorComm.__parse_ping(
                            split_string[1:])
                    if command == 'set':
                        value = PreprocessorComm.__parse_number(
                            split_string[2])
                        return 'write_response', {
                            'cmd': split_string[1],
                            'val': value
                        }
                    elif command in command_dict:
                        value = PreprocessorComm.__parse_number(
                            split_string[1])
                        return 'read_response', {'cmd': command, 'val': value}
        # if not a proper instruction
        return None, None

    @staticmethod
    def __parse_ping(data):
        ping_data = {'timestamp': time.time()}
        for pair in data:
            key_val_pair = pair.split('=')
            if len(key_val_pair) == 2:
                key = key_val_pair[0]
                value = PreprocessorComm.__parse_number(key_val_pair[1])
                if value is not None:
                    ping_data[key] = value
        return ping_data

    @staticmethod
    def __parse_number(data):
        try:
            value = float(data)
            # if int or float
            if value == int(value):
                return int(value)
            else:
                return value
        except ValueError:
            return None
