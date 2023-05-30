#!/usr/bin/env python3

# Copyright (c) 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import abc
import logging
import threading
import time
from typing import Callable, List
import queue
import traceback

class CaBotArduinoSerialDelegate(abc.ABC):
    """Delegate definition for CaBotArduinoDriver class"""
    def __init__(self):
        pass

    @abc.abstractmethod
    def system_time(self):
        """
        return system time by tuple (sec, nsec)
        """

    @abc.abstractmethod
    def stopped(self):
        """
        signal stopped
        """

    @abc.abstractmethod
    def log(self, level: int, text: str) -> None:
        """
        implement log output for the system
        @param level: logging level
        @param text: logging text
        """

    @abc.abstractmethod
    def log_throttle(self, level: int, interval: float, text: str) -> None:
        """
        implement log throttle output for the system
        @param level: logging level
        @param interval: logging interval
        @param text: logging text
        """

    @abc.abstractmethod
    def get_param(self, name: str, callback: Callable[[List[int]], None]) -> None:
        """
        get parameter from the system
        @param name: name of the parameter
        @param callback: call this callback with an int array
        """

    @abc.abstractmethod
    def publish(self, cmd: bytes, data: bytearray) -> None:
        """
        publish data according to the cmd
        @param cmd: publish topic type
        @param data: compact data of the message which needs to be converted

        defined topics and cmd
        # touch       0x10
        # touch_raw   0x11
        # buttons     0x12
        # imu         0x13
        # calibration 0x14
        # pressure    0x15
        # temperature 0x16
        # wifi        0x20
        """


class CaBotArduinoSerial:
    """
    Implementation of serial protocol to communicate with cabot arduino
    """
    def __init__(self, port, baud, timeout=1):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self.delegate = None
        self.read_thread = None
        self.read_lock = threading.RLock()
        self.write_thread = None
        self.write_queue = queue.Queue()
        self.write_lock = threading.RLock()
        self.is_alive = True
        self.read_count = 0
        self.time_synced = False
        self.no_input_count = 0

    def start(self):
        self.reset_serial()

    def reset_serial(self):
        self.delegate.log(logging.INFO, "Resetting serial port")
        self.port.setDTR(False)
        time.sleep(0.2)
        self.port.flushInput()
        self.port.setDTR(True)

    def stop(self):
        self.is_alive = False
        self.delegate.stopped()

    def run_once(self):
        try:
            if self._process_read_once():
                self.no_input_count = 0
            else:
                self.no_input_count += 1
                if self.no_input_count > 10000:
                    self.no_input_count = 0
                    self.stop()
        except OSError as error:
            # somtimes read error can happen even if it is okay
            pass
        except:  # noqa: E722
            self.delegate.log(logging.ERROR, traceback.format_exc())
            self.stop()
        try:
            self._process_write_once()
        except:  # noqa: E722
            self.delegate.log(logging.ERROR, traceback.format_exc())
            self.stop()

    def send_command(self, command, arg):
        count = len(arg)
        data = bytearray()
        data.append(0xAA)
        data.append(0xAA)
        data.append(command)
        data.append(count & 0xFF)
        # if you want to extend the data size more than 256
        # need to change cabot-arduino-serial as well
        # data.append((count >> 8) & 0xFF)
        for i in range(0, count):
            data.append(arg[i])
        data.append(self.checksum(arg))
        self.delegate.log(logging.DEBUG, F"send {str(data)}")
        self.write_queue.put(bytes(data))

    def _process_write_once(self):
        if self.write_queue.empty():
            return False
        data = self.write_queue.get()
        if isinstance(data, bytes):
            length = len(data)
            total = 0
            while total < length:
                total += self.port.write(data[total:])
                self.delegate.log(logging.DEBUG, F"{total} bytes written")
        else:
            self.delegate.log(logging.ERROR,
                              F"Trying to write invalid data type: {type(data)}")
        return True

    def _try_read(self, length):
        try:
            bytes_remaining = length
            result = bytearray()
            read_start = time.time()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    result.extend(received)
                    bytes_remaining -= len(received)
            if bytes_remaining != 0:
                raise IOError(F"Returned short (expected {length} bytes,"
                              F"received {length - bytes_remaining} instead).")
            self.read_count += length
            return bytes(result)
        except Exception as error:
            raise IOError(F"Serial Port read failure: {str(error)}")

    def _process_read_once(self):
        """
        serial command format:
        \xAA\xAA[cmd,1][size,2][data,size][checksum]
        """
        if self.port.inWaiting() < 1:
            return False

        cmd = 0
        received = self._try_read(1)
        if received[0] != 0xAA:
            return False
        received = self._try_read(1)
        if received[0] != 0xAA:
            return False

        self.delegate.log(logging.DEBUG, "reading command")
        cmd = self._try_read(1)[0]
        self.delegate.log(logging.DEBUG, F"cmd={cmd}")
        size = int.from_bytes(self._try_read(2), 'little')
        self.delegate.log(logging.DEBUG, F"size={size}")
        data = self._try_read(size)
        self.delegate.log(logging.DEBUG, F"data length={len(data)}")
        checksum = int.from_bytes(self._try_read(1), 'little')
        checksum2 = self.checksum(data)
        self.delegate.log(logging.DEBUG, F"checksum {checksum} {checksum2}")
        if checksum != checksum2:
            return False
        self.delegate.log(logging.DEBUG, F"read data command={cmd} size={size}")

        if cmd == 0x01:  # time sync
            self._send_time_sync(data)
        elif cmd == 0x02:  # logdebug
            self.delegate.log(logging.DEBUG, data.decode('utf-8'))
        elif cmd == 0x03:  # loginfo
            self.delegate.log(logging.INFO, data.decode('utf-8'))
        elif cmd == 0x04:  # logwarn
            self.delegate.log(logging.WARNING, data.decode('utf-8'))
        elif cmd == 0x05:  # logerr
            self.delegate.log(logging.ERROR, data.decode('utf-8'))
        elif cmd == 0x08:  # get param
            def send_param(data):
                temp = bytearray()
                if isinstance(data, (list, tuple)):
                    for d in data:
                        temp.extend(d.to_bytes(4, 'little'))
                else:
                    temp.extend(data.to_bytes(4, 'little'))
                self.send_command(0x08, temp)
            self.delegate.get_param(data.decode('utf-8'), send_param)
        elif 0x10 <= cmd:
            self.delegate.publish(cmd, data)
        else:
            self.delegate.log(logging.ERROR, F"unknwon command {cmd:#04x}")
            return False
        return True

    def _send_time_sync(self, data):
        # send current time
        (sec, nsec) = self.delegate.system_time()
        temp = bytearray()
        temp.extend(sec.to_bytes(4, 'little'))
        temp.extend(nsec.to_bytes(4, 'little'))
        self.send_command(0x01, temp)
        self.time_synced = True
        self.delegate.log(logging.DEBUG, "sync")

        remote_sec = int.from_bytes(data[0:4], 'little')
        remote_nsec = int.from_bytes(data[4:8], 'little')
        diff_ms = (sec - remote_sec) * 1000 + (nsec - remote_nsec) / 1000000
        self.delegate.log(
            logging.DEBUG,
            F",,,,,,,,,,{remote_sec%1000}.{remote_nsec/1000000},"
            F"{sec%1000}.{nsec/1000000},diff,{int(diff_ms)}")

    def checksum(self, data):
        temp = 0
        for d in data:
            temp += d
        return 0xFF - (0xFF & temp)
