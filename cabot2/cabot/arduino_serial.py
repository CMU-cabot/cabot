#!/usr/bin/env python3
import abc
from typing import Callable, List
import logging
import struct
import sys
import termios
import threading
import time
import traceback
import queue

import serial

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

    def start(self):
        self._open_serial()
        self._run()

    def _open_serial(self):
        self.delegate.log(logging.INFO, "resetting serial port")
        self.port.setDTR(False)
        time.sleep(0.1)
        self.port.flushInput()
        self.port.setDTR(True)
        self.delegate.log(logging.INFO, "serial port reset")

    def stop(self):
        self.is_alive = False
        self.delegate.stopped()

    def _process_write(self):
        try:
            while self.is_alive:
                if self.write_queue.empty():
                    time.sleep(0.000001)
                    continue
                data = self.write_queue.get()
                if isinstance(data, bytes):
                    with self.write_lock:
                        self.port.write(data)
                else:
                    self.deegate.log(logging.ERROR,
                                      'Trying to write invalid data type: %s' % type(data))
        except serial.SerialTimeoutException as exc:
            self.delegate.log(logging.ERROR, 'Write timeout: %s' % exc)
            time.sleep(1)
        except RuntimeError as exc:
            self.delegate.log(logging.ERROR, 'Write thread exception: %s' % exc)
        finally:
            self.delegate.log(logging.INFO, "stopped writing")
            self.stop()

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
                raise IOError("Returned short (expected %d bytes,"
                              "received %d instead)." % (length, length - bytes_remaining))
            self.read_count += length
            return bytes(result)
        except Exception as error:
            raise IOError("Serial Port read failure: %s" % str(error))

    def _run(self):
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self._process_write)
            self.write_thread.daemon = True
            self.write_thread.start()
        if self.read_thread is None:
            self.read_thread = threading.Thread(target=self._process_read)
            self.read_thread.daemon = True
            self.read_thread.start()

    def _process_read(self):
        try:
            while self.is_alive:
                self._process_read_once()
        except OSError:
            pass
        finally:
            self.delegate.log(logging.ERROR, traceback.format_exc())
            self.delegate.log(logging.INFO, "stopped reading")
            self.stop()

    def _process_read_once(self):
        """
        serial command format:
        \xAA\xAA[cmd,1][size,2][data,size][checksum]
        """
        time.sleep(0.000001)
        if self.port.inWaiting() < 1:
            return

        cmd = 0
        received = self._try_read(1)
        if received[0] != 0xAA:
            return
        received = self._try_read(1)
        if received[0] != 0xAA:
            return

        self.delegate.log(logging.DEBUG, "reading command")
        cmd = self._try_read(1)[0]
        self.delegate.log(logging.DEBUG, "cmd={}".format(cmd))
        size = int.from_bytes(self._try_read(2), 'little')
        self.delegate.log(logging.DEBUG, "size={}".format(size))
        data = self._try_read(size)
        self.delegate.log(logging.DEBUG, "data length={}".format(len(data)))
        checksum = int.from_bytes(self._try_read(1), 'little')
        checksum2 = self.checksum(data)
        self.delegate.log(logging.DEBUG, "checksum {} {}".format(checksum, checksum2))
        if checksum != checksum2:
            return
        self.delegate.log(logging.DEBUG, "read data command={} size={}".format(cmd, size))

        if cmd == 0x01:  # time sync
            #self.check_time_diff(data)
            self.send_time_sync(data)
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
                for d in data:
                    temp.extend(d.to_bytes(4, 'little'))
                self.send_command(0x08, temp)
            self.delegate.get_param(data.decode('utf-8'), send_param)
        elif 0x10 <= cmd:
            self.delegate.publish(cmd, data)
        else:
            self.delegate.log(logging.ERROR, "unknwon command %x"%(cmd))

    def check_time_diff(self, data):
        # check time difference
        if not self.time_synced:
            return
        remote_sec = int.from_bytes(data[0:4], 'little')
        remote_nsec = int.from_bytes(data[4:8], 'little')
        (sec, nsec) = self.delegate.system_time()
        diff_ms = (sec - remote_sec) * 1000 + (nsec - remote_nsec) / 1000000

        if abs(diff_ms) > 100:
            self.delegate.log(logging.WARNING,
                              "large difference in time %d ms"%(diff_ms))

    def send_time_sync(self, data):
        # send current time
        (sec, nsec) = self.delegate.system_time()
        remote_sec = int.from_bytes(data[0:4], 'little')
        remote_nsec = int.from_bytes(data[4:8], 'little')
        temp = bytearray()
        diff_ms = (sec - remote_sec) * 1000 + (nsec - remote_nsec) / 1000000
        temp.extend(sec.to_bytes(4, 'little'))
        temp.extend(nsec.to_bytes(4, 'little'))
        self.send_command(0x01, temp)
        self.time_synced = True
        self.delegate.log(logging.DEBUG, "sync")
        #self.delegate.log(logging.INFO,
        #                  ",,,,,,,,,,%d.%04d,%d.%04d,diff,%d"%(remote_sec%1000,remote_nsec/100000,sec%1000,nsec/100000,int(diff_ms)))

    def send_command(self, command, arg):
        count = len(arg)
        data = bytearray(count+6)
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
        #self.delegate.log(logging.INFO, "send %s", data)
        self.write_queue.put(bytes(data))

    def checksum(self, data):
        temp = 0
        for d in data:
            temp += d
        return 0xFF - (0xFF & temp)
