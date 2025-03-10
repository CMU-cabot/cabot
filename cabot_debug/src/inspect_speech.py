#!/usr/bin/env python3

from dataclasses import dataclass

import socketio
import time
import argparse
import json
import sys
import os
import threading

@dataclass
class CurrentSpeech:
    text: str = ""
    start_location = None
    end_location = None
    location: int = None
    length: int = 0
    start_at: float = 0
    end_at: float = 0

    def __str__(self):
        dt = time.localtime(self.start_at)
        ms = int((self.start_at - int(self.start_at)) * 10)
        start_at = time.strftime("%Y-%m-%d-%H-%M-%S", dt) + f'.{ms:01d}'
        read = self.text[self.location:self.location + self.length]
        remaining = self.text[self.location + self.length:]
        duration = self.end_at - self.start_at
        text = f"{start_at},"
        text += f"{duration:.1f},"
        if self.start_location:
            lat = self.start_location['lat']
            lng = self.start_location['lng']
            text += f"{lat:.7f},{lng:.7f},"
        text += ""
        text += f"{read},"
        text += f"{remaining}" if len(remaining) > 0 else ""
        return text


class SpeechInspector:
    def __init__(self):
        self.latest_location = None
        self.last_data = None
        self.count = 0
        self.current = CurrentSpeech()

    def input(self, event_data):
        event = event_data['event']
        data = event_data['data']

        if event == 'location':
            self.latest_location = data
        elif event == 'share':
            event_type = data['type']
            text = data['value']
            location = data['location']
            length = data['length']
            flag1 = data['flag1']
            info_id = data['info_id']
            if event_type == "Speak":
                self.current.text = text
                self.current.start_at = info_id / 1000000000.0
                self.current.start_location = dict(self.latest_location)
            elif event_type == "SpeakProgress":
                if flag1:
                    if self.current.location is not None:
                        print(f"{self.current}")
                    self.current = CurrentSpeech()
                else:
                    if self.current.location is None:
                        self.current.location = location
                    self.current.end_location = dict(self.latest_location)
                    self.current.end_at = info_id / 1000000000.0
                    self.current.length = location + length
            else:
                return
            self.count += 1
            self.last_data = data
            # for key, value in data.items():
            #     print(f"    {key}: {value}")



lock = threading.Lock()
alive = True
event_queue = []
def log_writer(log_file):
    global event_queue, alive
    print(f"Log writer thread started, writing to {log_file}")
    # if exists exit
    if os.path.exists(log_file):
        print(f"Log file {log_file} already exists. Exiting.")
        alive = False
        return
    with open(log_file, 'w') as log:
        while alive:
            with lock:
                if event_queue:
                    event = event_queue.pop(0)
                    log.write(json.dumps(event) + '\n')
                    log.flush()
            time.sleep(0.001)
    print("Log writer thread exiting.")

def main():
    global alive, event_queue
    parser = argparse.ArgumentParser(description="Inspect speech events.")
    parser.add_argument('-f', '--file', type=str, help="File to read events from")
    parser.add_argument('-l', '--log', type=str, help="Log events to a file")
    args = parser.parse_args()

    inspector = SpeechInspector()

    if args.file:
        with open(args.file, 'r') as f:
            with open(args.file, 'r') as f:
                for line in f:
                    event_data = json.loads(line)
                    inspector.input(event_data)
    else:
        writer_thread = None
        if args.log:
            writer_thread = threading.Thread(target=log_writer, args=(args.log,))
            writer_thread.start()
        # Create a Socket.IO client
        sio = socketio.Client()

        # Define event handlers
        @sio.event
        def connect():
            print("Connected to the server")

        @sio.event
        def disconnect():
            print("Disconnected from the server")

        # Listen to all events
        @sio.on('*')
        def catch_all(event, data):
            data = json.loads(data)
            payload = {'event': event, 'data': data}
            if args.log:
                with lock:
                    event_queue.append(payload)
            inspector.input(payload)


        # Connect to the server
        sio.connect('ws://localhost:5000/cabot')

        # Keep the script running
        try:
            while alive:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Exiting...")
            if writer_thread:
                alive = False
                writer_thread.join()
            sio.disconnect()


if __name__ == "__main__":
    main()
