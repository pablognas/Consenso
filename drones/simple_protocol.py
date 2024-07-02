import enum
import json
import logging
from typing import TypedDict

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import SendMessageCommand, BroadcastMessageCommand
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration
from gradysim.protocol.position import squared_distance

def getRange():
    return 50

class SimpleSender(enum.Enum):
    SENSOR = 0
    UAV = 1
    GROUND_STATION = 2

class MessageType(enum.Enum):
    BEACON = 0
    DATA = 1
    ASSIGNMENT = 2
    ACKNOLEDGMENT = 3

class Message ():
    category: int
    sender_type: int
    sender: int
    position: tuple
    waypoints: list
    sensor_count: int

def report_Dmessage(message: Message) -> str:
    return (f"Received data from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']}")

def report_DataReport(message: Message) -> str:
    return (f"Received {message['sensor_count']} sensor(s) data from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']}")

def report_Bmessage(message: Message) -> str:
    return (f"Received beacon from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']} at {message['position']}")

def report_Amessage(message: Message) -> str:
    return  (f"Received mission with {message['waypoints']} waypoins from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']}")

class SimpleSensorProtocol(IProtocol):
    _log: logging.Logger
    packet: bool
    position: tuple

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self.packet = False

        self._generate_packet()

    def _generate_packet(self) -> None:
        self.packet = True
        self._log.info(f"Generated packet, communication ready")
        self.provider.schedule_timer("", self.provider.current_time() + 1)
    
    def _send_beacon(self) -> None:
        self._log.info(f"Sending beacon at {self.position}")

        message: Message = {
            'category': MessageType.BEACON.value,
            'position': self.position,
            'sender_type': SimpleSender.SENSOR.value,
            'sender': self.provider.get_id(),
            'waypoints': []
        }
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        if self.packet:
            self._send_beacon()
    
    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.DATA.value:
            self._log.info(report_Dmessage(received_message))

            if received_message['sender_type'] == SimpleSender.UAV.value and self.packet:
                response: Message = {
                    'category': MessageType.DATA.value,
                    'sender_type': SimpleSender.SENSOR.value,
                    'sender': self.provider.get_id()
                }

                command = SendMessageCommand(json.dumps(response), received_message['sender'])
                self.provider.send_communication_command(command)
                self.packet = False
                self._log.info(f"Sent data to UAV {received_message['sender']}")

                self.packet = False
                self.provider.cancel_timer("")
        elif received_message['category'] == MessageType.BEACON.value and self.packet:
            #simple_message: Message = json.loads(message)
            self._log.info(report_Bmessage(received_message))
            self.provider.cancel_timer("")
            self._send_beacon()


    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.position = telemetry.current_position

    def finish(self) -> None:
        self._log.info(f"Final state: {self.packet}")
    
class SimpleUAVProtocol(IProtocol):
    _log: logging.Logger
    
    sensor_count: int
    assigned: bool
    position: tuple

    _mission: MissionMobilityPlugin

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self.sensor_count = 0
        self.assigned = False
        self._log.info(f"Operation started")

        self._mission = MissionMobilityPlugin(self, MissionMobilityConfiguration(
             tolerance= 0.5
        ))

        self._mission.stop_mission()

    def _send_beacon(self) -> None:
        self._log.info(f"Sending beacon at {self.position}")

        message: Message = {
            'category': MessageType.BEACON.value,
            'position': self.position,
            'sender_type': SimpleSender.UAV.value,
            'sender': self.provider.get_id()
        }
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def _send_data(self) -> None:
        self._log.info(f"Sending data")

        message: Message = {
            'category': MessageType.DATA.value,
            'sender_type': SimpleSender.UAV.value,
            'sender': self.provider.get_id(),
            'sensor_count': self.sensor_count
        }
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

    def _request_data(self) -> None:
        self._log.info(f"Requesting data")

        message: Message = {
            'category': MessageType.DATA.value,
            'sender_type': SimpleSender.UAV.value,
            'sender': self.provider.get_id()
        }
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        if not self._mission.is_idle:
            self._send_beacon()

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.position = telemetry.current_position
        try:
            if squared_distance(self.position, self._mission._current_mission[self._mission._current_waypoint]) < getRange()/10:
                self.provider.cancel_timer("")
                self._request_data()
        except TypeError:
            pass

    
    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.BEACON.value:
                #received_message: Message = json.loads(message)
                self._log.info(report_Bmessage(received_message))
                if received_message['sender_type'] == SimpleSender.SENSOR.value:
                    #waypoints = self._mission._current_mission
                    new_waypoints = [received_message['position']]+self._mission._current_mission[self._mission._current_waypoint:]
                    self._mission.stop_mission()
                    self._mission.start_mission(new_waypoints)
                

                    
        elif received_message['category'] == MessageType.DATA.value:
            self._log.info(report_Dmessage(received_message))
            if received_message['sender_type'] == SimpleSender.SENSOR.value:
                self.sensor_count += 1
        elif received_message['category'] == MessageType.ASSIGNMENT.value:
            if not self.assigned:
                received_message: Message = json.loads(message)
                self._log.info(report_Amessage(received_message))
                self._mission.start_mission(received_message['waypoints'])
                self.assigned = True
                self.provider.schedule_timer("", self.provider.current_time() + 1)
            elif self.sensor_count > 0:
                    self._send_data()
                    self.sensor_count = 0
        pass

    def finish(self) -> None:
        self._log.info(f"Final sensor count: {self.sensor_count}")

#points = [(100,100,20),(100,900,20),(900,900,20),(900,100,20),(25,25,0)]

class SimpleGroundStationProtocol(IProtocol):
    _log: logging.Logger
    sensor_count: int
    points: tuple

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self._log.info(f"Operation started")
        self.sensor_count = 0
        self.points = [(50,50,20),(50,450,20),(450,450,20),(450,50,20),(15,15,0)]
        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def _send_assignment(self) -> None:
        self._log.info(f"Sending assignment to {self.points}")

        message: Message = {
            'category': MessageType.ASSIGNMENT.value,
            'waypoints': self.points,
            'sender_type': SimpleSender.GROUND_STATION.value,
            'sender': self.provider.get_id()
        }
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        self._send_assignment()

    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.DATA.value:
            if received_message['sender_type'] == SimpleSender.UAV.value:
                try:
                    self.sensor_count += received_message['sensor_count']
                    self._log.info(report_DataReport(received_message))
                    self.provider.cancel_timer("")
                except KeyError:
                    self._log.info(report_Dmessage(received_message))

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        self._log.info(f"Final sensor count: {self.sensor_count}")
