import enum
import json
import logging
from typing import TypedDict

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import SendMessageCommand, BroadcastMessageCommand
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration
from gradysim.protocol.position import squared_distance

global checked
global unchecked

checked = 0
unchecked = 0

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
    COPY = 3
    DATAREQ = 4
    HALT = 5
    PARLAY = 6

class Message ():
    category: int
    sender_type: int
    sender: int
    position: tuple
    waypoints: list
    count: int
    away: bool

def new_message(category: int, sender_type: int, sender: int, position: tuple = None, waypoints: list = None, count: int = None, away: bool = None) -> Message:
    message: Message = {
        'category': category,
        'sender_type': sender_type,
        'sender': sender,
        'position': position,
        'waypoints': waypoints,
        'count': count,
        'away': away
    }
    return message

def report_Dmessage(message: Message) -> str:
    return (f"Received data from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']}")

def report_copy(message: Message) -> str:
    return (f"Received COPY from "
            f"{SimpleSender(message['sender_type']).name} {message['sender']}")

def report_DataReport(message: Message) -> str:
    return (f"Received {message['count']} sensor(s) data from "
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
    tracked: bool
    position: tuple

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self.packet = True
        self.tracked = False
        self._generate_packet()

    def _generate_packet(self) -> None:
        self.packet = True
        self._log.info(f"Generated packet, communication ready")
        #self.provider.schedule_timer("", self.provider.current_time() + 1)
    
    def _send_beacon(self,  destination : int = None) -> None:
        self._log.info(f"Sending beacon at {self.position}")

        message = new_message(MessageType.BEACON.value,SimpleSender.SENSOR.value,self.provider.get_id(),(self.position[0],self.position[1],20))
        command = SendMessageCommand(json.dumps(message), destination)
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        if not self.tracked:
            self._send_beacon()
    
    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.DATAREQ.value and self.packet:
            self._log.info(report_Dmessage(received_message))

            if received_message['sender_type'] == SimpleSender.UAV.value and self.tracked:
                response = new_message(MessageType.DATA.value,SimpleSender.SENSOR.value,self.provider.get_id())
                command = SendMessageCommand(json.dumps(response), received_message['sender'])
                self.provider.send_communication_command(command)
                self._log.info(f"Sent data to UAV {received_message['sender']}")
                self.packet = False
        elif received_message['category'] == MessageType.BEACON.value and received_message['sender_type'] == SimpleSender.UAV.value and not self.tracked:
            self._log.info(report_Bmessage(received_message))
            self._send_beacon(received_message['sender'])
            self.tracked = True
        elif received_message['category'] == MessageType.COPY.value:
            self._log.info(report_copy(received_message))
            

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.position = telemetry.current_position

    def finish(self) -> None:
        global checked
        global unchecked
        if self.packet:
            unchecked += 1
        else:
            checked += 1
        self._log.info(f"{checked} sensor(s) data acquired and {unchecked} sensor(s) data not acquired")
    
class SimpleUAVProtocol(IProtocol):
    _log: logging.Logger
    
    sensor_count: int
    assigned: bool
    position: tuple
    retired: bool
    away: bool
    gotData: bool
    next_waypoints: list

    parent: int
    deltaId: int

    _mission: MissionMobilityPlugin

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self.sensor_count = 0
        self.assigned = False
        self._log.info(f"Operation started")
        self.retired = False
        self.away = False
        self.gotData = False
        self.next_waypoints = []
        self.parent = None
        self.left = None
        self.right = None
        self.deltaId = None
        self.deltaSize = None

        self._mission = MissionMobilityPlugin(self, MissionMobilityConfiguration(
             tolerance= 0.5
        ))

        self._mission.stop_mission()
        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def _send_beacon(self) -> None:
        self._log.info(f"Sending beacon at {self.position}")
        message = new_message(
            category = MessageType.BEACON.value,
            position = self.position,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
            away = self.away,
            count = self.deltaSize,
            waypoints =self._mission._current_mission 
        )  
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)

        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def _send_data(self,  destination : int = None) -> None:
        self._log.info(f"Sending data")

        message = new_message(
            category = MessageType.DATA.value,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
            count = self.sensor_count
        )
        command = SendMessageCommand(json.dumps(message), destination)
        self.provider.send_communication_command(command)

    def _send_copy(self,  destination : int = None) -> None:
        self._log.info(f"Sending COPY")

        message = new_message(
            category = MessageType.COPY.value,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
        )
        command = SendMessageCommand(json.dumps(message), destination)
        self.provider.send_communication_command(command)

    def _send_halt(self,  destination : int = None) -> None:
        self._log.info(f"Sending Halt")

        message = new_message(
            category = MessageType.HALT.value,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
        )
        command = SendMessageCommand(json.dumps(message), destination)
        self.provider.send_communication_command(command)

    def _send_parlay(self, destination : int = None) -> None:
        self._log.info(f"Sending Parlay")

        message = new_message(
            category = MessageType.PARLAY.value,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
            position = self.position
        )
        command = SendMessageCommand(json.dumps(message),destination)
        self.provider.send_communication_command(command)

    def _send_assignment(self, destination: int = None) -> None:
        self._log.info(f"Sending assignment to {self.points}")

        message = new_message(
            category = MessageType.ASSIGNMENT.value,
            waypoints = self._mission._current_mission,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id()
        )
        command = SendMessageCommand(json.dumps(message),destination)
        self.provider.send_communication_command(command)

    def _request_data(self) -> None:
        self._log.info(f"Requesting data")

        message = new_message(
            category = MessageType.DATAREQ.value,
            sender_type = SimpleSender.UAV.value,
            sender = self.provider.get_id(),
        )
        command = BroadcastMessageCommand(json.dumps(message))
        self.provider.send_communication_command(command)
        self.provider.schedule_timer("", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        if not self.retired:
            self._send_beacon()
            
    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.position = telemetry.current_position
        try:
            if squared_distance(self.position, self._mission._current_mission[self._mission._current_waypoint]) < getRange()/10 and not self.retired:
                self.provider.cancel_timer("")
                if not self.gotData:
                    self._request_data()
                else:
                    self.next_waypoints = self._mission._current_mission[self._mission._current_waypoint+1:]
                    self._mission.stop_mission()
                    self._mission.start_mission(self.next_waypoints)
                    self._log.info(f"Next Waypoint {self._mission._current_mission[self._mission._current_waypoint]}")
                    self.gotData = False
                    self._send_beacon()
                self.away = True
        except TypeError:
            pass
    
    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.BEACON.value:
                self._log.info(report_Bmessage(received_message))
                if received_message['sender_type'] == SimpleSender.SENSOR.value:
                    try:
                        self.next_waypoints = [received_message['position']]+self._mission._current_mission[self._mission._current_waypoint:]
                        self._mission.stop_mission()
                        self._mission.start_mission(self.next_waypoints)
                        self._send_copy(received_message['sender'])
                    except TypeError:
                        pass
                if received_message['sender_type'] == SimpleSender.UAV.value:
                    if self.assigned:
                        if self._mission._current_mission == received_message['waypoints']:
                            if received_message['sender'] != self.parent and received_message['sender'] != self.left and received_message['sender'] != self.right:
                                if self.left:
                                    self._send_halt(self.left)
                                if self.right:
                                    self._send_halt(self.right)
                                if self.deltaId > 0:
                                    self._send_halt(self.parent)
                                    self._send_parlay(received_message['sender'])
                                    self._mission.stop_mission()
                                else:
                                    if self.deltaSize > received_message['count'] or (self.deltaSize == received_message['count'] and self.provider.get_id() < received_message['sender']):
                                        self._send_assignment(received_message['sender'])
                                

        elif received_message['category'] == MessageType.DATA.value:
            self._log.info(report_Dmessage(received_message))
            if received_message['sender_type'] == SimpleSender.SENSOR.value:
                self.sensor_count += 1
                self.gotData = True
        elif received_message['category'] == MessageType.ASSIGNMENT.value:
            if not self.assigned and received_message['sender_type'] == SimpleSender.GROUND_STATION.value:
                received_message: Message = json.loads(message)
                self._log.info(report_Amessage(received_message))
                self._mission.start_mission(received_message['waypoints'])
                self.assigned = True
                self.deltaId = 0
                self.deltaSize = 1
        elif received_message['category'] == MessageType.DATAREQ.value:
            if received_message['sender_type'] == SimpleSender.SENSOR.value:
                self._send_data(received_message['sender'])
            elif received_message['sender_type'] == SimpleSender.GROUND_STATION.value:
                missing_waypoints = len(self._mission._current_mission)
                if missing_waypoints == 1 or self._mission.current_waypoint == missing_waypoints-1:
                    self._send_data(received_message['sender'])
        elif received_message['category'] == MessageType.COPY.value:
            if received_message['sender_type'] == SimpleSender.GROUND_STATION.value:
                self.retired = True
                self.provider.cancel_timer("")

    def finish(self) -> None:
        self._log.info(f"Final sensor count: {self.sensor_count}")

class SimpleGroundStationProtocol(IProtocol):
    _log: logging.Logger
    sensor_count: list
    points: tuple
    assigned: list

    def initialize(self) -> None:
        self._log = logging.getLogger()
        self._log.info(f"Operation started")
        self.sensor_count = []
        self.points = [(50,50,20),(50,450,20),(450,450,20),(450,50,20),(15,15,0)]
        self.assigned = []
        #self.provider.schedule_timer("", self.provider.current_time() + 1)

    def _send_assignment(self, destination: int = None) -> None:
        self._log.info(f"Sending assignment to {self.points}")

        message = new_message(
            category = MessageType.ASSIGNMENT.value,
            waypoints = self.points,
            sender_type = SimpleSender.GROUND_STATION.value,
            sender = self.provider.get_id()
        )
        command = SendMessageCommand(json.dumps(message),destination)
        self.provider.send_communication_command(command)

    def _send_copy(self, destination: int = None) -> None:
        self._log.info(f"Sending COPY")

        message = new_message(
            category = MessageType.COPY.value,
            sender_type = SimpleSender.GROUND_STATION.value,
            sender = self.provider.get_id(),
        )
        command = SendMessageCommand(json.dumps(message),destination)
        self.provider.send_communication_command(command)

    def find(self, searched: int) -> int:
        for i in range(len(self.assigned)):
            if self.assigned[i] == searched:
                return i
        return -1

    def _request_data(self,destination: int = None) -> None:
        self._log.info(f"Requesting data")

        message = new_message(
            category = MessageType.DATAREQ.value,
            sender_type = SimpleSender.GROUND_STATION.value,
            sender = self.provider.get_id(),
        )
        command = SendMessageCommand(json.dumps(message),destination)
        self.provider.send_communication_command(command)

    def handle_timer(self, timer: str) -> None:
        self._send_assignment()

    def handle_packet(self, message: Message) -> None:
        received_message: Message = json.loads(message)
        if received_message['category'] == MessageType.BEACON.value:
            if received_message['sender'] in self.assigned:
                if not self.sensor_count[self.find(received_message['sender'])] and received_message['away']:
                    self._request_data(received_message['sender'])
            else:
                self._send_assignment(received_message['sender'])
                self.assigned.append(received_message['sender'])
                self.sensor_count.append(False)

        elif received_message['category'] == MessageType.DATA.value:
            if received_message['sender_type'] == SimpleSender.UAV.value:
                self.sensor_count[self.find(received_message['sender'])] = received_message['count']
                self._log.info(report_DataReport(received_message))
                self._send_copy(received_message['sender'])

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        count = 0
        for el in self.sensor_count:
            if el:
                count+=el
        self._log.info(f"Final sensor count: {count}")
