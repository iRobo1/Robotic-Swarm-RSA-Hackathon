import requests
import websockets
import threading 
import struct
import asyncio
import time
from arena import Basket, Position, Team
LOCATION_MSG = 1
START_MSG = 2
STOP_MSG = 3
OBJECTIVE_FOUND_MSG = 4
CUSTOM_MSG = 5

OUR_BASKET_MSG = 37 # hopefully nobody is going to use it

class Communication:
    def __init__(self, host, team_id, robot_id, password):
        self.host = host
        self.team_id = team_id
        self.robot_id = robot_id
        self.password = password
        self.auth = (str(team_id), password)
        self.location_callback = None
        self.start_callback = None
        self.stop_callback = None
        self.objective_callback = None
        self.custom_callback = None
        self.basket_callback = None
        self._start_listening()

    def register_callback_our_basket(self, func):
        """
        Register a callback for location updates.

        Example:
            def on_receive_basket(basket):
                # team_id (int): Team ID that found the basket (should be us - 5)
                # robot_id (int): Robot ID that found the basket
                # basket: Basket
            
                client.register_callback_basket(on_receive_basket)
        """
        self.basket_callback = func
    

    def register_callback_location(self, func):
        """
        Register a callback for location updates.

        Example:
            def on_receive_location(x, y, angle, visible, last_seen):
                # x (float): X coordinate
                # y (float): Y coordinate
                # angle (float): Orientation angle
                # visible (bool): Whether currently visible
                # last_seen (int): Timestamp of last detection
                print(x, y)

            client.register_callback_location(on_receive_location)
        """
        self.location_callback = func
    

    def register_callback_start(self, func):
        """
        Register a callback for start messages.

        Example:
            def on_receive_start():
                stopped = False # Keep track of the state with a variable
                print("Started")

            client.register_callback_start(on_receive_start)
        """
        self.start_callback = func
        

    def register_callback_stop(self, func):
        """
        Register a callback for stop events.

        Example:
            def on_receive_stop():
                robot.drive(0, 0) # Stop the robot!
                stopped = True
                print("Stopped")

            client.register_callback_stop(on_receive_stop)
        """
        self.stop_callback = func
    

    def register_callback_objective(self, func):
        """
        Register a callback for objective detection events.

        Example:
            def on_objective(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
                # team_id (int): Team ID that found the objective
                # robot_id (int): Robot ID that found the objective
                # tag_id (int): Objective/tag ID
                # x (float): X position of the robot that found the objective
                # y (float): Y position of the robot that found the objective
                # angle (float): Orientation of the robot that found the objective
                # visible (bool): Wether the robot that found the objective is currently visible for the localization system
                # last_seen (int): Time at which the location information was taken
                print(tag_id)

            client.register_callback_objective(on_objective)
        """
        self.objective_callback = func


    def register_callback_custom(self, func):
        """
        Register a callback for custom messages.

        Example:
            def on_custom(team_id, robot_id, internal_type, contents):
                # team_id (int): Sender team ID
                # robot_id (int): Sender robot ID
                # internal_type (int): Custom message type
                # contents (bytes): Raw payload
                print("custom received!", internal_type)
                if internal_type == 12:
                    try: # Don't forget to use a try catch, in case a robot sends a malformed message
                        a, b = struct.unpack("<BB", bytes)
                        print(a,b)
                    except Exception as e:
                        print("Could not parse message")

            client.register_callback_custom(on_custom)
        """
        self.custom_callback = func


    def send_objective_msg(self, target_team_id, tag_id):
        """
        Send an objective message to a target team.

        Example:
            client.send_objective_msg(2, 5)
            # target_team_id (int): Target team ID
            # tag_id (int): Objective/tag identifier

        Returns:
            bool: True if successful, False otherwise.
        """
        packet = struct.pack("<BB", target_team_id, tag_id)
        url = f"http://{self.host}/objective/{self.team_id}/{self.robot_id}"
        try:
            requests.post(url, auth=self.auth, data=packet)
            return True
        except Exception as e:
            print(f"Could not send objective message: {e}")
            return False


    def send_custom_msg(self, target_team_id, target_robot_id, internal_type, contents):
        """
        Send a custom message to a specific robot (or all robots of a team if target_robot_id is 0)

        Example:
            client.send_custom_msg(1, 3, 12, struct.pack("<BB", 123, 45))
            # target_team_id (int): Target team ID
            # target_robot_id (int): Target robot ID (0 to send to all robots in the team)
            # internal_type (int): Custom message type
            # contents (bytes): Payload

        Returns:
            bool: True if successful, False otherwise.
        """
        packet = struct.pack("<BBBB", target_team_id, target_robot_id, 0, internal_type) + contents
        url = f"http://{self.host}/custom/{self.team_id}/{self.robot_id}"
        try:
            requests.post(url, auth=self.auth, data=packet)
            return True
        except Exception as e:
            print(f"Could not send objective message: {e}")
            return False    

    def send_basket(self, basket, target_robot_id=0):
        contents = struct.pack(
            "<ff B ?? f",
            basket.pos.x,
            basket.pos.y,
            basket.team.value,
            basket.scanned,
            basket.item_delivered,
            basket.measurement_distance
        )
        self.send_custom_msg(5, target_robot_id, OUR_BASKET_MSG, contents) 


    def _start_listening(self):
        self._thread = threading.Thread(
            target=lambda: asyncio.run(self._connect_and_listen_to_websocket()),
            daemon=True
        )
        self._thread.start()

    async def _connect_and_listen_to_websocket(self):
        url = f"ws://{self.host}/ws/connect/{self.team_id}/{self.robot_id}?password={self.password}"
        while True:
            try:
                async with websockets.connect(url) as websocket:
                    print("Connected to websocket!")
                    async for packet in websocket:
                        try:
                            message = struct.unpack_from("<B", packet)[0]
                            if message == START_MSG:
                                if self.start_callback is not None:
                                    self.start_callback()
                            elif message == STOP_MSG:
                                if self.stop_callback:
                                    self.stop_callback()
                            elif message == LOCATION_MSG:
                                _, x, y, angle, visible, last_seen = struct.unpack("<Bfff?I", packet)
                                if self.location_callback:
                                    self.location_callback(x, y, angle, visible, last_seen)
                            elif message == OBJECTIVE_FOUND_MSG:
                                _, team_id, robot_id, tag_id, x, y, angle, visible, last_seen = struct.unpack("<BBBBfff?I", packet)
                                if self.objective_callback:
                                    self.objective_callback(team_id, robot_id, tag_id, x, y, angle, visible, last_seen)
                            elif message == CUSTOM_MSG:
                                _, team_id, robot_id, internal_type = struct.unpack("<BBBB", packet[0:4])
                                if internal_type == OUR_BASKET_MSG:
                                    if self.basket_callback:
                                        try:
                                            x, y, team, scanned, item_delivered, measurement_distance = struct.unpack("<ff B ?? f", packet[4:])
                                            basket = Basket(
                                                pos=Position(x, y),
                                                team=Team(team),
                                                measurement_distance=measurement_distance
                                            )
                                            basket.scanned = False
                                            basket.item_delivered = False

                                            print(f"Received basket from robot {robot_id}: {basket}")
                                            self.basket_callback(team_id, robot_id, basket)

                                        except Exception as e:
                                            print(f"Could not parse basket message: {e}")

                                elif self.custom_callback:
                                    self.custom_callback(team_id, robot_id, internal_type, packet[4:])
                            else:
                                print(f"Unknown message type {message} received")
                        except Exception as e:
                            print("Could not parse message")
            except Exception as e:
                print(f"Error/Disconnected: {e}, retrying in 5 seconds...")
                await asyncio.sleep(5)   