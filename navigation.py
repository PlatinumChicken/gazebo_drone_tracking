#!/usr/bin/env python3

import asyncio
from mavsdk import System
from std_msgs.msg import String
import rospy

class Navigation:
    def __init__(self):
        self.detection_info = ""
        self.centerx = 240
        self.centery = 220
        self.height = 100
        self.tolx = 20
        self.toly = 20
        self.tolh = 20
        self.drone = System()

    def detection_cb(self, msg):
        self.detection_info = msg.data

    async def setup_and_takeoff(self):
        await self.drone.connect(system_address="udp://:14540")

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position state is good enough for flying.")
                break

        await self.drone.manual_control.set_manual_control_input(
            float(0), float(0), float(0.5), float(0)
        )

        print("-- Arming")
        await self.drone.action.arm()

        print("-- Taking off")
        await self.drone.action.takeoff()
        await asyncio.sleep(5)

        await self.drone.manual_control.set_manual_control_input(
            float(0), float(0), float(0.5), float(0)
        )

        print("-- Starting manual control")
        await self.drone.manual_control.start_position_control()

        await self.control_loop()

    async def control_loop(self):
        while True:
            if len(self.detection_info) != 0:
                cx, cy, h = self.detection_info.split()
                cx = int(cx)
                cy = int(cy)
                h = int(h)

                yaw = 0
                pitch = 0
                roll = 0
                throttle = 0.5

                if abs(cx - self.centerx) > self.tolx:
                    if cx - self.centerx <= 0:
                        yaw = -0.25
                    else:
                        yaw = 0.25

                if abs(cy - self.centery) > self.toly:
                    if cy - self.centery > 0:
                        throttle = 0.2
                    else:
                        throttle = 0.8

                if abs(h - self.height) > self.tolh:
                    if h - self.height > 0:
                        pitch = -0.3
                    else:
                        pitch = 0.3

                await self.drone.manual_control.set_manual_control_input(
                    float(pitch), float(roll), float(throttle), float(yaw)
                )

            await asyncio.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("detection", anonymous=False)
    navigation = Navigation()
    detection_sub = rospy.Subscriber("detection_info", String, callback=navigation.detection_cb)

    asyncio.run(navigation.setup_and_takeoff())
