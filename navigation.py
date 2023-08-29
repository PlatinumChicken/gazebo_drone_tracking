#!/usr/bin/env python3

import asyncio
from mavsdk import System
from std_msgs.msg import String
import rospy

detection_info=""

def detection_cb(msg):
    global detection_info
    detection_info=msg.data

async def manual_controls():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    while True:
        
        if len(detection_info)!=0:
            cx, cy, h=detection_info.split()
            cx=int(cx)
            cy=int(cy)
            h=int(h)
            print(detection_info)

            yaw=0
            pitch=0
            roll=0
            throttle=0.5

            if abs(cx-229)>20:
                if cx-229<=0:
                    yaw=-0.25
                else:
                    yaw=0.25
        
            if abs(cy-250)>10:
                if cy-250>0:
                    throttle=0.4
                else:
                    throttle=0.6

            if abs(h-100)>10:
                if h-100>0:
                    pitch=-0.15
                else:
                    pitch=0.15
        

            await drone.manual_control.set_manual_control_input(
            float(pitch), float(roll), float(throttle), float(yaw))

        await asyncio.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("detection", anonymous=False)
    detection_sub = rospy.Subscriber("detection_info", String, callback=detection_cb)

    asyncio.run(manual_controls())
    rospy.spin()
    