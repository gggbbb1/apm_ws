from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import math
import time
from threading import Thread


master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Make sure the connection is valid
master.wait_heartbeat()

def do_gimbal_pitch_yaw(r, p, y, mode=0):
    """
    
    """
    gimbal_device_id = 1
    not_use = 0
    msg = master.mav.command_long_encode(
        0, 
        0,
        ardupilotmega.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 
        0,
        p, # The MAVLink message ID
        y,
        math.nan,
        math.nan,
        ardupilotmega.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME,
        not_use,
        gimbal_device_id,
    )
    master.mav.send(msg)

def manager_set_pitchyaw(pitch = 0, yaw = 0, is_rates=False):
    """
    GIMBAL_MANAGER_SET_PITCHYAW ( #287 )
    https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin/blob/gimbal-protocol-v2-plugin/mavros_extras/src/plugins/gimbal_control.cpp#L433
    """
    # | ardupilotmega.GIMBAL_MANAGER_FLAGS_RETRACT
    # ardupilotmega.GIMBAL_MANAGER_FLAGS_NEUTRAL
    msg = ardupilotmega.MAVLink_gimbal_manager_set_pitchyaw_message(
        0,
        0,
        flags=32  ,
        gimbal_device_id=1,
        pitch=pitch if not is_rates else math.nan,
        yaw=yaw if not is_rates else math.nan,
        pitch_rate= pitch if is_rates else math.nan,
        yaw_rate= yaw if is_rates else math.nan
    )
    master.mav.send(msg)

def main():
    while True:
        try:
            msg = master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'COMMAND_ACK':
                print(msg.to_dict())
            if msg.get_type() == 'HEARTBEAT':
                print(msg.to_dict())
        except:
            pass
        time.sleep(0.1)

def pitch_request():
    neutral = -90
    retract = 0
    for i in range(2):
        print("send gimbal request")
        manager_set_pitchyaw(-1.2, 0, 0)
        # do_gimbal_pitch_yaw(r=0,p=neutral,y=0)
        time.sleep(0.1)

if __name__ == "__main__":
    time.sleep(1)
    t1 = Thread(target=pitch_request,daemon=True, name="work_t")
    t1.start()
    main()  