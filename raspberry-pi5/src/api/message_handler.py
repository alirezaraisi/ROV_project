import asyncio
from .mavlink import mavlink, client, ButtonFunctions
from . import command_handler
from mission_planner import planner, missions
from robot_core import robot
from .joystick import GamepadController

mission_request_retries = 0

perv_send_esp = "5000000500000"
perv_send_robot = "0"
gear = 0.25
gamepadcontroller = GamepadController()

async def command_recv_handler(msg: mavlink.MAVLink_command_long_message | mavlink.MAVLink_command_int_message):
    if msg.command in command_handler.handlers:
        await command_handler.handlers[msg.command](msg)


async def command_ack_handler(msg: mavlink.MAVLink_command_ack_message):
    pass


async def manual_control_handler(msg: mavlink.MAVLink_manual_control_message):
    global perv_send_esp
    global perv_send_robot
    global gear
    #print(f"{msg.buttons:016b}")
    #print(msg)
    #print(f"mode:{mode}")
    #print(f"gamepadcontroller:{gamepadcontroller.combine_data(msg.x, msg.y, msg.r, msg.z, msg.s, msg.t, msg.buttons, msg.buttons2)}")
    try:
        send_esp, send_robot = gamepadcontroller.combine_data(msg.x, msg.y, msg.r, msg.z, msg.s, msg.t, msg.buttons, msg.buttons2)
        robot.gear_yaw = gamepadcontroller.gear_move
        robot.gear_depth = gamepadcontroller.gear_depth
        #print(f"gear : {robot.gear_yaw}")
        #print(f"gear : {robot.gear_depth}")
        print(f"send_robot{send_robot}")
        
        if (perv_send_esp !=send_esp):
            robot.latest_joystick = send_esp
            perv_send_esp = send_esp
        if (perv_send_robot != send_robot):
            robot.button = send_robot
            print(f"ok send robot{send_robot}")
            perv_send_robot = send_robot
        
        if msg.buttons == 193:
            robot.mode = "AUTO"
        elif msg.buttons2 == 2:
            robot.mode = "MANUAL"
    except Exception as e:
        print(f"Error in massage_control_handler: {e}")
        # Fallback to default values in case of error
        robot.latest_joystick = "5000000500000"
                   
                   
async def heartbeat_handler(msg: mavlink.MAVLink_heartbeat_message):
    pass


# THis is just for example you should implement your own!

async def param_request_list_handler(msg: mavlink.MAVLink_param_request_list_message):
    param_count = len(parameters)
    for index, param in enumerate(parameters):
        print(param)
        await client.mav.param_value_send(**param, param_count=param_count, param_index=index)


async def param_request_read_handler(msg: mavlink.MAVLink_param_request_read_message):
    print(f"{msg.target_system} {msg.target_component} requested parameter {msg}")
    param_id = msg.param_id.encode()
    if msg.target_system != client.source_system or (msg.target_component != client.source_component and msg.target_component != mavlink.MAV_COMP_ID_ALL):
        return
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)


async def param_set_handler(msg: mavlink.MAVLink_param_set_message):
    print(msg)
    param_id = msg.param_id.encode()
    for index, param in enumerate(parameters):
        if param['param_id'] == param_id:
            param['param_value'] = msg.param_value
            param['param_type'] = msg.param_type
            await client.mav.param_value_send(**param, param_count=len(parameters), param_index=index)
            break
    else:
        new_object = {'param_id': msg.param_id.encode(
        ), 'param_value': msg.param_value, 'param_type': msg.param_type}
        parameters.append(new_object)
        await client.mav.param_value_send(**new_object, param_count=len(parameters), parama_index=len(parameters)-1)


async def mission_request_list_handler(msg: mavlink.MAVLink_mission_request_int_message):
    await client.mav.mission_count_send(target_system=msg.get_srcSystem(), target_component=msg.get_srcComponent(), count=planner.get_missions_count(), mission_type=0)


async def mission_request_int_handler(msg: mavlink.MAVLink_mission_request_int_message):
    mission = planner.get_mission_item(msg.seq)
    if not mission:
        return await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=msg.seq
        )
        # return await client.mav.mission_ack_send(
        #     target_system=msg.get_srcSystem(),
        #     target_component=msg.get_srcComponent(),
        #     type=mavlink.MAV_MISSION_
        # )
    await client.mav.mission_item_int_send(target_system=msg.get_srcSystem(), target_component=msg.get_srcComponent(), seq=msg.seq, **mission.to_dict())


async def mission_item_int_handler(msg: mavlink.MAVLink_mission_item_int_message):
    global mission_request_retries
    mission_request_retries = 0
    mission = missions.from_message(msg)

    if not mission:
        return await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_UNSUPPORTED
        )

    remaining = planner.store_mission(mission, seq=msg.seq)

    if remaining > 0:
        mission_request_retries = 5
        await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=msg.seq+1
        )
    elif remaining == 0:
        await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_ACCEPTED
        )
    else:
        await client.mav.mission_ack_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            type=mavlink.MAV_MISSION_ERROR,
        )


async def mission_count_handler(msg: mavlink.MAVLink_mission_count_message):
    planner.prepare_mission_download(msg.count)

    if msg.count > 0:
        await client.mav.mission_request_int_send(
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            seq=0
        )


async def mission_ack_handler(msg: mavlink.MAVLink_mission_ack_message):
    print("Mission ACK received,", msg.type)


handlers = {
    mavlink.MAVLINK_MSG_ID_COMMAND_LONG: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_INT: command_recv_handler,
    mavlink.MAVLINK_MSG_ID_COMMAND_ACK: command_ack_handler,
    mavlink.MAVLINK_MSG_ID_HEARTBEAT: heartbeat_handler,
    mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: manual_control_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST: param_request_list_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ: param_request_read_handler,
    mavlink.MAVLINK_MSG_ID_PARAM_SET: param_set_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST: mission_request_list_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_INT: mission_request_int_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: mission_item_int_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_COUNT: mission_count_handler,
    mavlink.MAVLINK_MSG_ID_MISSION_ACK: mission_ack_handler,
}


def btn_functions(id: int, normal_function: ButtonFunctions, shift_function: ButtonFunctions):
    return [
        {"param_id": f"BTN{id}_FUNCTION".encode(
        ), "param_value": normal_function.value, "param_type": 2},
        {"param_id": f"BTN{id}_SFUNCTION".encode(
        ), "param_value": shift_function.value, "param_type": 2},
    ]


# fmt: off
parameters = [
    *btn_functions(0, ButtonFunctions.Shift, ButtonFunctions.Shift),
    *btn_functions(1, ButtonFunctions.ModeManual, ButtonFunctions.KNone),
    *btn_functions(2, ButtonFunctions.ModeDepthHold, ButtonFunctions.ModePoshold),
    *btn_functions(3, ButtonFunctions.ModeStabilize, ButtonFunctions.ModeAcro),
    *btn_functions(4, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(5, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(6, ButtonFunctions.MountTiltDown, ButtonFunctions.Servo1Min),
    *btn_functions(7, ButtonFunctions.MountTiltUp, ButtonFunctions.Servo1Max),
    *btn_functions(8, ButtonFunctions.Disarm, ButtonFunctions.KNone),
    *btn_functions(9, ButtonFunctions.Arm, ButtonFunctions.KNone),
    *btn_functions(10, ButtonFunctions.MountCenter, ButtonFunctions.Relay1Toggle),
    *btn_functions(11, ButtonFunctions.InputHoldSet, ButtonFunctions.KNone),
    *btn_functions(12, ButtonFunctions.GainInc, ButtonFunctions.TrimPitchInc),
    *btn_functions(13, ButtonFunctions.GainDec, ButtonFunctions.TrimPitchDec),
    *btn_functions(14, ButtonFunctions.Lights1Dimmer, ButtonFunctions.TrimRollDec),
    *btn_functions(15, ButtonFunctions.Lights1Brighter, ButtonFunctions.TrimRollInc),
    *btn_functions(16, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(17, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(18, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(19, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(20, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(21, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(22, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(23, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(24, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(25, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(26, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(27, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(28, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(29, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(30, ButtonFunctions.KNone, ButtonFunctions.KNone),
    *btn_functions(31, ButtonFunctions.KNone, ButtonFunctions.KNone),
]
# fmt: on
