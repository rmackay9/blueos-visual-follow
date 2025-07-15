#!/usr/bin/env python3

"""
Visual Follow MAVLink Message Module

This module handles sending MAVLink messages to the vehicle
via BlueOS MAV2Rest API interface.
"""

import time
import logging
import json
from typing import Dict, Any, Optional
import urllib.request

# Get logger
logger = logging.getLogger("visual-follow")

# MAV2Rest endpoint
MAV2REST_ENDPOINT = "http://host.docker.internal:6040"

# MAVLink component ID
MAV_COMP_ID_ONBOARD_COMPUTER = 191  # Component ID for onboard computer systems

# COMMAND_LONG message template for SET_MESSAGE_INTERVAL
COMMAND_LONG_SET_MESSAGE_INTERVAL_TEMPLATE = """{{
  "header": {{
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  }},
  "message": {{
    "type": "COMMAND_LONG",
    "target_system": {target_system},
    "target_component": {target_component},
    "command": {{
      "type": "MAV_CMD_SET_MESSAGE_INTERVAL"
    }},
    "confirmation": 0,
    "param1": {message_id},
    "param2": {interval_us},
    "param3": {param3},
    "param4": {param4},
    "param5": {param5},
    "param6": {param6},
    "param7": {response_target}
  }}
}}"""

# STATUSTEXT message template
STATUSTEXT_TEMPLATE = """{{
  "header": {{
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  }},
  "message": {{
    "type": "STATUSTEXT",
    "severity": {{
      "type": "{severity}"
    }},
    "text": {text_array},
    "id": {id},
    "chunk_seq": {chunk_seq}
  }}
}}"""

# GIMBAL_MANAGER_SET_PITCHYAW message template
GIMBAL_MANAGER_SET_PITCHYAW_TEMPLATE = """{{
  "header": {{
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  }},
  "message": {{
    "type": "GIMBAL_MANAGER_SET_PITCHYAW",
    "target_system": {target_system},
    "target_component": {target_component},
    "flags": {flags},
    "gimbal_device_id": {gimbal_device_id},
    "pitch": {pitch},
    "yaw": {yaw},
    "pitch_rate": {pitch_rate},
    "yaw_rate": {yaw_rate}
  }}
}}"""

# CAMERA_INFORMATION message template
CAMERA_INFORMATION_TEMPLATE = """{{
  "header": {{
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  }},
  "message": {{
    "type": "CAMERA_INFORMATION",
    "time_boot_ms": {time_boot_ms},
    "vendor_name": {vendor_name},
    "model_name": {model_name},
    "firmware_version": {firmware_version},
    "focal_length": {focal_length},
    "sensor_size_h": {sensor_size_h},
    "sensor_size_v": {sensor_size_v},
    "resolution_h": {resolution_h},
    "resolution_v": {resolution_v},
    "lens_id": {lens_id},
    "flags": {flags},
    "cam_definition_version": {cam_definition_version},
    "cam_definition_uri": "{cam_definition_uri}"
  }}
}}"""

# CAMERA_TRACKING_IMAGE_STATUS message template
CAMERA_TRACKING_IMAGE_STATUS_TEMPLATE = """{{
  "header": {{
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  }},
  "message": {{
    "type": "CAMERA_TRACKING_IMAGE_STATUS",
    "tracking_status": {{
      "type": "{tracking_status}"
    }},
    "tracking_mode": {{
      "type": "{tracking_mode}"
    }},
    "target_data": {{
      "type": "{target_data}"
    }},
    "point_x": {point_x},
    "point_y": {point_y},
    "radius": {radius},
    "rec_top_x": {rec_top_x},
    "rec_top_y": {rec_top_y},
    "rec_bottom_x": {rec_bottom_x},
    "rec_bottom_y": {rec_bottom_y}
  }}
}}"""


# send mavlink message using MAV2Rest
def post_to_mav2rest(url: str, data: str) -> Optional[str]:
    """
    Sends a POST request to MAV2Rest with JSON data
    Returns response text if successful, None otherwise
    """
    try:
        jsondata = data.encode("ascii")  # data should be bytes
        req = urllib.request.Request(url, jsondata)
        req.add_header("Content-Type", "application/json")

        with urllib.request.urlopen(req, timeout=5) as response:
            return response.read().decode()
    except Exception as error:
        logger.error(f"post_to_mav2rest: error : {url}: {error}")
        return None


# Low level function to send STATUSTEXT MAVLink message
def send_statustext_msg(sysid: int,
                        text: str,
                        severity: str = "MAV_SEVERITY_INFO",
                        message_id: int = 0,
                        chunk_seq: int = 0) -> Dict[str, Any]:
    """
    Send STATUSTEXT MAVLink message

    Args:
        sysid: System ID to send message (normally 1)
        text: Status text message (max 50 characters)
        severity: Message severity level (MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ALERT,
                 MAV_SEVERITY_CRITICAL, MAV_SEVERITY_ERROR, MAV_SEVERITY_WARNING,
                 MAV_SEVERITY_NOTICE, MAV_SEVERITY_INFO, MAV_SEVERITY_DEBUG)
        message_id: Unique (or wrap around) id for this message (0 for auto)
        chunk_seq: Allows the safe construction of larger messages from chunks

    Returns:
        Dictionary with send results
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "send_statustext_msg:"

    try:
        # Truncate text to 50 characters (MAVLink STATUSTEXT limit)
        if len(text) > 50:
            text = text[:50]
            logger.warning(f"{logging_prefix_str} text truncated to 50 characters")

        # Convert text string to character array as required by MAV2Rest
        # MAVLink STATUSTEXT expects a 50-character array, null-terminated
        text_chars = list(text)
        
        # Pad with null terminators to make it 50 characters total
        while len(text_chars) < 50:
            text_chars.append('\u0000')
        
        # Convert to JSON array format
        text_array = json.dumps(text_chars)

        # Format the STATUSTEXT message
        statustext_data = STATUSTEXT_TEMPLATE.format(
            sysid=sysid,
            component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
            severity=severity,
            text_array=text_array,
            id=message_id,
            chunk_seq=chunk_seq
        )

        # Send message via MAV2Rest
        url = f"{MAV2REST_ENDPOINT}/mavlink"
        response = post_to_mav2rest(url, statustext_data)

        if response is not None:
            logger.debug(f"{logging_prefix_str} STATUSTEXT sent with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER}: '{text}'")
            return {
                "success": True,
                "message": f"STATUSTEXT message sent successfully with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER}",
                "text": text,
                "severity": severity,
                "sysid": sysid,
                "response": response
            }
        else:
            logger.error(f"{logging_prefix_str} failed to send STATUSTEXT")
            return {
                "success": False,
                "message": "MAV2Rest returned no response",
                "network_error": True
            }

    except Exception as e:
        logger.error(f"{logging_prefix_str} unexpected error {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }


# Send GIMBAL_MANAGER_SET_PITCHYAW MAVLink message
def send_gimbal_manager_set_pitchyaw(sysid: int,
                                     target_system: int,
                                     target_component: int,
                                     pitch: float,
                                     yaw: float,
                                     pitch_rate: float = float('nan'),
                                     yaw_rate: float = float('nan'),
                                     flags: int = 0,
                                     gimbal_device_id: int = 0) -> Dict[str, Any]:
    """
    Send GIMBAL_MANAGER_SET_PITCHYAW MAVLink message

    Args:
        sysid: System ID to send message from (normally 1)
        target_system: Target system ID (normally 1)
        target_component: Target component ID (normally 1 for autopilot)
        pitch: Pitch angle in radians (negative = down)
        yaw: Yaw angle in radians (positive = right)
        pitch_rate: Pitch rate in radians/second (NaN to use default)
        yaw_rate: Yaw rate in radians/second (NaN to use default)
        flags: Gimbal manager flags (0 for default)
        gimbal_device_id: Gimbal device ID (0 for primary gimbal)

    Returns:
        Dictionary with send results
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "send_gimbal_manager_set_pitchyaw:"

    try:
        # Format the GIMBAL_MANAGER_SET_PITCHYAW message
        gimbal_data = GIMBAL_MANAGER_SET_PITCHYAW_TEMPLATE.format(
            sysid=sysid,
            component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
            target_system=target_system,
            target_component=target_component,
            flags=flags,
            gimbal_device_id=gimbal_device_id,
            pitch=pitch,
            yaw=yaw,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate
        )

        # Send message via MAV2Rest
        url = f"{MAV2REST_ENDPOINT}/mavlink"
        response = post_to_mav2rest(url, gimbal_data)

        if response is not None:
            logger.debug(f"{logging_prefix_str} GIMBAL_MANAGER_SET_PITCHYAW sent with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER} pitch={pitch:.4f} yaw={yaw:.4f}")
            return {
                "success": True,
                "message": f"GIMBAL_MANAGER_SET_PITCHYAW message sent successfully with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER}",
                "pitch": pitch,
                "yaw": yaw,
                "pitch_rate": pitch_rate,
                "yaw_rate": yaw_rate,
                "target_system": target_system,
                "target_component": target_component,
                "response": response
            }
        else:
            logger.error(f"{logging_prefix_str} failed to send GIMBAL_MANAGER_SET_PITCHYAW")
            return {
                "success": False,
                "message": "MAV2Rest returned no response",
                "network_error": True
            }

    except Exception as e:
        logger.error(f"{logging_prefix_str} unexpected error {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }


# Send CAMERA_INFORMATION MAVLink message
def send_camera_information(sysid: int,
                            vendor_name: str,
                            model_name: str,
                            firmware_version: int,
                            focal_length: float,
                            sensor_size_h: float,
                            sensor_size_v: float,
                            resolution_h: int,
                            resolution_v: int,
                            lens_id: int = 0,
                            flags: int = 0,
                            cam_definition_version: int = 0,
                            cam_definition_uri: str = "") -> Dict[str, Any]:
    """
    Send CAMERA_INFORMATION MAVLink message

    Args:
        sysid: System ID to send message from (normally 1)
        vendor_name: Camera vendor name (up to 32 characters)
        model_name: Camera model name (up to 32 characters)
        firmware_version: Camera firmware version (32 bit value)
        focal_length: Focal length in millimeters
        sensor_size_h: Horizontal sensor size in millimeters
        sensor_size_v: Vertical sensor size in millimeters
        resolution_h: Horizontal image resolution in pixels
        resolution_v: Vertical image resolution in pixels
        lens_id: Reserved for a lens ID (0 for default)
        flags: Camera capability flags (0 for default)
        cam_definition_version: Camera definition version (0 for default)
        cam_definition_uri: Camera definition URI (empty string for default)

    Returns:
        Dictionary with send results
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "send_camera_information:"

    try:
        # Get current time in milliseconds since boot
        time_boot_ms = int(time.time() * 1000)

        # Truncate strings to MAVLink limits
        vendor_name = vendor_name[:32] if len(vendor_name) > 32 else vendor_name
        model_name = model_name[:32] if len(model_name) > 32 else model_name
        cam_definition_uri = cam_definition_uri[:140] if len(cam_definition_uri) > 140 else cam_definition_uri

        # Convert strings to byte arrays for MAVLink
        vendor_name_bytes = json.dumps(list(vendor_name.encode('utf-8').ljust(32, b'\x00')))
        model_name_bytes = json.dumps(list(model_name.encode('utf-8').ljust(32, b'\x00')))

        # Format the CAMERA_INFORMATION message
        camera_info_data = CAMERA_INFORMATION_TEMPLATE.format(
            sysid=sysid,
            component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
            time_boot_ms=time_boot_ms,
            vendor_name=vendor_name_bytes,
            model_name=model_name_bytes,
            firmware_version=firmware_version,
            focal_length=focal_length,
            sensor_size_h=sensor_size_h,
            sensor_size_v=sensor_size_v,
            resolution_h=resolution_h,
            resolution_v=resolution_v,
            lens_id=lens_id,
            flags=flags,
            cam_definition_version=cam_definition_version,
            cam_definition_uri=cam_definition_uri
        )

        # Send message via MAV2Rest
        url = f"{MAV2REST_ENDPOINT}/mavlink"
        response = post_to_mav2rest(url, camera_info_data)

        if response is not None:
            logger.debug(f"{logging_prefix_str} CAMERA_INFORMATION sent with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER} vendor={vendor_name} model={model_name}")
            return {
                "success": True,
                "message": f"CAMERA_INFORMATION message sent successfully with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER}",
                "vendor_name": vendor_name,
                "model_name": model_name,
                "resolution": f"{resolution_h}x{resolution_v}",
                "focal_length": focal_length,
                "response": response
            }
        else:
            logger.error(f"{logging_prefix_str} failed to send CAMERA_INFORMATION")
            return {
                "success": False,
                "message": "MAV2Rest returned no response",
                "network_error": True
            }

    except Exception as e:
        logger.error(f"{logging_prefix_str} unexpected error {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }


# Send CAMERA_TRACKING_IMAGE_STATUS MAVLink message
def send_camera_tracking_image_status(sysid: int,
                                      tracking_status: str,
                                      tracking_mode: str,
                                      target_data: str,
                                      point_x: float,
                                      point_y: float,
                                      radius: float,
                                      rec_top_x: float,
                                      rec_top_y: float,
                                      rec_bottom_x: float,
                                      rec_bottom_y: float) -> Dict[str, Any]:
    """
    Send CAMERA_TRACKING_IMAGE_STATUS MAVLink message

    Args:
        sysid: System ID to send message from (normally 1)
        tracking_status: Current tracking status (e.g., "CAMERA_TRACKING_STATUS_FLAGS_IDLE")
        tracking_mode: Current tracking mode (e.g., "CAMERA_TRACKING_MODE_NONE")
        target_data: Target data type (e.g., "CAMERA_TRACKING_TARGET_DATA_NONE")
        point_x: Current tracked point x value (normalized 0..1, 0 is left, 1 is right)
        point_y: Current tracked point y value (normalized 0..1, 0 is top, 1 is bottom)
        radius: Current tracked radius (normalized 0..1, 0 is image left, 1 is image right)
        rec_top_x: Current tracked rectangle top x value (normalized 0..1)
        rec_top_y: Current tracked rectangle top y value (normalized 0..1)
        rec_bottom_x: Current tracked rectangle bottom x value (normalized 0..1)
        rec_bottom_y: Current tracked rectangle bottom y value (normalized 0..1)

    Returns:
        Dictionary with send results
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "send_camera_tracking_image_status:"

    try:
        # Format the CAMERA_TRACKING_IMAGE_STATUS message
        tracking_data = CAMERA_TRACKING_IMAGE_STATUS_TEMPLATE.format(
            sysid=sysid,
            component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
            tracking_status=tracking_status,
            tracking_mode=tracking_mode,
            target_data=target_data,
            point_x=point_x,
            point_y=point_y,
            radius=radius,
            rec_top_x=rec_top_x,
            rec_top_y=rec_top_y,
            rec_bottom_x=rec_bottom_x,
            rec_bottom_y=rec_bottom_y
        )

        # Send message via MAV2Rest
        url = f"{MAV2REST_ENDPOINT}/mavlink"
        response = post_to_mav2rest(url, tracking_data)

        if response is not None:
            logger.debug(f"{logging_prefix_str} CAMERA_TRACKING_IMAGE_STATUS sent with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER} status={tracking_status} point=({point_x:.3f},{point_y:.3f})")
            return {
                "success": True,
                "message": f"CAMERA_TRACKING_IMAGE_STATUS message sent successfully with SysID {sysid} CompID {MAV_COMP_ID_ONBOARD_COMPUTER}",
                "tracking_status": tracking_status,
                "tracking_mode": tracking_mode,
                "target_data": target_data,
                "point_x": point_x,
                "point_y": point_y,
                "radius": radius,
                "response": response
            }
        else:
            logger.error(f"{logging_prefix_str} failed to send CAMERA_TRACKING_IMAGE_STATUS")
            return {
                "success": False,
                "message": "MAV2Rest returned no response",
                "network_error": True
            }

    except Exception as e:
        logger.error(f"{logging_prefix_str} unexpected error {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }


# Retrieve the latest GIMBAL_DEVICE_ATTITUDE_STATUS message via MAV2Rest
def get_gimbal_attitude(sysid: int) -> Dict[str, Any]:
    """
    Retrieve the latest GIMBAL_DEVICE_ATTITUDE_STATUS message via MAV2Rest

    Args:
        sysid: System ID to query gimbal attitude from

    Returns:
        Dictionary with gimbal attitude data or error information
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "get_gimbal_attitude:"

    try:
        # Request the latest GIMBAL_DEVICE_ATTITUDE_STATUS message from the autopilot
        compid = 1  # autopilot component ID
        url = f"{MAV2REST_ENDPOINT}/mavlink/vehicles/{sysid}/components/{compid}/messages/GIMBAL_DEVICE_ATTITUDE_STATUS"
        req = urllib.request.Request(url)
        req.add_header("Content-Type", "application/json")

        with urllib.request.urlopen(req, timeout=5) as response:
            if response.getcode() == 200:
                response_data = response.read().decode()

                # Parse the response (MAV2Rest returns JSON)
                data = json.loads(response_data)

                # Extract attitude information from the message
                if "message" in data:
                    msg = data["message"]

                    # Extract quaternion components
                    q = msg.get("q", [1.0, 0.0, 0.0, 0.0])

                    # log attitude as quaternion
                    logger.debug(f"{logging_prefix_str} q0:{q[0]:.4f}, q1:{q[1]:.4f}, q2:{q[2]:.5f}, q3:{q[3]:.4f}")

                    return {
                        "success": True,
                        "message": "Gimbal attitude retrieved successfully",
                        "quaternion": {
                            "w": q[0],
                            "x": q[1],
                            "y": q[2],
                            "z": q[3]
                        }
                    }
                else:
                    logger.warning(f"{logging_prefix_str} could not parse GIMBAL_DEVICE_ATTITUDE_STATUS message")
                    return {
                        "success": False,
                        "message": "Invalid gimbal attitude message format",
                        "raw_response": response_data
                    }
            else:
                logger.warning(f"{logging_prefix_str} HTTP error {response.getcode()} retrieving gimbal attitude")
                return {
                    "success": False,
                    "message": f"HTTP error {response.getcode()}",
                    "http_error": True
                }

    except urllib.error.HTTPError as e:
        if e.code == 404:
            logger.warning(f"{logging_prefix_str} GIMBAL_DEVICE_ATTITUDE_STATUS not found")
            return {
                "success": False,
                "message": "Gimbal not found or not publishing attitude data",
                "error": "Gimbal not available"
            }
        else:
            logger.error(f"{logging_prefix_str} could not retrieve GIMBAL_DEVICE_ATTITUDE_STATUS {e.code} - {e.reason}")
            return {
                "success": False,
                "message": f"HTTP error {e.code}: {e.reason}",
                "http_error": True
            }
    except urllib.error.URLError as e:
        logger.error(f"Network error retrieving gimbal attitude: {e.reason}")
        return {
            "success": False,
            "message": f"Network error: {e.reason}",
            "network_error": True
        }
    except Exception as e:
        logger.error(f"{logging_prefix_str} could not retrieve GIMBAL_DEVICE_ATTITUDE_STATUS: {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }


# request GIMBAL_DEVICE_ATTITUDE_STATUS messages at specified rate
def request_gimbal_attitude_status(sysid: int, interval_hz: float) -> Dict[str, Any]:
    """
    Request GIMBAL_DEVICE_ATTITUDE_STATUS messages at specified rate

    Args:
        sysid: System ID to send message to
        interval_hz: Frequency in Hz (1.0 = 1Hz, 0 = disable)

    Returns:
        Dictionary with send results
    """

    logger.debug(f"request_gimbal_attitude_status: requesting GIMBAL_DEVICE_ATTITUDE_STATUS at {interval_hz}Hz")

    return send_set_message_interval(
        sysid=sysid,
        message_id=285,         # GIMBAL_DEVICE_ATTITUDE_STATUS
        interval_hz=interval_hz
    )


# send SET_MESSAGE_INTERVAL command
def send_set_message_interval(sysid: int,  # target system id
                              message_id: int,  # MAVlink message id
                              interval_hz: float) -> Dict[str, Any]:
    """
    Send COMMAND_LONG with MAV_CMD_SET_MESSAGE_INTERVAL to request specific message at given rate

    Args:
        sysid: System ID to send message to (e.g 1 for autopilot)
        message_id: MAVLink message ID to request (e.g 285 for GIMBAL_DEVICE_ATTITUDE_STATUS)
        interval_hz: Frequency in Hz (-1 = disable, 0 = request default rate, 1.0 = 1Hz)

    Returns:
        Dictionary with send results
    """
    logging_prefix_str = "set_message_interval"

    try:
        # Convert frequency to interval in microseconds
        if interval_hz < 0:
            interval_us = -1  # Disable the message
            logger.info(f"{logging_prefix_str}: disabling message {message_id}")
        elif interval_hz == 0:
            interval_us = 0  # Request default rate (0 = default)
            logger.info(f"{logging_prefix_str}: default rate for {message_id}")
        else:
            interval_us = int(1000000 / interval_hz)  # Convert Hz to microseconds
            logger.info(f"{logging_prefix_str}: requesting message ID {message_id} at {interval_hz}Hz ({interval_us}µs interval)")

        # Format the COMMAND_LONG message with SET_MESSAGE_INTERVAL command
        target_compid = 1  # autopilot component ID
        command_data = COMMAND_LONG_SET_MESSAGE_INTERVAL_TEMPLATE.format(
            sysid=sysid,
            component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
            target_system=sysid,
            target_component=target_compid,  # autopilot component ID
            message_id=message_id,
            interval_us=interval_us,
            param3=0.0,        # Unused
            param4=0.0,        # Unused
            param5=0.0,        # Unused
            param6=0.0,        # Unused
            response_target=1  # requestor is target address of message stream
        )

        # Send message via MAV2Rest
        url = f"{MAV2REST_ENDPOINT}/mavlink"
        response = post_to_mav2rest(url, command_data)

        if response is not None:
            logger.info(f"{logging_prefix_str}: sent SET_MESSAGE_INTERVAL to sysid:{sysid} compid:{target_compid} msg_id:{message_id}, interval={interval_hz}Hz)")
            return {
                "success": True,
                "message": f"SET_MESSAGE_INTERVAL sent to sysid:{sysid} compid:{target_compid} msg_id:{message_id}, interval={interval_hz}Hz)",
                "message_id": message_id,
                "interval_hz": interval_hz,
                "interval_us": interval_us,
                "target_component": target_compid,
                "response": response
            }
        else:
            logger.error(f"{logging_prefix_str}: failed to send SET_MESSAGE_INTERVAL to sysid:{sysid} compid:{target_compid} msg_id:{message_id}, interval={interval_hz}Hz)")
            return {
                "success": False,
                "message": "MAV2Rest returned no response",
                "network_error": True
            }

    except Exception as e:
        logger.error(f"{logging_prefix_str}: failed to send SET_MESSAGE_INTERVAL: {str(e)}")
        return {
            "success": False,
            "message": f"Unexpected error: {str(e)}",
            "unexpected_error": True
        }
