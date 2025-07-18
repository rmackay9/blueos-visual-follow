#!/usr/bin/env python3

# Visual Follow Python backend
# Implements these features required by the index.html frontend:
# - Save camera settings including type and RTSP URL
# - Get camera settings including last used settings
# - Save/get visual follow enabled state (persistent across restarts)
# - "Test" button to view the live video and calculate visual follow values
# - "Run" button to enable the visual follow including sending MAVLink messages to the vehicle
# - Status endpoint to check if visual follow is currently running

import logging.handlers
import sys
import asyncio
import time
import math
import cv2
import base64
from math import tan, atan, radians, degrees
from pathlib import Path
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
from fastapi.requests import Request
from fastapi import Query
from typing import Dict, Any

# Import the local modules
from app import settings
from app import image_capture
from app import mavlink_interface
from app import tracking

# Configure console logging
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
console_handler.setFormatter(console_formatter)

# Create logger
logger = logging.getLogger("visual-follow")
logger.setLevel(logging.INFO)
logger.addHandler(console_handler)

app = FastAPI()


# Global exception handler to ensure all errors return JSON
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.exception(f"Unhandled exception in {request.url}: {str(exc)}")
    return JSONResponse(
        status_code=500,
        content={
            "success": False,
            "message": f"Internal server error: {str(exc)}",
            "error": "Internal server error"
        }
    )

# Global variables
visualfollow_running = False  # True if visual follow is currently running
gimbal_down_q = [0.7071, 0, -0.7071, 0]  # Quaternion used to check for downward facing gimbal

# log that the backend has started
logger.info("Visual Follow backend started")


# Auto-start visual follow if it was previously enabled
async def startup_auto_restart():
    """Check if visual follow was previously enabled and auto-restart if needed"""

    # logging prefix for all messages from this function
    logging_prefix_str = "visual-follow:"

    try:
        enabled = settings.get_visualfollow_enabled()
        if enabled:
            logger.info(f"{logging_prefix_str} auto-restarting")

            # Get last used settings for auto-restart
            last_used = settings.get_last_used()
            camera_type = last_used.get("camera_type")
            rtsp_url = last_used.get("rtsp")
            if not camera_type or not rtsp_url:
                logger.error(f"{logging_prefix_str} auto-restart failed, could not retrieve camera settings")
                return

            # call startup function in a background thread
            asyncio.create_task(start_visualfollow_internal(camera_type, rtsp_url))

    except Exception as e:
        logger.error(f"{logging_prefix_str} error during auto-restart: {str(e)}")


# Internal function to start visual follow main loop
async def start_visualfollow_internal(camera_type: str, rtsp_url: str):
    """Internal function to start the visual follow process"""
    global visualfollow_running

    # logging prefix for all messages from this function
    logging_prefix_str = "visual-follow:"

    # Initialize target_system_id with default value in case of early exceptions
    target_system_id = 1

    try:
        logger.info(f"{logging_prefix_str} started")
        visualfollow_running = True

        # Send status message to ground station
        send_text_result = mavlink_interface.send_statustext_msg(settings.get_mavlink_sysid(), "VisualFollow: started")
        if not send_text_result["success"]:
            logger.error(f"{logging_prefix_str} MAV2Rest connection failed: {send_text_result['message']}")
            visualfollow_running = False
            return

        # Get camera setting
        camera_hfov = settings.get_camera_horizontal_fov(camera_type)

        # Get MAVLink target system ID from settings
        target_system_id = settings.get_mavlink_sysid()

        # Get gimbal attitude usage setting
        use_gimbal_attitude = settings.get_gimbal_attitude_settings()

        # log settings used
        logger.info(f"{logging_prefix_str} Camera type: {camera_type}, HFOV: {camera_hfov}, RTSP:{rtsp_url}, SysId:{target_system_id}, UseGimbalAttitude:{use_gimbal_attitude}")

        # Testing capturing frame from RTSP stream
        frame_result = image_capture.capture_frame_from_stream(rtsp_url)
        if not frame_result["success"]:
            mavlink_interface.send_statustext_msg(settings.get_mavlink_sysid(), "VisualFollow: failed to capture image, stopping", "MAV_SEVERITY_ERROR")
            logger.error(f"{logging_prefix_str} failed to capture frame: {frame_result['message']}")
            visualfollow_running = False
            return

        # get gimbal attitude, if not available request it
        gimbal_result = mavlink_interface.get_gimbal_attitude(target_system_id)
        if not gimbal_result["success"]:
            # request gimbal attitude status at 1hz
            mavlink_interface.request_gimbal_attitude_status(target_system_id, 1)
            logger.info(f"{logging_prefix_str} Requested GIMBAL_DEVICE_ATTITUDE_STATUS at 1hz")

        # Send status text that visual follow is running
        mavlink_interface.send_statustext_msg(target_system_id, "VisualFollow: running")

        # main loop
        last_frame_time = time.time()
        last_log_time = last_frame_time
        last_send_time = last_frame_time
        frame_count = 0
        sent_count = 0

        # Track last frame capture time to avoid processing the same frame multiple times
        last_frame_capture_time = 0.0

        while visualfollow_running:
            try:
                # Start timing for the entire loop iteration
                loop_start_time = time.time()

                # check for tracking commands from user
                command_start_time = time.time()
                check_for_tracking_commands(target_system_id)
                command_check_time = time.time() - command_start_time

                # Capture frame from RTSP stream
                frame_start_time = time.time()
                frame_result = image_capture.capture_frame_from_stream(rtsp_url, last_frame_capture_time)
                frame_capture_time = time.time() - frame_start_time

                if not frame_result["success"]:
                    logger.warning(f"{logging_prefix_str} failed to capture frame: {frame_result['message']}")

                    # if no frames captured in 10 seconds, restart the capture
                    if time.time() - last_frame_time > 10:
                        mavlink_interface.send_statustext_msg(settings.get_mavlink_sysid(), "VisualFollow: restarting image capture", "MAV_SEVERITY_ERROR")
                        logger.error(f"{logging_prefix_str} No frames captured in 10 seconds, restarting capture")
                        image_capture.cleanup_video_capture()
                        last_frame_time = time.time()

                    # Wait 0.01 seconds before retrying
                    await asyncio.sleep(0.01)
                    continue

                # Get frame capture time and check if it's new
                last_frame_capture_time = frame_result.get("capture_time", time.time())

                # record success
                last_frame_time = time.time()
                frame_count += 1

                # Get the captured frame for potential ROI processing
                frame = frame_result["frame"]
                width = frame_result["width"]
                height = frame_result["height"]

                # Check if we should use gimbal attitude and if gimbal is facing downward
                # Defaults to sending target if gimbal attitude is unavailable
                should_send_target = True
                gimbal_check_time = 0.0
                if use_gimbal_attitude:
                    gimbal_start_time = time.time()
                    gimbal_result = mavlink_interface.get_gimbal_attitude(target_system_id)
                    gimbal_check_time = time.time() - gimbal_start_time

                    if gimbal_result["success"]:
                        # Use quaternion to check if gimbal is facing downward within 10 degrees
                        gimbal_attitude_dict = gimbal_result["quaternion"]
                        # Convert dictionary format to array format [w, x, y, z]
                        gimbal_attitude_q = [
                            gimbal_attitude_dict["w"],
                            gimbal_attitude_dict["x"],
                            gimbal_attitude_dict["y"],
                            gimbal_attitude_dict["z"]
                        ]
                        angle_diff_rad = angle_between_quaternions(gimbal_down_q, gimbal_attitude_q)
                        angle_diff_deg = degrees(angle_diff_rad)
                        logger.debug(f"{logging_prefix_str} Gimbal attitude angle_diff:{angle_diff_deg:.1f} deg")
                        if angle_diff_rad > radians(10):
                            should_send_target = False
                            logger.debug(f"{logging_prefix_str} Gimbal attitude angle_diff:{angle_diff_deg:.1f} > 10, skipping target")
                    else:
                        # If we can't get gimbal attitude but it's required, send anyway
                        logger.warning(f"{logging_prefix_str} gimbal attitude unavailable, sending anyway")

                tracking_time = 0.0
                mavlink_send_time = 0.0
                command_check_time = 0.0

                if should_send_target:
                    # Perform visual follow calculation
                    tracking_start_time = time.time()
                    tracking_result = tracking.get_tracking(frame, False)
                    tracking_time = time.time() - tracking_start_time

                    if tracking_result.get("success"):
                        # calculate vfov
                        camera_vfov = calculate_vertical_fov(camera_hfov, width, height)

                        # get center of tracked object and convert to pitch and yaw angles
                        center_x = tracking_result.get("center_x", 0.5)  # 0.5 is center
                        center_y = tracking_result.get("center_y", 0.5)  # 0.5 is center
                        
                        # Convert from 0-1 range to -1 to +1 range for angle calculations
                        center_x_normalized = (center_x - 0.5) * 2  # Convert 0-1 to -1 to +1
                        center_y_normalized = (center_y - 0.5) * 2  # Convert 0-1 to -1 to +1
                        
                        pitch_angle_rad = radians(center_y_normalized * camera_vfov * 0.5)
                        yaw_angle_rad = radians(center_x_normalized * camera_hfov * 0.5)

                        logger.debug(f"{logging_prefix_str} Frame:{frame_count} "
                                        f"center_x={center_x:.4f}, center_y={center_y:.4f} "
                                        f"pitch={degrees(pitch_angle_rad):.2f}, yaw={degrees(yaw_angle_rad):.2f}")

                        # Send GIMBAL_MANAGER_PITCH_YAW message
                        mavlink_start_time = time.time()
                        send_result = mavlink_interface.send_gimbal_manager_set_pitchyaw(
                            sysid=target_system_id,   # system ID of the target vehicle
                            pitch_rad=pitch_angle_rad,    # pitch angle in radians
                            yaw_rad=yaw_angle_rad,        # yaw angle in radians
                        )
                        mavlink_send_time = time.time() - mavlink_start_time

                        if send_result["success"]:
                            sent_count += 1
                            
                            # Send tracking status update
                            mavlink_interface.send_camera_tracking_image_status(
                                sysid=target_system_id,
                                tracking_status="CAMERA_TRACKING_STATUS_FLAGS_ACTIVE",
                                tracking_mode="CAMERA_TRACKING_MODE_RECTANGLE",
                                target_data="CAMERA_TRACKING_TARGET_DATA_NONE",
                                point_x=center_x,  # Already in 0..1 range
                                point_y=center_y,  # Already in 0..1 range
                                radius=0.1,
                                rec_top_x=0.0,
                                rec_top_y=0.0,
                                rec_bottom_x=1.0,
                                rec_bottom_y=1.0
                            )
                        else:
                            logger.error(f"{logging_prefix_str} Failed to send GIMBAL_MANAGER_SET_PITCHYAW: {send_result['message']}")

                        # record last send time
                        last_send_time = time.time()
                    else:
                        logger.debug(f"{logging_prefix_str} tracking failed: {tracking_result.get('message', 'unknown error')}")
                        
                        # Send tracking lost status
                        mavlink_interface.send_camera_tracking_image_status(
                            sysid=target_system_id,
                            tracking_status="CAMERA_TRACKING_STATUS_FLAGS_IDLE",
                            tracking_mode="CAMERA_TRACKING_MODE_NONE",
                            target_data="CAMERA_TRACKING_TARGET_DATA_NONE",
                            point_x=0.5,
                            point_y=0.5,
                            radius=0.0,
                            rec_top_x=0.0,
                            rec_top_y=0.0,
                            rec_bottom_x=1.0,
                            rec_bottom_y=1.0
                        )
                else:
                    # log that we are waiting for gimbal to face downwards
                    logger.debug(f"{logging_prefix_str} waiting for gimbal to face downwards")

                # Calculate total loop time and log timing details
                loop_total_time = time.time() - loop_start_time

                # Log detailed timing every 10 frames or if any stage takes unusually long
                if frame_count % 10 == 0 or loop_total_time > 0.05:
                    logger.debug(f"{logging_prefix_str} TIMING Frame:{frame_count} "
                                 f"Total:{loop_total_time*1000:.1f}ms "
                                 f"FrameCapture:{frame_capture_time*1000:.1f}ms "
                                 f"GimbalCheck:{gimbal_check_time*1000:.1f}ms "
                                 f"CommandCheck:{command_check_time*1000:.1f}ms "
                                 f"Tracking:{tracking_time*1000:.1f}ms "
                                 f"MAVLinkSend:{mavlink_send_time*1000:.1f}ms")

                # log every 5 seconds
                current_time = time.time()
                if current_time - last_log_time > 5:
                    update_rate_hz = frame_count / (current_time - last_log_time)
                    avg_loop_time = (current_time - last_log_time) / frame_count * 1000  # ms
                    logger.info(f"{logging_prefix_str} rate:{update_rate_hz:.1f}hz frames:{frame_count} MsgsSent:{sent_count} "
                                f"AvgLoopTime:{avg_loop_time:.1f}ms")
                    last_log_time = current_time
                    frame_count = 0
                    sent_count = 0

                # sleep to reduce CPU load
                # long sleep if gimbal is not vertical
                sleep_time = 0.01
                if current_time - last_send_time > 10:
                    sleep_time = 0.8
                await asyncio.sleep(sleep_time)

            except Exception as e:
                logger.error(f"{logging_prefix_str} loop error {str(e)}")
                await asyncio.sleep(1)  # Wait 1 second before retrying

    except Exception as e:
        logger.error(f"{logging_prefix_str} error {str(e)}")
        mavlink_interface.send_statustext_msg(target_system_id, "VisualFollow: error")
    finally:
        # Clean up video capture when stopping
        image_capture.cleanup_video_capture()
        visualfollow_running = False
        mavlink_interface.send_statustext_msg(target_system_id, "VisualFollow: stopped")
        logger.info(f"{logging_prefix_str} stopped")


# Check for incoming camera tracking commands
def check_for_tracking_commands(target_system_id: int):
    """Check for tracking commands from the user"""

    # logging prefix for all messages from this function
    logging_prefix_str = "check_for_tracking_commands:"

    # poll for new tracking commands
    command_result = mavlink_interface.get_camera_tracking_commands(target_system_id)

    if command_result["success"] and command_result["command"] is not None:
        try:
            # Handle different command types directly
            if command_result["command"] == "TRACK_POINT":
                # Extract point coordinates and call set_point_normalised
                point_x = command_result.get("point_x", 0.5)
                point_y = command_result.get("point_y", 0.5)
                radius = command_result.get("radius", 0.1)

                tracking.set_point_normalised((point_x, point_y, radius))
                logger.info(f"{logging_prefix_str} Set point tracking: ({point_x:.3f}, {point_y:.3f}), radius={radius:.3f}")

            elif command_result["command"] == "TRACK_RECTANGLE":
                # Extract rectangle coordinates and call set_rectangle_normalised
                top_left_x = command_result.get("top_left_x", 0.45)
                top_left_y = command_result.get("top_left_y", 0.45)
                bottom_right_x = command_result.get("bottom_right_x", 0.55)
                bottom_right_y = command_result.get("bottom_right_y", 0.55)

                tracking.set_rectangle_normalised((top_left_x, top_left_y, bottom_right_x, bottom_right_y))
                logger.info(f"{logging_prefix_str} Set rectangle tracking: ({top_left_x:.3f}, {top_left_y:.3f}) to ({bottom_right_x:.3f}, {bottom_right_y:.3f})")

            # Send success messages
            mavlink_interface.send_statustext_msg(target_system_id, f"VisualFollow: tracking {command_result['command'].lower()}")

            # Send command acknowledgment
            mavlink_interface.send_command_ack(target_system_id, f"MAV_CMD_CAMERA_{command_result['command']}", "MAV_RESULT_ACCEPTED")
                
        except Exception as e:
            logger.error(f"{logging_prefix_str} Error processing tracking command: {str(e)}")
            mavlink_interface.send_command_ack(target_system_id, f"MAV_CMD_CAMERA_{command_result['command']}", "MAV_RESULT_DENIED")


# Test RTSP connection using OpenCV with FFMPEG backend
# Called from index.html's Test button
def test_rtsp_connection(rtsp_url: str, camera_type: str) -> Dict[str, Any]:
    """
    Test RTSP connection and capture a frame and perform tracking.
    Returns connection status and basic stream information.
    """

    try:
        # Capture single frame from RTSP stream
        frame_result = image_capture.capture_frame_from_stream(rtsp_url)

        if not frame_result["success"]:
            return {
                "success": False,
                "message": f"RTSP connection failed: {frame_result['message']}",
                "error": frame_result.get("error", "Frame capture failed")
            }

        # Get frame data
        frame = frame_result["frame"]
        width = frame_result["width"]
        height = frame_result["height"]

        # Test tracking calculation
        tracking_result = tracking.get_tracking(frame, True)

        # Encode frame as base64 (use augmented image if available)
        if tracking_result.get("success") and tracking_result.get("image_base64"):
            image_base64 = tracking_result["image_base64"]
        else:
            # Fall back to original frame
            _, buffer = cv2.imencode('.jpg', frame)
            image_base64 = base64.b64encode(buffer).decode('utf-8')

        return {
            "success": True,
            "message": f"RTSP connection successful ({width}x{height}). Method: {rtsp_url}",
            "connection_method": rtsp_url,
            "resolution": f"{width}x{height}",
            "image_base64": image_base64,
            "tracking": tracking_result
        }

    except Exception as e:
        logger.exception(f"test_rtsp_connection: exception {str(e)}")
        return {
            "success": False,
            "message": f"Error testing RTSP connection: {str(e)}. Method: {rtsp_url}",
            "error": str(e)
        }


# helper function to calculate the vertical FOV based on the horizontal FOV, image width, and height (in pixels)
def calculate_vertical_fov(hfov_deg: float, width: int, height: int) -> float:
    """Calculate vertical FOV based on horizontal FOV, image width, and height
       tan(vfov/2) = tan(hfov/2) * (height/width)
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "calculate_vertical_fov:"

    # Validate inputs to prevent mathematical errors
    if width <= 0:
        logger.error(f"{logging_prefix_str} invalid image width: {width}")
        return 0.0

    if height <= 0:
        logger.error(f"{logging_prefix_str} invalid image height: {height}")
        return 0.0

    if hfov_deg <= 0 or hfov_deg >= 180:
        logger.error(f"{logging_prefix_str} invalid horizontal FOV: {hfov_deg} (must be between 0 and 180)")
        return 0.0

    try:
        # Convert horizontal FOV from degrees to radians
        hfov_rad = radians(hfov_deg)

        # Calculate aspect ratio
        aspect_ratio = height / width

        # Use trigonometric relationship to calculate vertical FOV
        vfov_rad = 2 * atan(tan(hfov_rad / 2) * aspect_ratio)

        # Convert back to degrees
        vfov_deg = degrees(vfov_rad)

        # Sanity check result
        if vfov_deg <= 0 or vfov_deg >= 180:
            logger.error(f"{logging_prefix_str} calculated invalid VFOV: {vfov_deg}")
            return 0.0

        return vfov_deg
    except (ValueError, OverflowError) as e:
        logger.error(f"{logging_prefix_str} mathematical error calculating VFOV: {e}")
        return 0.0


# calculate the angle in radians between two quaternions
def angle_between_quaternions(q1, q2):
    # Ensure both quaternions are unit length
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, max(-1.0, dot))  # Clamp for acos
    return 2 * math.acos(dot)  # in radians


# Visual Follow API Endpoints

# Get the list of available camera configs (RTSP and FOV)
@app.get("/visual-follow/camera-configs")
async def get_camera_configs() -> Dict[str, Any]:
    """Return the list of available camera configs (RTSP and FOV)"""
    logger.debug("Getting camera configs")

    try:
        settings_dict = settings.get_settings()
        cameras = settings_dict.get('cameras', {})
        return {"success": True, "camera_configs": cameras}
    except Exception as e:
        logger.error(f"Error getting camera configs: {str(e)}")
        return {"success": False, "message": str(e)}


# Load settings
@app.post("/visual-follow/get-settings")
async def get_settings() -> Dict[str, Any]:
    """Get saved camera settings"""
    logger.debug("Getting all settings")

    try:
        # Get the last used camera settings
        last_used = settings.get_last_used()

        # Get RTSP URLs and FOV values for all camera types
        cameras = {}
        for camera_type in ["siyi-a8", "siyi-zr10", "siyi-zt6-ir", "siyi-zt6-rgb"]:
            rtsp_url = settings.get_camera_rtsp(camera_type)
            horizontal_fov = settings.get_camera_horizontal_fov(camera_type)
            cameras[camera_type] = {
                "rtsp": rtsp_url,
                "horizontal_fov": horizontal_fov
            }

        # Get MAVLink settings
        mavlink_settings = {
            "flight_controller_sysid": settings.get_mavlink_sysid()
        }

        # Get gimbal settings
        gimbal_settings = {
            "use_gimbal_attitude": settings.get_gimbal_attitude_settings()
        }

        return {
            "success": True,
            "last_used": last_used,
            "cameras": cameras,
            "mavlink": mavlink_settings,
            "gimbal_attitude": gimbal_settings
        }
    except Exception as e:
        logger.exception(f"Error getting all settings: {str(e)}")
        return {"success": False, "message": f"Error: {str(e)}"}


# Save all settings
@app.post("/visual-follow/save-settings")
async def save_settings(
    type: str = Query(...),
    rtsp: str = Query(...),
    fov: float = Query(...),
    flight_controller_sysid: int = Query(...),  # Keep this for HTML compatibility
    use_gimbal_attitude: bool = Query(True)     # Default to True
) -> Dict[str, Any]:
    """Save all settings to persistent storage (using query parameters)"""
    # Map flight_controller_sysid to sysid for internal use
    sysid = flight_controller_sysid
    logger.info(f"Saving settings: camera_type={type}, rtsp_url={rtsp}, fov={fov}, "
                f"sysid={sysid}, use_gimbal_attitude={use_gimbal_attitude}")

    # Save camera settings
    camera_success = settings.update_camera_settings(type, rtsp, fov)

    # Save MAVLink settings
    mavlink_success = settings.update_mavlink_sysid(sysid)

    # Save gimbal attitude settings
    gimbal_success = settings.update_gimbal_attitude_settings(use_gimbal_attitude)

    if camera_success and mavlink_success and gimbal_success:
        return {"success": True, "message": f"Settings saved for {type}"}
    else:
        return {"success": False, "message": "Failed to save some settings"}


# Get enabled state
@app.get("/visual-follow/get-enabled-state")
async def get_visualfollow_enabled_state() -> Dict[str, Any]:
    """Get enabled state"""
    logger.debug("Getting enabled state")

    try:
        enabled = settings.get_visualfollow_enabled()
        return {
            "success": True,
            "enabled": enabled
        }
    except Exception as e:
        logger.exception(f"Error getting enabled state: {str(e)}")
        return {"success": False, "message": f"Error: {str(e)}", "enabled": False}


# Save enabled state
@app.post("/visual-follow/save-enabled-state")
async def save_enabled_state(enabled: bool = Query(...)) -> Dict[str, Any]:
    """Save enabled state to persistent storage (using query parameter)"""
    logger.info(f"Enabled state: {enabled}")
    success = settings.update_visualfollow_enabled(enabled)

    if success:
        return {"success": True, "message": f"Enabled state saved: {enabled}"}
    else:
        return {"success": False, "message": "Failed to save enabled state"}


# Test image retrieval from the RTSP stream
@app.post("/visual-follow/test")
async def test(type: str = Query(...), rtsp: str = Query(...)) -> Dict[str, Any]:
    """Test RTSP connection"""
    logger.info(f"Testing with camera_type={type}, rtsp={rtsp}")

    try:
        # Run the RTSP connection test in a thread to avoid blocking
        def run_test():
            return test_rtsp_connection(rtsp, type)

        # Run the test in an executor to avoid blocking the async loop
        import concurrent.futures
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(run_test)
            result = future.result(timeout=60)  # 60 second timeout should be sufficient

        if result["success"]:
            logger.info(f"Test successful for {type}: {result['message']}")
            # Add camera type to the response
            result["camera_type"] = type
            result["rtsp_url"] = rtsp
        else:
            logger.warning(f"Test failed for {type}: {result['message']}")

        return result

    except concurrent.futures.TimeoutError:
        logger.error(f"Test timed out for {type} camera")
        return {
            "success": False,
            "message": "Test timed out - unable to connect to camera within 60 seconds",
            "error": "Connection timeout"
        }
    except Exception as e:
        logger.exception(f"Error during test: {str(e)}")
        return {"success": False, "message": f"Test failed: {str(e)}"}


# Get running status
@app.get("/visual-follow/status")
async def get_status() -> Dict[str, Any]:
    """Get running status"""
    logger.debug("Getting status")

    try:
        return {
            "success": True,
            "running": visualfollow_running,
            "message": "Running" if visualfollow_running else "Stopped"
        }
    except Exception as e:
        logger.exception(f"Error getting running status: {str(e)}")
        return {"success": False, "message": f"Error: {str(e)}", "running": False}


# Start visual follow process (this is called by the frontend's "Run" button)
@app.post("/visual-follow/start")
async def start_visualfollow(type: str = Query(...), rtsp: str = Query(...)) -> Dict[str, Any]:
    """Start visual follow process"""
    logger.info(f"Start request received for type={type}, rtsp={rtsp}")

    try:
        if visualfollow_running:
            return {"success": False, "message": "Visual follow is already running"}

        # Start the visual follow process
        asyncio.create_task(start_visualfollow_internal(type, rtsp))

        # Wait a few seconds to catch immediate failures
        await asyncio.sleep(2)

        # Check if it's actually running now
        if visualfollow_running:
            return {
                "success": True,
                "message": f"Visual follow started successfully with {type} camera"
            }
        else:
            return {
                "success": False,
                "message": "Visual follow failed to start (check logs for details)"
            }

    except Exception as e:
        logger.exception(f"Error starting visual follow: {str(e)}")
        return {"success": False, "message": f"Failed to start: {str(e)}"}


# Stop visual follow (this is called by the frontend's "Stop" button)
@app.post("/visual-follow/stop")
async def stop_visualfollow() -> Dict[str, Any]:
    """Stop visual follow"""
    global visualfollow_running

    logger.info("Stop visual follow request received")

    try:
        # Stop the visual follow process
        visualfollow_running = False

        # Clean up video capture when manually stopping
        image_capture.cleanup_video_capture()

        return {
            "success": True,
            "message": "Visual follow stopped successfully"
        }
    except Exception as e:
        logger.exception(f"Error stopping visual follow: {str(e)}")
        return {"success": False, "message": f"Failed to stop: {str(e)}"}


# Set tracking ROI manually (for testing)
@app.post("/visual-follow/set-roi")
async def set_tracking_roi(
    x: float = Query(..., description="Normalized X coordinate (0..1)"),
    y: float = Query(..., description="Normalized Y coordinate (0..1)"), 
    width: float = Query(..., description="Normalized width (0..1)"),
    height: float = Query(..., description="Normalized height (0..1)")
) -> Dict[str, Any]:
    """Set tracking ROI manually for testing"""
    logger.info(f"Manual ROI set: x={x}, y={y}, width={width}, height={height}")
    
    try:
        # Get current frame to initialize tracker
        settings_data = settings.get_last_used()
        rtsp_url = settings_data.get("rtsp")
        
        if not rtsp_url:
            return {"success": False, "message": "No RTSP URL configured"}
        
        # Capture a frame
        frame_result = image_capture.capture_frame_from_stream(rtsp_url)
        if not frame_result["success"]:
            return {"success": False, "message": f"Failed to capture frame: {frame_result['message']}"}
        
        frame = frame_result["frame"]
        img_width = frame_result["width"]
        img_height = frame_result["height"]
        
        # Convert normalized coordinates to pixel coordinates
        pixel_x = int(x * img_width)
        pixel_y = int(y * img_height)
        pixel_width = int(width * img_width)
        pixel_height = int(height * img_height)
        
        # Ensure coordinates are within bounds
        pixel_x = max(0, min(pixel_x, img_width - 1))
        pixel_y = max(0, min(pixel_y, img_height - 1))
        pixel_width = max(1, min(pixel_width, img_width - pixel_x))
        pixel_height = max(1, min(pixel_height, img_height - pixel_y))
        
        roi = (pixel_x, pixel_y, pixel_width, pixel_height)
        
        # Initialize tracker
        init_result = tracking.initialize_tracker(frame, roi)
        
        if init_result["success"]:
            return {
                "success": True,
                "message": f"ROI set successfully: {roi}",
                "roi": roi,
                "normalized": {"x": x, "y": y, "width": width, "height": height}
            }
        else:
            return {
                "success": False,
                "message": f"Failed to initialize tracker: {init_result['message']}"
            }
            
    except Exception as e:
        logger.exception(f"Error setting ROI: {str(e)}")
        return {"success": False, "message": f"Error setting ROI: {str(e)}"}


# Clear tracking ROI
@app.post("/visual-follow/clear-roi")
async def clear_tracking_roi() -> Dict[str, Any]:
    """Clear/reset tracking ROI"""
    logger.info("Clearing tracking ROI")
    
    try:
        tracking.clear_roi()
        return {"success": True, "message": "Tracking ROI cleared"}
    except Exception as e:
        logger.exception(f"Error clearing ROI: {str(e)}")
        return {"success": False, "message": f"Error clearing ROI: {str(e)}"}


# Initialize auto-restart task
@app.on_event("startup")
async def on_startup():
    """Application startup event handler"""
    await startup_auto_restart()


# Mount static files AFTER defining API routes
# Use absolute path to handle Docker container environment
static_dir = Path(__file__).parent / "static"
app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

# Set up logging for the app
log_dir = Path('./logs')  # Use local logs directory instead of /app/logs
log_dir.mkdir(parents=True, exist_ok=True)
fh = logging.handlers.RotatingFileHandler(log_dir / 'lumber.log', maxBytes=2**16, backupCount=1)
logger.addHandler(fh)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
