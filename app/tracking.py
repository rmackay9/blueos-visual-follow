#!/usr/bin/env python3

"""
Visual tracking module for target following

This module handles visual tracking using OpenCV's TrackerCSRT algorithm
for real-time object tracking in video streams.
"""

import cv2
import numpy as np
import logging
from typing import Dict, Any
import base64

# Get logger
logger = logging.getLogger("visual-follow")

# local variables
tracker = None    # OpenCV TrackerCSRT instance
image_roi_norm = False  # normalised bounding box expressed as a tuple of 4 values in range 0 to 1.  (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
image_roi = None  # roi bounding box expressed as a tuple with 4 integer values in pixels: (x, y, width, height)
                  # x: Left edge of the bounding box (in pixels)
                  # y: Top edge of the bounding box (in pixels)
                  # width: Width of the bounding box (in pixels)
                  # height: Height of the bounding box (in pixels)


# Set tracking point using normalised coordinates
#     accepts a tuple with 3 values: (x, y, radius)
#     x : 0 ~ 1 where 0 is left an 1 is right
#     y : 0 ~ 1 where 0 is top and 1 is bottom
#     radius: 0 ~ 1, 0 is 1 pixel, 1 is full image size
def set_point_normalised(roi: tuple):
    """
    set_point_normalised

    Set tracking target using normalised coordinates
    """
    global image_roi_norm, image_roi

    # validate ROI is a tuple with 3 values in 0 to 1 range named "x", "y", "radius"
    if not isinstance(roi, tuple) or len(roi) != 3:
        logger.error(f"set_point_normalised: Invalid ROI format: {roi}. Expected tuple with 3 values (x, y, radius)")
        return

    # extract x, y and radius from the tuple
    x, y, radius = roi

    # Validate that all values are numbers and in the range 0 to 1
    if not all(isinstance(val, (int, float)) for val in [x, y, radius]):
        logger.error(f"set_point_normalised: Invalid ROI values: {roi}. All values must be numbers")
        return
    if not (0 <= x <= 1 and 0 <= y <= 1 and 0 <= radius <= 1):
        logger.error(f"set_point_normalised: Invalid ROI range: {roi}. All values must be in range 0 to 1")
        return

    # Convert point + radius to normalized rectangle coordinates
    # Calculate bounding box around the point
    half_radius = radius / 2
    top_left_x = max(0, x - half_radius)
    top_left_y = max(0, y - half_radius)
    bottom_right_x = min(1, x + half_radius)
    bottom_right_y = min(1, y + half_radius)

    # record ROI as normalized rectangle (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
    image_roi_norm = (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
    image_roi = None
    logger.info(f"set_point_normalised: ROI set to normalized rectangle: {image_roi_norm}")
    logger.debug(f"set_point_normalised: Original point ({x:.3f}, {y:.3f}) with radius {radius:.3f}")


# Set tracking rectangle using normalised coordinates
#     accepts a tuple with 4 values: (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
#     all values: 0 ~ 1 where 0 is left/top and 1 is right/bottom
def set_rectangle_normalised(roi: tuple):
    """
    set_rectangle_normalised

    Set tracking target using normalised rectangle coordinates
    """
    global image_roi_norm

    # validate ROI is a tuple with 4 values in 0 to 1 range
    if not isinstance(roi, tuple) or len(roi) != 4:
        logger.error(f"set_rectangle_normalised: Invalid ROI format: {roi}. Expected tuple with 4 values (top_left_x, top_left_y, bottom_right_x, bottom_right_y)")
        return

    # extract top_left_x, top_left_y, bottom_right_x, bottom_right_y from the tuple
    top_left_x, top_left_y, bottom_right_x, bottom_right_y = roi

    # Validate that all values are numbers and in the range 0 to 1
    if not all(isinstance(val, (int, float)) for val in [top_left_x, top_left_y, bottom_right_x, bottom_right_y]):
        logger.error(f"set_rectangle_normalised: Invalid ROI values: {roi}. All values must be numbers")
        return
    if not (0 <= top_left_x <= 1 and 0 <= top_left_y <= 1 and 0 <= bottom_right_x <= 1 and 0 <= bottom_right_y <= 1):
        logger.error(f"set_rectangle_normalised: Invalid ROI range: {roi}. All values must be in range 0 to 1")
        return

    # Validate that the rectangle is valid (top-left must be less than bottom-right)
    if top_left_x >= bottom_right_x or top_left_y >= bottom_right_y:
        logger.error(f"set_rectangle_normalised: Invalid rectangle: {roi}. Top-left must be less than bottom-right")
        return

    # record ROI as normalized rectangle (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
    image_roi_norm = (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
    image_roi = None
    logger.info(f"set_rectangle_normalised: ROI set to normalized rectangle: {image_roi_norm}")


# Clear ROI
def clear_roi():
    """
    Clear ROI
    
    This function should be called to stop tracking a target
    """
    global image_roi_norm, image_roi
    image_roi_norm = None
    image_roi = None
    logger.info(f"clear_roi: ROI cleared")


# Run tracking algorithm on new image using OpenCV's TrackerCSRT
def get_tracking(image: np.ndarray, include_augmented_image: bool) -> Dict[str, Any]:
    """
    Run tracking algorithm on the provided image using OpenCV's TrackerCSRT

    Args:
        image: Input image as numpy array (BGR format from OpenCV)
        include_augmented_image: if true an augmented image with a rectangle drawn around the target should be returned to the caller

    Returns:
        Dictionary containing:
        - success: bool indicating if detection was successful
        - center_x: normalised horizontal value of the tracked object (0 is left, 1 is right)
        - center_y: normalised vertical value of the tracked object (0 is top, 1 is bottom)
        - box_x: normalised bounding box top-left corner x coordinate (0 to 1)
        - box_y: normalised bounding box top-left corner y coordinate (0 to 1)
        - box_width: normalised bounding box width (0 to 1)
        - box_height: normalised bounding box height (0 to 1)
        - message: Status message
        - image_base64: Base64 encoded image, None if not requested or could not be generated
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "get_tracking:"

    global tracker

    try:
        # create tracker if not already created
        if tracker is None:
            tracker = cv2.TrackerCSRT_create()
            logger.info(f"{logging_prefix_str} Created new TrackerCSRT instance")

        # exit immediately if no roi
        if image_roi_norm is None:
            logger.debug(f"{logging_prefix_str} ROI not defined")
            return {
                "success": False,
                "center_x": None,
                "center_y": None,
                "box_x": None,
                "box_y": None,
                "box_width": None,
                "box_height": None,
                "message": "ROI not defined",
                "image_base64": None
            }

        # get image dimensions
        image_height, image_width = image.shape[:2]

        # update image_roi from image_roi_norm if necessary
        if image_roi is None:
            # Rectangle format: (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
            top_left_x, top_left_y, bottom_right_x, bottom_right_y = image_roi_norm

            # Convert to pixel coordinates
            x = int(top_left_x * image_width)
            y = int(top_left_y * image_height)
            width = int((bottom_right_x - top_left_x) * image_width)
            height = int((bottom_right_y - top_left_y) * image_height)

            # save to image_roi and send to tracker
            image_roi = (x, y, width, height)
            tracker.init(image, image_roi)

        # run tracking algorithm
        success, box = tracker.update(image)
        if not success:
            logger.warning(f"{logging_prefix_str} Tracking failed")
            return {
                "success": False,
                "center_x": None,
                "center_y": None,
                "box_x":None,
                "box_y": None,
                "box_width": None,
                "box_height": None,
                "message": "Tracking failed",
                "image_base64": None
            }

        # tracking successful, process returned bounding box
        # calculate center of the bounding box
        (box_x, box_y, box_width, box_height) = [int(v) for v in box]
        center_x = box_x + (box_width // 2)
        center_y = box_y + (box_height // 2)

        # normalize coordinates to 0 to 1 range
        norm_center_x = center_x / image_width  # 0 (left) to 1 (right)
        norm_center_y = center_y / image_height  # 0 (top) to 1 (bottom)
        norm_box_x = box_x / image_width
        norm_box_y = box_y / image_height
        norm_box_width = box_width / image_width
        norm_box_height = box_height / image_height

        # Create augmented image
        image_base64 = None
        if include_augmented_image:
            try:
                # Create a copy of the original image for visualization
                augmented_image = image.copy()

                # Draw the tracking bounding box
                cv2.rectangle(augmented_image, (box_x, box_y), 
                            (box_x + box_width, box_y + box_height), 
                            (0, 255, 0), 2)  # Green rectangle

                # Draw center point
                cv2.circle(augmented_image, (center_x, center_y), 5, (0, 0, 255), -1)  # Red center point

                # Draw crosshair at center
                cv2.line(augmented_image, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
                cv2.line(augmented_image, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)

                # Add text overlay with tracking info
                text = f"Tracking: {box_width}x{box_height}"
                cv2.putText(augmented_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                coord_text = f"Center: ({center_x}, {center_y})"
                cv2.putText(augmented_image, coord_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Encode image to base64
                _, buffer = cv2.imencode('.jpg', augmented_image)
                image_base64 = base64.b64encode(buffer).decode('utf-8')

            except Exception as e:
                logger.warning(f"{logging_prefix_str} Failed to create augmented image: {str(e)}")
                image_base64 = None

        # return success
        return {
            "success": True,
            "message": "Success",
            "center_x": norm_center_x,
            "center_y": norm_center_y,
            "box_x": norm_box_x,
            "box_y": norm_box_y,
            "box_width": norm_box_width,
            "box_height": norm_box_height,
            "image_base64": image_base64
        }

    except Exception as e:
        logger.exception(f"Error during tracking calculation: {str(e)}")
        return {
            "success": False,
            "message": f"Tracking calculation failed: {str(e)}",
            "center_x": None,
            "center_y": None,
            "box_x": None,
            "box_y": None,
            "box_width": None,
            "box_height": None,
            "image_base64": None
        }


# Initialize the tracker with a specific ROI
def initialize_tracker(image: np.ndarray, roi: tuple) -> Dict[str, Any]:
    """
    Initialize the tracker with a specific ROI
    
    Args:
        image: Input image as numpy array (BGR format from OpenCV)
        roi: Region of interest as tuple (x, y, width, height)
        
    Returns:
        Dictionary containing success status and message
    """
    global tracker, image_roi
    
    # logging prefix for all messages from this function
    logging_prefix_str = "initialize_tracker:"

    # exit immediately if tracker is None
    if tracker is None:
        return {
            "success": False,
            "message": "Tracker not initialised"
        }

    try:
        # Initialize tracker with ROI
        success = tracker.init(image, roi)
        
        if success:
            image_roi = roi
            logger.info(f"{logging_prefix_str} Tracker initialized successfully with ROI: {roi}")
            return {
                "success": True,
                "message": f"Tracker initialized with ROI: {roi}"
            }
        else:
            tracker = None
            image_roi = None
            logger.error(f"{logging_prefix_str} Failed to initialize tracker with ROI: {roi}")
            return {
                "success": False,
                "message": f"Failed to initialize tracker with ROI: {roi}"
            }
            
    except Exception as e:
        tracker = None
        image_roi = None
        logger.exception(f"{logging_prefix_str} Error initializing tracker: {str(e)}")
        return {
            "success": False,
            "message": f"Error initializing tracker: {str(e)}"
        }


# Check and initialize tracking from normalized ROI
def check_and_initialize_from_normalized_roi(image: np.ndarray) -> Dict[str, Any]:
    """
    Check if there's a normalized ROI set and initialize tracking if needed
    
    Args:
        image: Current image frame
        
    Returns:
        Dictionary with initialization result
    """
    global image_roi_norm, image_roi, tracker
    
    # If there's no normalized ROI or tracker is already initialized, return early
    if not image_roi_norm or image_roi is not None:
        return {"success": False, "message": "No normalized ROI set or tracker already initialized"}
    
    # Convert normalized coordinates to pixel coordinates
    image_height, image_width = image.shape[:2]
    
    if len(image_roi_norm) == 4:
        # Rectangle format: (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
        top_left_x, top_left_y, bottom_right_x, bottom_right_y = image_roi_norm
        
        # Convert to pixel coordinates
        x = int(top_left_x * image_width)
        y = int(top_left_y * image_height)
        width = int((bottom_right_x - top_left_x) * image_width)
        height = int((bottom_right_y - top_left_y) * image_height)
        
        roi = (x, y, width, height)
        
        # Initialize tracker
        result = initialize_tracker(image, roi)
        
        if result["success"]:
            # Clear the normalized ROI since we've used it
            image_roi_norm = False
            logger.info(f"check_and_initialize_from_normalized_roi: Initialized tracker with normalized ROI: {roi}")
        
        return result
    else:
        logger.error(f"check_and_initialize_from_normalized_roi: Invalid normalized ROI format: {image_roi_norm}")
        return {"success": False, "message": "Invalid normalized ROI format"}
