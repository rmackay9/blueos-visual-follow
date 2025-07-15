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
image_roi = None  # region of interest for tracking


# Run tracking algorithm on the provided image using OpenCV's TrackerCSRT
def get_tracking(curr_image: np.ndarray, roi, include_augmented_image: bool) -> Dict[str, Any]:
    """
    Run tracking algorithm on the provided image using OpenCV's TrackerCSRT

    Args:
        image: Input image as numpy array (BGR format from OpenCV)
        roi: region of interest to track, if none then any pre-existing region will be used
        include_augmented_image: if true an augmented image with a rectangle drawn around the target should be returned to the caller

    Returns:
        Dictionary containing:
        - success: bool indicating if detection was successful
        - center_x: normalised horizontal value of the tracked object (-1 is left, +1 is right)
        - center_y: normalised vertical value of the tracked object (-1 is top, +1 is bottom)
        - box_x: normalised bounding box top-left corner x coordinate (-1 to +1)
        - box_y: normalised bounding box top-left corner y coordinate (-1 to +1)
        - box_width: normalised bounding box width (-1 to +1)
        - box_height: normalised bounding box height (-1 to +1)
        - message: Status message
        - image_base64: Base64 encoded image, None if not requested or could not be generated
    """

    # logging prefix for all messages from this function
    logging_prefix_str = "get_tracking:"

    global tracker, image_roi

    try:
        # create tracker if not already created
        if tracker is None:
            tracker = cv2.TrackerCSRT_create()
            logger.info(f"{logging_prefix_str} Created new TrackerCSRT instance")

        # update ROI if provided
        if roi is not None:
            image_roi = roi
            # Initialize tracker with new ROI
            tracker = cv2.TrackerCSRT_create()
            success = tracker.init(curr_image, tuple(roi))
            if not success:
                logger.warning(f"{logging_prefix_str} Failed to initialize tracker with ROI")
                return {
                    "success": False,
                    "center_x": None,
                    "center_y": None,
                    "box_x": None,
                    "box_y": None,
                    "box_width": None,
                    "box_height": None,
                    "message": "Failed to initialize tracker with ROI",
                    "image_base64": None
                }

        # fail immediately if no tracker or ROI
        if tracker is None or image_roi is None:
            logger.warning(f"{logging_prefix_str} Tracking failed - no tracker or ROI")
            return {
                "success": False,
                "center_x": None,
                "center_y": None,
                "box_x": None,
                "box_y": None,
                "box_width": None,
                "box_height": None,
                "message": "Tracking failed, no tracker or ROI",
                "image_base64": None
            }

        # run tracking
        success, box = tracker.update(curr_image)
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

        # calculate center of the bounding box
        (box_x, box_y, box_width, box_height) = [int(v) for v in box]
        center_x = box_x + (box_width // 2)
        center_y = box_y + (box_height // 2)

        # get image dimensions for normalization
        h_orig, w_orig = curr_image.shape[:2]

        # normalize coordinates to -1 to +1 range
        norm_center_x = (center_x / w_orig) * 2 - 1  # -1 (left) to +1 (right)
        norm_center_y = (center_y / h_orig) * 2 - 1  # -1 (top) to +1 (bottom)
        norm_box_x = (box_x / w_orig) * 2 - 1
        norm_box_y = (box_y / h_orig) * 2 - 1
        norm_box_width = (box_width / w_orig) * 2
        norm_box_height = (box_height / h_orig) * 2

        # Create augmented image
        image_base64 = None
        if include_augmented_image:
            try:
                # Create a copy of the original image for visualization
                augmented_image = curr_image.copy()

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


# Reset the tracker instance and clear the ROI
def reset_tracker():
    """
    Reset the tracker instance and clear the ROI
    
    This function should be called when tracking needs to be restarted
    or when switching between different targets
    """
    global tracker, image_roi
    
    # logging prefix for all messages from this function
    logging_prefix_str = "reset_tracker:"
    
    tracker = None
    image_roi = None
    logger.info(f"{logging_prefix_str} Tracker reset successfully")


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
    
    try:
        # Create new tracker instance
        tracker = cv2.TrackerCSRT_create()
        
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
