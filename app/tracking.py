#!/usr/bin/env python3

"""
Optical Flow based motion estimation module

This module handles Optical Flow estimation using a downward facing camera.
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


def get_tracking(curr_image: np.ndarray, capture_time, roi, include_augmented_image: bool) -> Dict[str, Any]:
    """
    Run tracking algorithm on the provided image using OpenCV's TrackerCSRT

    Args:
        image: Input image as numpy array (BGR format from OpenCV)
        capture_time: system time that the image was captured
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

    try:
        # create tracker if not already created
        if tracker is None:
            tracker = cv2.TrackerCSRT_create()
            logger.info(f"{logging_prefix_str} Created new TrackerCSRT instance")

        # record ROI is provided
        if roi is not None:
            image_roi = roi

        # fail immediately if no tracker or ROI
        if tracker is None or image_roi is None:
            logger.warning(f"{logging_prefix_str} Tracking failed")
            return {
                "success": False,
                "center_x": None,
                "center_y": None,
                "box_x":None,
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

        # Create augmented image
        image_base64 = None
        if include_augmented_image:
            # Use the original curr_image for visualization, but scale flow vectors appropriately
            augmented_image = curr_image.copy()

            # Scale coordinates back to original image size
            h_orig, w_orig = curr_image.shape[:2]
            h_crop, w_crop = int(h_orig * 0.6), int(w_orig * 0.6)
            offset_y, offset_x = (h_orig - h_crop) // 2, (w_orig - w_crop) // 2

            for i, (new, old) in enumerate(zip(good_new, good_old)):
                # Scale coordinates back to original image coordinates
                a, b = new.ravel() * scale_factor
                c, d = old.ravel() * scale_factor

                # Add crop offset
                a, b = a + offset_x, b + offset_y
                c, d = c + offset_x, d + offset_y

                # Convert coordinates to integers for OpenCV drawing functions
                a, b, c, d = int(a), int(b), int(c), int(d)

                # Only draw if coordinates are within image bounds
                if 0 <= a < w_orig and 0 <= b < h_orig and 0 <= c < w_orig and 0 <= d < h_orig:
                    cv2.line(augmented_image, (a, b), (c, d), (0, 255, 0), 2)
                    cv2.circle(augmented_image, (a, b), 5, (0, 0, 255), -1)

            _, buffer = cv2.imencode('.jpg', augmented_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')

        # return success
        return {
            "success": True,
            "message": "Success",
            "center_x": center_x,
            "center_y": center_y,
            "image_base64": image_base64
        }

    except Exception as e:
        logger.exception(f"Error during Optical Flow calculation: {str(e)}")
        return {
            "success": False,
            "message": f"Optical Flow calculation failed: {str(e)}",
            "flow_x": None,
            "flow_y": None,
            "dt": None,
            "image_base64": None
        }
