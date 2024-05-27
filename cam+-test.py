#!/usr/bin/env python3
import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines):
    if lines is None:
        return img
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img

def process(image):
    height, width = image.shape[:2]
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    # Apply Gaussian Blur
    blur_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
    
    # Edge detection using Canny
    canny_image = cv2.Canny(blur_image, 50, 150)
    
    # Define region of interest
    vertices = np.array([[(0, height), (width // 2, height // 2), (width, height)]], dtype=np.int32)
    cropped_image = region_of_interest(canny_image, vertices)
    
    # Hough Transform to detect lines
    lines = cv2.HoughLinesP(cropped_image, rho=1, theta=np.pi / 180, threshold=50, lines=np.array([]), minLineLength=40, maxLineGap=150)
    
    # Draw lines on the image
    image_with_lines = draw_lines(image, lines)
    
    return image_with_lines

# Capture video from the webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Process the frame for lane detection
    frame_with_lanes = process(frame)
    
    # Display the frame with lanes
    cv2.imshow('Lane Detection', frame_with_lanes)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()

