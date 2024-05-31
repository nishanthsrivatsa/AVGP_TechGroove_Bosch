import cv2
import numpy as np

# Define the lower and upper boundaries of the "green" color in the HSV color space
lower_green = np.array([40, 40, 40])  
upper_green = np.array([80, 255, 255])

def green_light_detection(lower_green, upper_green):
    # Capture video from the default camera (usually the built-in webcam)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open video stream")
        return False
    
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Could not read frame")
            break
        
        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the green color
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Bitwise-AND mask and original image
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # If there are any green pixels in the mask, return True
        if cv2.countNonZero(mask) > 0:
            cap.release()
            return True
        
        # Display the result frame for debugging (optional)
        cv2.imshow('Frame', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the video capture object and close display windows
    cap.release()
    cv2.destroyAllWindows()
    return False

# Example usage
if green_light_detection(lower_green, upper_green):
    print("Green light detected")
else:
    print("No green light detected")
