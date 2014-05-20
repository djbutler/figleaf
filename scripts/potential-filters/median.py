import numpy as np
import cv2
import copy

# Defaults to webcam
cap = cv2.VideoCapture(0)

# Create Window
cv2.namedWindow('median')

# Callback function for user input
def nothing(x):
    pass

# User input
cv2.createTrackbar('amount', 'median', 0, 10, nothing)

# Video loop
while(True):

    # Capture image
    ret, frame = cap.read()

    # Resize image
    frame = cv2.resize(frame, None, fx=0.2, fy=0.2)

    # Query user input
    amount = cv2.getTrackbarPos('amount', 'median')*2+1 # odd

    # Process images
    color = cv2.medianBlur(frame, amount)
    gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)

    # Collect images
    r,c,d = frame.shape
    images = np.zeros((r,c*2,d), np.uint8)
    images[0:r,0:c,0:3] = color
    images[0:r,c:c*2,0:3] = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    # Display images
    cv2.imshow('median', images)
    
    # Exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
