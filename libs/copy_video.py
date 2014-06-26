import numpy as np
import cv2
import sys

cap = cv2.VideoCapture(sys.argv[1])
size = (int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))
fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)
print('size: %fx%f' % size)
print('fps: %f' % fps)
# Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC(*'DIV4')
out = cv2.VideoWriter(sys.argv[2],fourcc, fps, size)

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        #frame = cv2.flip(frame,0)
        # write the flipped frame
        #assert(out.isOpened())
        out.write(frame)
#        import pdb; pdb.set_trace()
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()

