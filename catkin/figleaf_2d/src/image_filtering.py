import numpy as np
import cv2
import sys
from skimage.io import imsave, imread
import os

from skimage import data, img_as_float, img_as_ubyte
from skimage.segmentation import slic as slic_seg
from skimage.filter import denoise_bilateral, gaussian_filter
import skimage.filter
import skimage.color
from skimage.color import rgb2hsv, hsv2rgb

def canny(img):
    return 255*skimage.filter.canny(skimage.color.rgb2gray(img))

# def process_video(video_file_in, video_file_out, func):
#    cap = cv2.VideoCapture(video_file_in)
#    size = (int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
#            int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))
#    fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)
#    print('size: %fx%f' % size)
#    print('fps: %f' % fps)
#    # Define the codec and create VideoWriter object
#    fourcc = cv2.cv.CV_FOURCC(*'DIV4')
#    out = cv2.VideoWriter(video_file_out,fourcc, fps, size)
#
#    while(cap.isOpened()):
#        ret, frame = cap.read()
#        if ret==True:
#            frame = func(frame)
#            out.write(frame)
#        else:
#            break
#    # Release everything if job is finished
#    cap.release()
#    out.release()

def bilateral(ubyte_frame):
    return img_as_ubyte(denoise_bilateral(img_as_float(ubyte_frame), sigma_range=0.3, sigma_spatial=15, win_size=10))
    
def gaussian(ubyte_frame, sigma=5):
    return img_as_ubyte(gaussian_filter(img_as_float(ubyte_frame), sigma=sigma, multichannel=True))

def slic(ubyte_frame, **kwargs):
    seg = slic_seg(ubyte_frame, **kwargs)
    f = np.zeros_like(ubyte_frame)
    for val in np.unique(seg):
        f[seg == val] = np.mean(ubyte_frame[seg == val], axis=0)
    return f

def slic_of_gaussian(ubyte_frame, sigma=5, n_segments=400):
    return slic(gaussian(ubyte_frame, sigma=sigma), n_segments=n_segments)

def inverted_color(ubyte_frame):
    return 255 - ubyte_frame;

def inverted_hue(ubyte_frame):
    img = cv2.cvtColor(ubyte_frame, cv2.cv.CV_BGR2HSV)
    img[:,:,0] = (img[:,:,0] + 90) % 180
    return cv2.cvtColor(img, cv2.cv.CV_HSV2BGR)

def inverted_hue_of_slic_of_gaussian(ubyte_frame, sigma=5):
    return inverted_hue(slic_of_gaussian(ubyte_frame, n_segments=400, sigma=sigma))

def inverted_hue_of_slic_of_gaussian(ubyte_frame, n_segments=400, sigma=5, compactness=40):
    return inverted_hue(slic_of_gaussian(ubyte_frame, sigma=sigma, n_segments=n_segments))

def inverted_color_of_slicsmall_of_gaussian(ubyte_frame):
    return inverted_color(slic(gaussian(ubyte_frame, sigma=5), compactness=30))


# IN_VIDEO = '/home/dan/Dropbox/projects/roboprivacy/data/videos/dans_bedroom_slr.m4v'
# OUT_DIR = '/home/dan/Dropbox/projects/roboprivacy/data'

#for func in [inverted_hue, inverted_color, slic, gaussian, bilateral, slic_of_gaussian, inverted_hue_of_slic_of_gaussian, inverted_color_of_slic_of_gaussian]:
#    filter_dir = OUT_DIR + '/' + func.__name__
#    out_fname = filter_dir + '/' + IN_VIDEO.split('/')[-1].split('.')[0] + '.avi'
#    try:
#        os.mkdir(filter_dir)
#        process_video(IN_VIDEO, out_fname, func)
#    except OSError:
#        print('Writing %s failed' % out_fname)

