import os
from pathlib import Path
import numpy as np
import cv2

def read_cv_xml(file_name, data_name):
    
    """
    this function read the data in an open cv xml file
    """
    
    if os.path.isfile(file_name) :
        
        cv_file = cv2.FileStorage(file_name, cv2.FILE_STORAGE_READ)
        # for some reason __getattr__ doesn't work for FileStorage object in python
        # however in the C++ documentation, getNode, which is also available, 
        # does the same thing
        # note we also have to specify the type to retrieve other wise we only get a 
        # FileNode object back instead of a matrix
        matrix = cv_file.getNode(data_name).mat()
        cv_file.release()
    
    else:
        raise FileNotFoundError

    return matrix

def grey_scale_check(img: np) -> bool:
    """
    This function check if the image is grey scale of not
    :param img (np.array): input image
    :return (bool): True means it is grey and False means it is not

    Note: The function is very simple and consider 3D gray scale images as non gray scale
    This can be further modified 
    """
    if len(np.shape(img)) == 2:

        return True

    else:
        
        return False
    
def grey_scale(img: np.array) -> np.array:
    """
    convert image to grey scale
    param: img (np.array): input image 
    return: img (np.array): input image 
    """
    return  cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) 


def undistort_img(img, cal_mtx, cal_dist):
    """
    This function takes the original raw image, intrinsic 
    calibration factors, and distortion factors and return the 
    undistorted image with blanked boundaries cropped  
    :param img (np.array): input image
    :param cal_mtx (np.array): intrinsic calibration matrix
    :param cal_dist (np.array): intrinsic distortion matrix
    :return undst_img (bool): cropped undistorted image 
    """
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cal_mtx, cal_dist, (w,h), 1, (w,h))

    # undistort
    undst_img = cv2.undistort(img, cal_mtx, cal_dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    undst_img = undst_img[y:y+h, x:x+w]

    return undst_img