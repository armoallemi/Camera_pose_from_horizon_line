import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys

import img_utility as imutil



def rho_theta_to_xy(rho, theta, x):

    y = (rho-x*np.cos(theta))/np.sin(theta)
    return y


class ImageHorizon:

    def __init__(self, img_load_status = False):

        #self.test_img = test_img
        self.img_load_status = img_load_status 
        self.__test_img = None
        self.preprocessed_img = None
        self.processed_img = None
        self.hough_lines = None
        self.horizon_x = None
        self.horizon_y = None
        self.vote_ratio = None
        self.rho = None
        self.theta = None
        self.roll = None
        self.pitch = None
    
    def load_data(self, img: np.array):
        """
        test image has to be grey scale
        """
        # check grey scale
        if not imutil.grey_scale_check(img):
            img_test = imutil.grey_scale(img)

        if img.shape[0]>20 and img.shape[1]>20: 

            self.__test_img = img
            self.img_load_status = True

        else:
            print('Please use a valid image data')

        return self
    
    def blur(self, **kwargs):
        """
        blur image with specified filter
        currently supported filters ['Gaussian']

        :param img (np.array): input image 
        :param **kwargs "filter" (str): name of blur filter to use 
        :param **kwargs "matrix_size" (2D set): size of the blur matrix 
        :param **kwargs "sigma" (float): blur sigma value 
        
        :return (np.array) of blurred image 
        """

        matrix_size = (7,7) # default blur matrix 
        sigma = 2 # default blur sigma 

        if 'matrix_size' in kwargs.keys():
            matrix_size = kwargs['matrix_size']
            print(matrix_size)

        if 'sigma' in kwargs.keys():
            sigma = kwargs['sigma']

        try:

            self.preprocessed_img = cv2.GaussianBlur(self.__test_img, matrix_size, sigma)
            
        except cv2.error:
            print("Faulty value(s) for one or more blur filter input(s) (input image, blur matrix size, or sigma) are faulty")
            raise cv2.error
            
        return self
        

    
    def crop(self, x_pixel_range: list, y_pixel_range: list):
        """
        crop the image: this function can be used to exclude certain features in the image it 
        (e.g., part of the vehicle).

        :param img (np.array): input image
        :param x_pixel_range (list of size 2): list of x pixel to be included x[0] lower limit index, x[1] upper limit index
        :param x_pixel_range (list of size 2): list of y pixel to be included y[0] lower limit index, y[1] upper limit index
        :return: (np.array): cropped image
        """
        
        self.preprocessed_img = self.preprocessed_img[y_pixel_range[0]:y_pixel_range[1], x_pixel_range[0]:x_pixel_range[1]]

        return self
    
    ### Here I included the Sobel filter however my preliminary tests 
    ### showed that using it deteriorated the horizon identification

    def sobel(self):
        """
        applies Sobel filter to input images 
        Applies Sobel filter to the x and y directions and 
        takes the sqrt root of the sum of squares of x and y Sobel filters
        :param img (np.array): the input image 
        :return np.array: Sobel filter result over image pixels    
        """
        # Sobel x and y direction 
        sobelx = cv2.Sobel(self.preprocessed_img,cv2.CV_64F,1,0,ksize=3)
        sobely = cv2.Sobel(self.preprocessed_img,cv2.CV_64F,0,1,ksize=3)
        
        # Square of x and y Sobel filters  
        sobelx2 = cv2.multiply(sobelx,sobelx)
        sobely2 = cv2.multiply(sobely,sobely)
        
        # add together and take square root
        sobel_magnitude = cv2.sqrt(sobelx2 + sobely2)
        #exposure.rescale_intensity(sobel_magnitude, in_range='image', out_range=(0,255)).clip(0,255).astype(np.uint8)

        # normalize to range 0 to 255 and clip negatives
        return sobel_magnitude



    def canny(self, threshold1: int, threshold2: int):
        """
        Applies Canny filter on the input image

        :param img (np.array): image
        :param threshold1 (int): lower canny threshold limit
        :param threshold2 (int): upper canny threshold limit
        :return np.array: canny filter result
        """

        try:
            self.processed_img = cv2.Canny(self.preprocessed_img, threshold1, threshold2)

        except cv2.error:
            print("The Canny filter inputs are faulty")
            raise cv2.error

        return self
    
    def hough_accumulator(self):
        try:
            lines = cv2.HoughLinesWithAccumulator(
                    self.processed_img, # Input edge image
                    1, # Distance resolution in pixels
                    0.5*np.pi/180, # Angle resolution in radians
                    threshold=40, # Min number of votes for valid line
                    srn = 0, # Min allowed length of line
                    stn = 0 # Max allowed gap between line for joining them
                    )
        
        except cv2.error:
            print("The HoughLinesWithAccumulator inputs are faulty")
            raise cv2.error

        self.hough_lines = lines
        # polar properties of identified lines in the Hough operator  
        rho_, theta_, vote_ = self.hough_lines[0][0]
        self.rho = rho_
        self.theta = theta_

        self.horizon_x = [0, np.shape(self.preprocessed_img)[1]]
        self.horizon_y = [rho_theta_to_xy(rho_, theta_, self.horizon_x[0]),
                          rho_theta_to_xy(rho_, theta_, self.horizon_x[1])]

        self.vote_ratio = round(self.hough_lines[1][0][-1]/self.hough_lines[0][0][-1],2)

        
        return self
    
    
    def get_attitude_polar(self, K_int):

        """
        This function uses the rho and theta values of the line with highest Hough vote 
        and used the formulae from Schwendeman and Thomson (2015) to obtain the pitch and roll 
        from horizon line data 

        :param K_int: intrinsic camera calibration matrix
        """

        theta = self.theta 
        r_ = self.rho

        # relevant calibration factors for attitude calculations    
        f_u = K_int[0,0]
        f_v = K_int[1,1]
        c_u = K_int[0,2]
        c_v = K_int[1,2]
        
        # following formulae are from the paper of Schwendeman and Thomson 2015 
        tau = np.arctan(-f_u/f_v/np.tan(theta))
        sig = np.arctan((f_u*np.sin(tau)*np.cos(theta) - f_v*np.cos(tau)*np.sin(theta))/(r_ - c_u*np.cos(theta) - c_v*np.sin(theta)))
        
        roll = tau*180/np.pi #deg
        pitch = 90-sig*180/np.pi #deg

        self.pitch = pitch
        self.roll = roll
        
        return self



if __name__ == '__main__':

    #load image
    img_test = cv2.imread('camera0-2021-08-24---10-52-42.514609.tiff')

    # make sure to un-distort the image
    # load camera calibration factors

    cal_mtx_ = imutil.read_cv_xml('intrinsics_00.xml', 'intrinsics_penne')
    cal_dist_ = imutil.read_cv_xml('distortion_00.xml', 'intrinsics_penne')
    test_img_und = imutil.undistort_img(img_test, cal_mtx_, cal_dist_) # undistort


    ###Horizon identification process
    ###

    horizon = ImageHorizon()

    horizon.load_data(test_img_und)
    horizon.blur()

    x_lim = [0, test_img_und.shape[1]]
    y_lim = [0, int(test_img_und.shape[0]*2/5)]
    horizon.crop(x_lim, y_lim)

    canny_min_lim = 35 
    canny_man_lim = 80
    
    horizon.canny(canny_min_lim, canny_man_lim)
    horizon.hough_accumulator()
    horizon.get_attitude_polar(cal_mtx_)

    ###plot results
    ###
    fig, ax = plt.subplots(1,2)
    deg_sign =  u'\N{DEGREE SIGN}'
    ax[0].set_title(f'Pitch: {np.round(horizon.pitch,3)}{deg_sign}, Roll: {np.round(horizon.roll,3)}{deg_sign}')
    ax[0].imshow(test_img_und)
    ax[0].plot(horizon.horizon_x, horizon.horizon_y, color = 'magenta', label = 'identified horizon', linewidth = 0.5)

    ax[1].imshow(horizon.preprocessed_img)
    ax[1].plot(horizon.horizon_x, horizon.horizon_y, color = 'magenta', linewidth = 0.5, label = 'identified horizon')

    plt.legend()





    
    
    













# def img_blur_n_crop(img: np.array) -> np.array:

#     # preprocess image:

#     # check grey scale
#     if not prep.grey_scale_check(img):
#         img_test = prep.grey_scale(img)

#     #apply blur
#     img_blur = prep.blur(img)

#     #crop image
#     crop_range_x = [5,int(np.shape(img_blur)[0]/3)]
#     crop_range_y = [5,np.shape(img_blur)[1]]
#     img_cropped = prep.crop(img_blur, crop_range_x, crop_range_y)
    
#     return img_cropped

# def hough_accumulator(img_cropped, canny_thresh_list):

#     for threshold1, threshold2 in canny_thresh_list:

#         canny_magnitude = proc.canny(img_cropped, threshold1=threshold1, threshold2=threshold2)

        
#         lines = cv2.HoughLinesWithAccumulator(
#                 canny_magnitude, # Input edge image
#                 1, # Distance resolution in pixels
#                 0.5*np.pi/180, # Angle resolution in radians
#                 threshold=40, # Min number of votes for valid line
#                 srn = 0, # Min allowed length of line
#                 stn = 0 # Max allowed gap between line for joining them
#                 )

#     return lines