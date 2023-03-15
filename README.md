# Camera_pose_from_horizon

Estimation of pose of imaging systems has important applications in different scientific and engineering fields. Here we present a program which is design to obtain certain information on pose of an imaging system form the horizon line in images obtained in marine environments.   


The main purpose of this program is to conduct the followings:

* Identify the horizon line from images taken in marine and oceanic environments
* Determine the pitch and roll of the camera using the identified horizon line 

<br />

## Horizon identification


Before starting the horizon identification process the user has to remove the distortion for the input image using intrinsic parameters obtained from the camera calibration process. Given that the camera distortion parameters are known, the undistort_img() function in img_utility module can be used to remove the image distortion.    

The horizon identification process consists of the following steps:

* Step 1: Blur the image. In the program blurring is conducted by the blur() function which by default uses a Gaussian filter and the user can specify the sigma value and blur matrix size. 

* Step 2: Part of the image has to be cropped, if the image contain systematic sharp edges, such as vehicle body, which may interfere with the horizon identification.

* Step 3: A Canny filter is applied on the image. The user can define the high and low thresholds for the Canny filter. 

* Step 4: A Hough transform is applied to the Canny-filtered images to find the prominent edges. Specifically we use the HoughLinesWithAccumulator() from OepnCV to obtain the edge line characteristics length ($r$) and angle ($\theta$) and vote values. Given that the horizon line is distinctively visible, the edge with highest vote should correspond to the horizon line.

All the methods required for the aforementioned steps are included in the OceanHorizonDetection module. 

__Note:__ the existence of other sharp-edge features in the images (that are not removed in the cropping step) can potentially interfere with the horizon detection. Therefore, the Canny filter limits has to be adjusted to assure proper identification of the horizon line.

<br />

## Pitch and Roll from horizon line

Once the horizon line is identified, the line information can be used to obtain the pitch and roll of the imaging system. Dusha et al. (2007) explained in detail the procedure and equations involved in calculation of pitch and roll from horizon line with the main intention for aerial vehicles. In summary, assuming that the ground (ocean surface) is a flat surface, the projection equation, relating the world coordinate to pixel Once the horizon line is identified, the line information can be used to obtain the pitch and roll of the imaging system. Dusha et al. ([2007](https://eprints.qut.edu.au/6852/)) explained in detail the procedure and equations involved in calculation of pitch and roll from horizon line with the main intention for aerial vehicles. In summary, assuming that the ground (ocean surface) is a flat surface and the horizon line is infinitly far, the projection equation, relating the world coordinate to pixel coordinate, is solved over the horizon line, and elements of the matrix relating to pitch and roll obtained form the solution. Schwendeman and Thomson (2015), uses a similar approach to obtain attitude of shipborne cameras. My current program is based on the equation and geometry explained by Schwendeman and Thomson ([2015](https://doi.org/10.1175/JTECH-D-14-00047.1)). 


Assuming the the horizon line is characterized by $r$ and $\theta$ we use the following formulae to calculate the pitch ($\phi$) and roll ($\tau$):

$$\ \tau = tan^{-1}(\frac{-f_{u}}{f_{v}tan(\theta)}) \$$
$$\  \$$

$$\ \phi = \frac{\pi}{2}-tan^{-1}(\frac{f_{u}sin(\tau)cos(\theta)-f_{v}cos(\tau)sin(\theta)}{r-c_{u}cos(\theta)-c_{v}sin(\theta)}) \$$


where $f_{u}$, $f_{v}$, $c_{v}$, $c_{u}$ are intrinsics camera calibration factors ($c$: focal center point, $f$: focal length in pixel, subscripts $v$, and $u$ correspond to vertical and horizontal screen directions, respectively). More detail on the equation parameters could be found in Schwendeman and Thomson ([2015](https://doi.org/10.1175/JTECH-D-14-00047.1)). 