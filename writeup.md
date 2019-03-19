**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "project.ipynb" 

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 


<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/camera_cal/calibration3.jpg" width="450" title="Original Image">

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/undistorted_imgs/calibration3.jpg" width="450" title="Undistorted image">

### Pipeline (single images)
#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/test_images/straight_lines1.jpg" width="450" title="Original Image">

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/output_images/straight_lines1_undistorted.jpg" width="450" title="Original Image">

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

We will first use colour thresholding and we will achieve this in 3 step process since we not only need to worry about seperating line but also about varying light conditions which will make it harder. The 3 steps are-

Step1- Apply gradient threshold in X direction

Step2- Apply directional gradient since most lanelines are close to vertical (threshold are 30 and 120)

Step3- We apply a RGB threshold since yellow lines can be seperated

Step4- HLS Threshold to remove shadows which effects edge detection and helps seperate yellow and white lines easily

Step5- We combine all the above thresholds to get a clean binary image for the lane lines

Finally we apply the undistortion to the image and get the undistorted thresholded image we can start working on!! Look at the output below as an example of the process


<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/test_images/straight_lines1.jpg" width="450" title="Original Image">

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/output_images/thresholded_undistorted.jpg" width="450" title="Original Image">


#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is in the 7th code cell of the IPython notebook. Now we will start with the perspective transform which can now get us the lane markes to compute lane curvature. To start we define a region of interest which we will take the transformation on and since mostly the lanes appear in the specific region of image we can hardcode the ROI polygon.

We will compute the transformation and its inverse now. The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 565, 470      | 320, 1        | 
| 200, 720      | 320, 720      |
| 1140, 720     | 920, 720      |
| 735, 470      | 920, 1        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/output_images/perspective_transform_polygon.jpg" width="450" title="Original Image">


<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/output_images/perspective_transform.jpg" width="450" title="Original Image">

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?
Histogram evalution gives info about the position of lane markers in the warped image. I then used sliding window based search and then i used polyfit to fit a second orfer poynomial.This idea was from a previous graduate of the course to search aroung the lines for fixed pixels since consecutive frames are likely to have lanes in similar positions. Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:


<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/sliding_window_Search.JPG" width="450" title="Original Image">

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/search_around_prev_line.JPG" width="450" title="Original Image">

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.
 I calculate the left lane curvature and right lane curvature by generating polynomial using polyfit and applying the curvature formula. Then i average out the curvature to get the curvature. Below is the code.
```python
def measure_radius_of_curvature(x_values):
    ym_per_pix = 3/100 # meters per pixel in y dimension
    xm_per_pix = 3.7/660 # meters per pixel in x dimension
    # If no pixels were found return None
    y_points = np.linspace(0, num_rows-1, num_rows)
    y_eval = np.max(y_points)

    # Fit new polynomials to x,y in world space
    fit_cr = np.polyfit(y_points*ym_per_pix, x_values*xm_per_pix, 2)
    curverad = ((1 + (2*fit_cr[0]*y_eval*ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])
    return curverad
```

```python
left_curve_rad = measure_radius_of_curvature(left_x_predictions)
right_curve_rad = measure_radius_of_curvature(right_x_predictions)
average_curve_rad = (left_curve_rad + right_curve_rad)/2
```

```python
# compute the offset from the center
lane_center = (right_x_predictions[719] + left_x_predictions[719])/2
xm_per_pix = 3.7/700 # meters per pixel in x dimension
center_offset_pixels = abs(img_size[0]/2 - lane_center)
center_offset_mtrs = xm_per_pix*center_offset_pixels
offset_string = "Center offset: %.2f m" % center_offset_mtrs
print(offset_string)
```


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines # through # in my code in `yet_another_file.py` in the function `map_lane()`.  Here is an example of my result on a test image:

<img src="https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/output_images/straight_lines1.jpg" width="450" title="Original Image">


### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](https://github.com/dikshantpatel95/CarND-Advanced-Lane-Lines/blob/master/project_video_output.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?
The pipeline doesnt work well in varying lighting conditions and also since we are hardcoding the points manually, this causes an issue with the challenge and harder_challenge videos since due to sharp turns there is a lot of movement in the lanelines and the code is unable to act on it.

I've considered a few possible approaches for making my algorithm more robust. These include more dynamic thresholding (perhaps considering separate threshold parameters for different horizontal slices of the image, or dynamically selecting threshold parameters based on the resulting number of activated pixels), designating a confidence level for fits and rejecting new fits that deviate beyond a certain amount (this is already implemented in a relatively unsophisticated way) or rejecting the right fit (for example) if the confidence in the left fit is high and right fit deviates too much (enforcing roughly parallel fits). I hope to revisit some of these strategies in the future.
