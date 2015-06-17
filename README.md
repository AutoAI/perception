![DriveAILogo](readme-assets/DriveAILogo.png)
# Perception

You are currently viewing a subsection of our project. To view the project in it's entierety, see our (platform)[https://github.com/DriveAI/DriveAI-Platform].

## About
Perception is a ROS Package on a self driving car that will take camera and lidar data, and tell other packages what the world around the vehicle looks like.

We're attempting a new approach that would dramatically reduce the computational and financial cost of self driving cars, below is that approach.

The stereo correspondance procedure is an algorithm that takes images coming from stereo cameras and creates depth-maps from these images. Making this procedure reliable is very computationally expensive.

![StereoCorrespondance](readme-assets/stereo-correspondance)

With the stereo corresponace procedure, a computer has to go through each row of pixels. It finds pairs of corresponding pixels, where a pixel in one image corresponds to the other if the pixels seem like they come from the same point in the real world. For each pair it sees what the `x` value of each pixel is and, based on the difference in `x` value, it can infer how far away the object is.

This procedure is normally done in a brute force manner, and we think that we can dramatically improve upon this procedure.

Our procedure, first takes lidar points, and determines what is a realistic range of depths for every point in our scene. We base this information off the fact that we can predict what the depth's of the points near, and between lidar points will be.

Once we have this information, going through the stereo correspondance becomes a whole lot easier. We come down from a O(n^2) to a O(n√n) or better procedure.

## Installation
You can integrate this into your ROS Workspace, as any other package

## Tests

### To compile our tests:
(In your workspace)
```
catkin_make Perception_utest
```

### To run our tests:
```
rosrun Perception Perception_utest
```
