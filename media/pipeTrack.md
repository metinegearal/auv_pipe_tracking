
![alt text](pipeTrack.gif)

# 2D Scan Algorithm

This is a scan algorithm that utilizes masks coming from segmentation algorithms to explore all area of the POI. 

## Main Algorithm

In current algorithm we are using our depth to seabed, compass's magnetic angle and AUV's coordinat point to find POI's pixels world positions. 

Depend on the distance to AUV pixels classified 3 different way: object, not-an-object, interested-point. If pixels are close *for some %*, they either classified as object or not-an-object based on mask output. if they are farther, tagged points in the mask are classified as interested-point.
To prevent flunctions, exponentional smoothing with *some parameter* is applied to the pixels that already classified before. Where  x<0.5 is not-an-object and x>0.5 is an object.
After that *closest* interested-point is selected to send movement algorithm this process continues until there is no interested-point left.

Movement algorithm works by taking the point and creating motor commands based on distance and angle to center point. Please read the movement section to get more information about how it works.

This points will also saved to a map. Map will resize and change resolution automatically based on the farthest two point.

![alt text](final_map.png)

![alt text](pipeTrack_no_DVL.gif)

## Scan Without Coordinates

Incase of a lack of coordinate system like a lack of precise DVL, IMU and/or environment based limitations. This second model just uses a camera that focuses to seabed. 

When we cant use coordinates and map based system will fail eventually so we should gather as much parameter as possible from one single frame or a few more frame to data processing like smoothing, gate etc. 

Algorithm will select the best suitable point to go by giving weights to every tagged pixel of the mask. This weights are given based on the distance to the center and angle of the pixel(0=Front 3.14=back)
-W = -W1*(d-dc)-W2*a where a is angle, d is distance, dc is ideal distance and W's positive floats.
Ideal distance is distance where system focuses. Depends of this parameter and weights we can set when the turn is executed. Looking father or reducing the negative effect of angle will cause early turns.

The pixel with less W will be selected and sent to movement algorithm.

![alt text](overlay.png)

One can wonder why not use bbox based systems unfortunately it can face some problems in case of seeing big part of the pipeline or some problematic detections.

## Future improvements

Sonar based detection systems will be added later.
Some precision metrics to ensure minimum time spent on scanning.
RL based method and its tests in real world to see simulation codes reality.
one pixel -> look more pixel -> less sensitivity to mistakes