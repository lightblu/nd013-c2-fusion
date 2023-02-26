
 

 Notes:

 - exec_data is gone and instructions should be updated: Due to changes in boilerplate code - helpers.py under misc, you don't need to pass this parameter any longer. make_exec_list function takes care of it.
 - maybe due to the same change, neither project or step setup mention you also need to adapt the sequence variable to 1, 2 or 3 according to selected sequence
 - min_iou missing in config, and the one func getting configs_det for nothing?
 
----
# Writeup: Visualize point cloud

## Find 10 examples of vehicles with varying degrees of visibility in the point-cloud

![./img/first.png](./img/first.png)
**One scene in the beginning: Contains already 6 different examples of cars (1-6) and also amazes how much is well recognizable in the scene in this pointcloud (A-E)**

| Id   |      Description      | 
|----------|:-------------:|
| A | Shadows large shadows caused by closest vehicles, center one by ego vehicle |   |
| B | Intersting shadow effect by the position of the camera and the rear shape of the ego vehicle |
| C | small warning construction sign thing  ![](./img/first-c-close.png) (interesting to sees the laser through the car windows on 2-3-4 )  |  
| D | high and far away features visiable by y value shading (red) |
| E | detailed garden structure recognizable |
| F | well visible curbs, left going around the curner |
| 1 | vehicle partly obstructed by ego vehicle shape "the front fell off" ![](./img/first-1-close.png) |
| 2 | very close and clearly visible front, top and right side ![](./img/first-234-mirrors.png) Very tiny and a bit hard to spot in still picture, but due to them stick off still good recognizable tiny point clouds for the mirrors.|
| 3 | Similar visible to 2 |
| 4 | Car with obstructed left front due to 3 ![](./img/first-234-f.png) also well visible cars at F |
| 5 | Jeep with trailers![](./img/first-5-jeep-trailer.png) |
| 6 | 3 cars barely visible (only 1/8th) and with much fewer points due to their distance ![](./img/first-6-far-away-cars.png) |

----

![](./img/second-4-cars-forward-far-away.png)
**7: Few far away cars with just their rear visible.**

----

![](./img/third-follower.png)
**8: Car following ego vehicle, close but only front and mirrors visible**

----

![](./img/fourth-close-car-right.png)
**9: Very close car to the right shadowed by ego vehicle, only top visible**

----

![](./img/fifth-truck-central-reserve.png)
**10: Big truck but quite obstructed and separated into multiple point clouds due to obstruction by various objects on the central reserve.**

----

![](./img/sixth-thress-cars-rear-obstructed.png)
**11: Three cars in the rear obstructed in different degrees due to ego vehicle.**

----

![](./img/seventh-obstructed-by-jeep-with-trailer.png)
**12: Very badly cars shadowed by the jeep and trailer before them, only recognizable by frames before.**

## Try to identify vehicle features that appear stable in most of the inspected examples and describe them
 
- **Mirrors**: Tiny, but due to their shape and sticking off well recognizable on many vehicles.
- **Windows**: Often well visible as "holes" through the vehicle.
- **Tires**: Good recognizable as round shapes if the bottom of the car is not obstructed.
- **General car shape**: Depending on obstruction, but in general also good recognizable is a lower larger box, and a smaller box on top as general shape


----

# Writeup: Track 3D-Objects Over Time


### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

