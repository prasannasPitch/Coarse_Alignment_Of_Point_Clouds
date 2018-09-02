# 3D Coarse Alignment of Point Clouds For Pose Estimation & Workpiece Localization

## Motivation

3D Vision Sensors can generate depth map, using the difference in location of an observed object (disparity) between the left and right camera views to measure the depth of the object from the viewer. This adds an additional dimensionality to a 2d image making it as a 3d image. This feature could be used to find the distance between objects, separating objects from the background layers behind them and much better object recognition than a traditional camera – well suits for computer vision systems in robots. 

Welding is an essential process in building machines and industrial products. Typically done by small and medium sized companies(SME), it requires skilled labor and experience. Aim of welding robots is to make this process cost effective and less strenuous. As programming of welding robots in online mode is a time consuming process, necessity of offline programming arises. This topic is an approach to localize the workpiece thereby achieving fully offline programming of welding tasks. Though this process is not specific to any application, the motivation behind is to localize objects by a 3D vision system.

## Goal of the Topic

- Coarse alignment of CAD point cloud and Camera Measurement (Registration of Point Cloud).
- Estimate Rigid transformation between Camera Coordinates and Workpiece coordinates.
- With initial pose estimation , achieve accurate workpiece localization.

## PCL Library

The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing. PCL has various methods processing point clouds and has the support of visualization in VTK. This work includes PCL (C++) version 1.8 integrated with Visual Studio 2017. More information is available on the site http://www.pointclouds.org/documentation/tutorials/

## Steps Involved

- Dataset Reduction
- Feature Generation
- Model Training & Evaluation
- Resolving Problems
- Observations & Results

#### Dataset Reduction - Keypoint Generation

A point cloud is a collection of data points defined by a given coordinates system. In a 3D coordinates system, for example, a point cloud may define the shape of some real or created physical system. Usually point clouds are massive in count i.e based on the camera parameters and density of measurement setting, point clouds would be huge in numbers. Basic approach used in this topic is to find corresponding points in both the point clouds and thereby estimating rigid transformation between corresponding points. But having a huge dataset makes computation much difficult. So PCL Keypoint generation is applied to both the point clouds to minimize the region of interest. There are various keypoint methods discussed in PCL library such as SIFT, NARF, ISS. For this problem statement, I have found that Harris 3d keypoint generation gave better results.

The Harris method (Harris and Stephens, 1988) is a corner and edge based method and these types of methods are characterized by their high-intensity changes in the horizontal and vertical directions. For the 3D case, the adjustment made in PCL for the Harris3D detector replaces the image gradients by surface normals. With that, they calculate the covariance matrix around each point.

    //Code Snippet to Detect Harris 3D Keypoints
    
    
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>* harris3D = new
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS);
			
    harris3D->setNonMaxSupression(false);
		harris3D->setRadius(12);
		harris3D->setInputCloud(orig_pcd);
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr key_regions(new pcl::PointCloud<pcl::PointXYZ>);
		harris3D->compute(*keypoints);
		pcl::StopWatch watch;
		pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size(), watch.getTimeSeconds());

		/*if (!keypoints->size())
		{*/
		pcl::io::savePCDFile("keypoints_10r_cad_pose0.pcd", *keypoints);
		pcl::console::print_info("Saved keypoints to keypoints.pcd\n");
		int size_pt = 0;
		float intensity_thresh = .0116f;
		for (size_t i = 0; i < keypoints->size(); i++)
		{
			if (keypoints->points[i].intensity >= intensity_thresh)
			{
				++size_pt;
			}
		}
		key_regions->width = size_pt;
		key_regions->height = 1;
		key_regions->is_dense = false;
		key_regions->points.resize(key_regions->width * key_regions->height);
		int kex_index = 0;
		for (size_t i = 0; i < keypoints->size(); i++)
		{
			
			if (keypoints->points[i].intensity >= intensity_thresh)
			{
				key_regions->points[kex_index].x = keypoints->points[i].x;
				key_regions->points[kex_index].y = keypoints->points[i].y;
				key_regions->points[kex_index].z = keypoints->points[i].z;
				++kex_index;
			}
		}
		pcl::io::savePCDFile("key_regions.pcd", *key_regions);
    
![cad-ptcld](https://user-images.githubusercontent.com/37708330/44541347-f31b5880-a709-11e8-9a9e-741a2173a950.png)
Fig. 1 - Step File of the Model and its corresponding point cloud with keypoints highlighted.






![side by side cad-cam-n](https://user-images.githubusercontent.com/37708330/44542327-c583de80-a70c-11e8-9f54-f02a6400a2e0.png)
Fig. 2 - 2d Image of work piece and 3d point cloud data after keypoint generation.

Major problem with Harris 3D keypoint generation is that it really gives the key regions of the point cloud but not the exact keypoint. So clustering algorithm is applied to find the most important point from the regions (cluster). Results showed that K Means clustering algorithm performed better compared with all other clustering methods.


#### Feature Generation

As you could see in fig.2 , the 3d sensor measurement is partial and might be skewed with respect to the 3d model. Our algorithm should obtain features in such a way that, irrespective of the pose of the workpiece, features should remain constant. From the 3D vision sensor we get only the location of each point cloud in space. PCL supports feautre descriptors such as FPFH, PFH which could be used but that didnt work well in my case. There are two major reasons for this:

- It is too computationally expensive to perform at real time.
- Similar features leaded to irrelevant correspondences.

Major challenge was to develop features which are insensitive to local features and without using heavy feature descriptors. Euclidean distance will be the best possible feature that could be exploited from a data which only has point location. 

Consider we found 14 Key points in source point cloud (CAD point cloud) and 5 points in target point cloud (Sensor point cloud).
Source points are represented in Workpiece Coardinates and target points are represented in Sensor/TCP/World Coardinates. Now the source is to be transformed to the target by which transformation between workpiece coardinate and world coardinate could be obtained. 



![image](https://user-images.githubusercontent.com/37708330/44544763-683f5b80-a713-11e8-8f67-6138bcc3963e.png)


Combinations of points in source point cloud i.e. by the number of key points in the target, combinations of points are selected from the source point cloud. Here 5 points are selected from 14 points. Each combination of points corresponds to one dataset of features – Euclidean distance between each points.



![image](https://user-images.githubusercontent.com/37708330/44545342-14ce0d00-a715-11e8-961a-594399fa9531.png)




![image](https://user-images.githubusercontent.com/37708330/44545516-9887f980-a715-11e8-8ed7-82bf579dbfc2.png)






![features](https://user-images.githubusercontent.com/37708330/44545669-fd435400-a715-11e8-94a6-feb0c6470879.png)

Now for each possible combination of points from source point cloud, there exist one corresponding feature matrix consisting of all possible distances. Considering the above points, there are 10 features (x0,x2,x3...x9) for a set of combination of points. These 10 features covers all the Eucledian distances between each other. This is considered as the features in my algorithm. The size of our training data will be [no. of combinations, size of features].

### Model Training & Evaluation

Now we have prepared our training data for our model. This model which we are about to develop would require a evaluation data to find the result which we are looking for. Just to be reminded, primary goal is to estimate the transformation between the two point clouds by finding the relevant correspondences. If you notice that, previously we had created our training data using CAD model. Now, we have only one set of points (keypoints) in the camera measurement. So this single set of camera measurement is used to evaluate our model which is already trained with our CAD training data.

![cam-key](https://user-images.githubusercontent.com/37708330/44930781-b5e54500-ad5f-11e8-81b4-567645c9efaa.PNG)

So the above is the camera measurement point and the corresponding single evaluation data. **Basically, the reason is that our features in a considered combination would be unique for each combination. Also the Eucledian distance features are independent irresepective of whatever the transformation may be.**

Thus we have our training and evaluation data prepared. Now based on generated data we could think about an efficient model or preprocess the data if needed. After reading some research work, I came to know that lot of journals used the 2d depth map or RGBD image data to estimate the pose and orientation of the objects. Then I started with implementation of **Feed forward Neural Network/ Convolutional Neural Network**

The major bottle necks found in using a CNN till now are :

- Basically, the point cloud could be converted into 2d depth map without RGB data but the original geometric property of the point cloud could be lost. So we tried to maintain the point cloud data as it is to ensure that whole geometric property of the data could be handled.
- The welding robot will register only one point cloud file i.e. the existing software is designed in such a way that the robot registers only one point cloud for each work piece. Each work piece could differ from the users input. So there are not much training data as any efficient deep learning method requires lot of training data.
- Difference in size of the point clouds is huge. Since the 3D CAD model will have the complete perception of the workpiece it has more points. But in case of the camera output, partial or a portion of the workpiece could be generated because it largely depends upon the view field of the camera.

Even though it had these major bottle necks, I managed to find an efficient approach. Using Decision Tree/Random Forest Regressor, we could split the training data into possible nodes. Then our evaluation data could be fitted with the possible node which we are looking for. By this method we could estimate which is the right combination of points present in our CAD model corresponds to our Camera point cloud. 

![decision tree](https://user-images.githubusercontent.com/37708330/44932992-c7cae600-ad67-11e8-95bd-9d6bc89804e2.PNG)

So in our model, each node represents a class or each combination of points from CAD key points. Our model is evaluated with the only set of combination from the camera key points. The result will be the node/class which represent a combination of point. Now we have found the corresponding points in CAD model for a given camera point cloud. Finally we estimate the transformation between the two point sets.

### Resolving Problems

The discussed algorithm works well when the data is ideal i.e. if we dont have any deviation in the data, this algorithm holds good. Eg. in the cases of finding transformation between two CAD point cloud data. But major challenges were

- Unfortunately our real time camera measurements contains noise which cannot be avoided. Our algorithm should be roboust enough to handle these noise. 
- Keypoint detection algorithm gives only the key regions. Keypoint is found by taking K Means Clustering algorithm. So deviation of keypoints from the actual corners is likely to happen.
- Symmetric workpieces like cuboid might have same eucledian distances between them.

To handle these issues, I have designed an approach by which only certain combinations of features are selected and trained in the model. Likewise, different sets of features are taken from training data, trained with the decision tree. These decision trees are then predicts for our evaluation data and votes for a class. The class which gets maximum no. of votes will be the required correspondences of points. The reason for doing this is to reduce the features which may be noisy. Not all the features/points will be deviated from the original value. Therefore, by this method we could elliminate the impact of noise in this data.

However, we have to take another strategy to eliminate symmetric points in our data. So, first 15 classes (assumption) which is ranked based on the number of votes will be selected. Then for each of this class, transformation matrix is estimated. With each of this 15 transformation matrix, our source points are transformed and verified with our target points. The difference between these two set of points is calculated and the class which has the lowest deviation will be our required correspondences of points. This could be well explained in the results below.

### Observations & Results

***Measurement 1:***

![measure1](https://user-images.githubusercontent.com/37708330/44956665-e34d0280-aec7-11e8-9966-42a88b2fde7d.PNG)
![measure12](https://user-images.githubusercontent.com/37708330/44956667-e8aa4d00-aec7-11e8-92a9-d9c5fadd797c.PNG)

If we analyze the table above, considering the results of the tree with 4,5,6,7,8,9 out of 10 features, with 9th feature, our algorithm gave a wrong class. This shows that some noisy features are present which votes for the wrong class. By taking a reduced set of features eg. 6 features out of 10, the results were better.


![measure13](https://user-images.githubusercontent.com/37708330/44956668-ea741080-aec7-11e8-8ea3-c0a2d4016cb8.PNG)


Eventhough class 129937 has highest vote, it is a symmetric duplicate of our camera points. Now we check alll the fifteen highest voted class and calculate the corresponding errors. The minimum error is given by the class was 38221 - which is our expected result.


![measure14](https://user-images.githubusercontent.com/37708330/44956669-ec3dd400-aec7-11e8-8329-2786cec87b1d.PNG)

Souce point cloud transformed by the transformation matrix given from the class 38221. 

***Measurement 2:***

![measure2](https://user-images.githubusercontent.com/37708330/44956673-f8299600-aec7-11e8-90bd-820975d23a0a.PNG)

![measure22](https://user-images.githubusercontent.com/37708330/44956675-0081d100-aec8-11e8-81ca-a3f986df8d90.PNG)

If we analyze the table above, considering the results of the tree with 4,5,6,7,8,9 out of 10 features, all different trees resulted in the same class- shows there is not much noise present in this considered measurement.

![measure23](https://user-images.githubusercontent.com/37708330/44956684-2ad38e80-aec8-11e8-9449-33d6609e724a.PNG)

Here, our measurement has no symmetric data corrresponding to the CAD measurement. So our correct correspondences is given the highest vote.

![measure24](https://user-images.githubusercontent.com/37708330/44956689-31fa9c80-aec8-11e8-96e9-2a135245fc5b.PNG)

***Key Obersvations:***

• Key points detected in Harris 3D key point algorithm is crucial for this algorithm to give an accurate result. The generated key points which approximately matched with the exact corner point had low RMS error. In the same time, measurements which had inaccurate key points had comparatively high RMS error.

• Since this algorithm splits the dataset and specifically matches with different combination of features, it could handle key points which are generated inaccurately*.

• It was observed that well-spaced key points gave better results compared with closely placed key points.

• The histogram for predicted class gave better results when number of features considered were chosen in reduced set of combination**.

*If the algorithm is not able to predict the correct correspondence, parameters could be tuned to get nearest possible solution.*

*Considering 5 points in Camera measurement, no. of features formed will be 10. Each point will contribute to 4 features. Suppose we take 6 features out of 10, then we eliminate contribution of one point which may or may not be an accurate point. So similarly, if we take combinations of features, we try to eliminate maximum possible “bad” features. Each of the corresponding trees will vote for a class by which our histogram is obtained. One could say that lesser features would eliminate all possible bad features but it has to be taken into account that more features will improve the performance of model and lesser features will over fit the model. So 6 or 7 was an optimized set of features for this dataset.*

