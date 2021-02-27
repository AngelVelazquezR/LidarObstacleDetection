/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	double rpx1=0;
	double rpy1=0;
	double rpx2=0;
	double rpy2=0;

	double a=0;
	double b=0;
	double c=0;

	while(maxIterations--)
  	{
		
		int ri1 = rand()%(cloud->points.size());
		int ri2 = rand()%(cloud->points.size());

		std::unordered_set<int> inliersP;

  		rpx1 = cloud->points[ri1].x;
  		rpy1 = cloud->points[ri1].y;
		rpx2 = cloud->points[ri2].x;
  		rpy2 = cloud->points[ri2].y;

		a = rpy1-rpy2;
		b = rpx1-rpx2;
		c = rpx1*rpy2 - rpx2*rpy1;

		int ind=0;
		for(pcl::PointXYZ point : cloud->points)
		{
			double d = abs((a*point.x)+(b*point.y)+c)/sqrt((a*a)+(b*b));
			if(d<=distanceTol){
				//cout << "distance: " << d <<endl;
				inliersP.insert(ind);
			}
			ind++;
		}

		if(inliersP.size()>inliersResult.size()){inliersResult=inliersP;}
  	}

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "ransac process took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> PlaneRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	double rpx1=0;
	double rpy1=0;
	double rpz1=0;

	double rpx2=0;
	double rpy2=0;
	double rpz2=0;

	double rpx3=0;
	double rpy3=0;
	double rpz3=0;

	double a=0;
	double b=0;
	double c=0;
	double d=0;

	double i=0;
	double j=0;
	double k=0;

	while(maxIterations--)
  	{
		
		int ri1 = rand()%(cloud->points.size());
		int ri2 = rand()%(cloud->points.size());
		int ri3 = rand()%(cloud->points.size());
		std::unordered_set<int> inliersP;

  		rpx1 = cloud->points[ri1].x;
  		rpy1 = cloud->points[ri1].y;
  		rpz1 = cloud->points[ri1].z;

		rpx2 = cloud->points[ri2].x;
  		rpy2 = cloud->points[ri2].y;
  		rpz2 = cloud->points[ri2].z;

		rpx3 = cloud->points[ri3].x;
  		rpy3 = cloud->points[ri3].y;
  		rpz3 = cloud->points[ri3].z;


		i=((rpy2-rpy1)*(rpz3-rpz1))-((rpz2-rpz1)*(rpy3-rpy1));
		j=((rpz2-rpz1)*(rpx3-rpx1))-((rpx2-rpx1)*(rpz3-rpz1));
		k=((rpx2-rpx1)*(rpy3-rpy1))-((rpy2-rpy1)*(rpx3-rpx1));

		a = i;
		b = j;
		c = k;
		d = -((i*rpx1)+(j*rpy1)+(k*rpz1));
		double divv = sqrt((a*a)+(b*b)+(c*c)); //Avoid div by 0


		int ind=0;
		for(pcl::PointXYZ point : cloud->points)
		{
			double dist = abs((a*point.x)+(b*point.y)+(c*point.z)+d)/divv;
			if(dist<=distanceTol){
				//cout << "distance: " << d <<endl;
				inliersP.insert(ind);
			}
			ind++;
		}

		if(inliersP.size()>inliersResult.size()){inliersResult=inliersP;}
  	}

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers




	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "ransac process took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); //Two dimension
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 100, .9); //Two dimension
	std::unordered_set<int> inliers = PlaneRansac(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
