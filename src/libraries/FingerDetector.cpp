#include"FingerDetector.h"

FingerDetector :: FingerDetector (){}

FingerDetector :: FingerDetector (cv::Mat & ROI, int & fingers)
{
	//-- Declare contours and final_contours
	std::vector <std::vector<cv::Point> > contours; 
	std::vector<std::vector<cv::Point> > final_contours;
	std::vector<std::vector<cv::Point> > hulls(1);
	std::vector<std::vector<int> > hullsI(1);
	std::vector<cv::Vec4i> defects;
	
	int n_fingers=0;
	
	cv::Mat bwROI;
	cv::Mat threshold_output;
	cv::cvtColor(ROI, bwROI, CV_RGB2GRAY);
	
	//-- Detect edges using Threshold
	cv::threshold( bwROI, threshold_output, 100, 255, cv::THRESH_BINARY );
	//-- Find contours in the processed image
	findContours(threshold_output,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	
		for(int i=0;i<contours.size();i++)
		
			//-- Choose only the big enough areas 
			if(contourArea(contours[i])>=5000)		    
			{
				final_contours.push_back(contours[i]);
				
				//-- Draw the contour in the image
				drawContours(ROI,final_contours,-1,cv::Scalar(0,0,255),2);
/*
				//-- Convex Hull
				convexHull(cv:: Mat(final_contours[0]),hulls[0],false);
				convexHull(cv:: Mat(final_contours[0]),hullsI[0],false);
				drawContours(ROI,hulls,-1,cv::Scalar(0,255,0),2);

				//-- Convex Defects

				if(hullsI[0].size()>0)
				{
					cv::Point2f rect_points[4]; 
					for( int j = 0; j < 4; j++ )
						cv::line( ROI, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
					cv::Point rough_palm_center;
					convexityDefects(final_contours[0], hullsI[0], defects);
					if(defects.size()>=3)
					{
						std::vector<cv::Point> palm_points;
						for(int j=0;j<defects.size();j++)
						{
							int startidx=defects[j][0]; cv::Point ptStart( final_contours[0][startidx] );
							int endidx=defects[j][1]; cv::Point ptEnd( final_contours[0][endidx] );
							int faridx=defects[j][2]; cv::Point ptFar( final_contours[0][faridx] );
							//Sum up all the hull and defect points to compute average
							rough_palm_center+=ptFar+ptStart+ptEnd;
							palm_points.push_back(ptFar);
							palm_points.push_back(ptStart);
							palm_points.push_back(ptEnd);
							
							//circle(ROI,cv:: Point (defects[j][0],5,cv::Scalar(0,0,255),3);
						}
				*/	/*	
						//Get palm center by 1st getting the average of all defect points, this is the rough palm center,
						//Then U chose the closest 3 points ang get the circle radius and center formed from them which is the palm center.
						rough_palm_center.x/=defects.size()*3;
						rough_palm_center.y/=defects.size()*3;
						cv::Point closest_pt=palm_points[0];
						std::vector< pair <double,int> > distvec;
						for(int i=0;i<palm_points.size();i++)
							distvec.push_back(make_pair(dist(rough_palm_center,palm_points[i]),i));
						sort(distvec.begin(),distvec.end());

						//Keep choosing 3 points till you find a circle with a valid radius
						//As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
						pair<Point,double> soln_circle;
						for(int i=0;i+2<distvec.size();i++)
						{
							cv::Point p1=palm_points[distvec[i+0].second];
							cv::Point p2=palm_points[distvec[i+1].second];
							cv::Point p3=palm_points[distvec[i+2].second];
							soln_circle=circleFromPoints(p1,p2,p3);//Final palm center,radius
							if(soln_circle.second!=0)
								break;
						}

						//Find avg palm centers for the last few ROIs to stabilize its centers, also find the avg radius
						palm_centers.push_back(soln_circle);
						if(palm_centers.size()>10)
							palm_centers.erase(palm_centers.begin());

						cv::Point palm_center;
						double radius=0;
						for(int i=0;i<palm_centers.size();i++)
						{
							palm_center+=palm_centers[i].first;
							radius+=palm_centers[i].second;
						}
						palm_center.x/=palm_centers.size();
						palm_center.y/=palm_centers.size();
						radius/=palm_centers.size();

						//Draw the palm center and the palm circle
						//The size of the palm gives the depth of the hand
						circle(ROI,palm_center,5,cv::Scalar(144,144,255),3);
						circle(ROI,palm_center,radius,cv::Scalar(144,144,255),2);

						//Detect fingers by finding points that form an almost isosceles triangle with certain thesholds

						for(int j=0;j<defects.size();j++)
						{
							int startidx=defects[j][0]; Point ptStart( tcontours[0][startidx] );
							int endidx=defects[j][1]; Point ptEnd( tcontours[0][endidx] );
							int faridx=defects[j][2]; Point ptFar( tcontours[0][faridx] );
							//X o--------------------------o Y
							double Xdist=sqrt(dist(palm_center,ptFar));
							double Ydist=sqrt(dist(palm_center,ptStart));
							double length=sqrt(dist(ptFar,ptStart));

							double retLength=sqrt(dist(ptEnd,ptFar));
							//Play with these thresholds to improve performance
							if(length<=3*radius&&Ydist>=0.4*radius&&length>=10&&retLength>=10&&max(length,retLength)/min(length,retLength)>=0.8)
								if(min(Xdist,Ydist)/max(Xdist,Ydist)<=0.8)
								{
									if((Xdist>=0.1*radius&&Xdist<=1.3*radius&&Xdist<Ydist)||(Ydist>=0.1*radius&&Ydist<=1.3*radius&&Xdist>Ydist))
										line( ROI, ptEnd, ptFar, cv::Scalar(0,255,0), 1 ),n_fingers++;
								}


						}
						
						
			
						if (n_fingers>5)
							n_fingers=5; 

						std::cout<<"NO OF FINGERS: "<<n_fingers<<std::endl;

					}*/
			//	}

	}
}
