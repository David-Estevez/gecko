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
	//cv:: imshow ("THRESHOLDED PICTURE", threshold_output);
	
	//-- Find contours in the processed image
	findContours(threshold_output,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	
		for(int i=0;i<contours.size();i++)
		
			//-- Choose only the big enough areas 
			if(contourArea(contours[i])>=5000)		    
			{
				final_contours.push_back(contours[i]);
				
				//-- Draw the contour in the image
				drawContours(ROI,final_contours,-1,cv::Scalar(0,0,255),2);

				//-- Convex Hull
				convexHull(cv:: Mat(final_contours[0]),hulls[0],false);
				convexHull(cv:: Mat(final_contours[0]),hullsI[0],false);
				drawContours(ROI,hulls,-1,cv::Scalar(0,255,0),2);

				//-- Convex Defects

				if(hullsI[0].size()>0)
				{
//					cv::Point2f rect_Points[4]; 
//					for( int j = 0; j < 4; j++ )
//						cv::line( ROI, rect_Points[j], rect_Points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
					cv::Point rough_palm_center;
					convexityDefects(final_contours[0], hullsI[0], defects);
					if(defects.size()>=3)
					{
						std::vector<cv::Point> palm_contour;
						
						for(int j=0;j<defects.size();j++)
						{
							cv::Point startingPoint( final_contours[0][defects[j][0]] );
							cv::Point endingPoint( final_contours[0][defects[j][1]] );
							cv::Point ptFar( final_contours[0][defects[j][2]] );
							
							//-- Add the defects points to the hand center coordinate to average it
							rough_palm_center+=ptFar+startingPoint+endingPoint;
							palm_contour.push_back(ptFar);
							palm_contour.push_back(startingPoint);
							palm_contour.push_back(endingPoint);
							
						}
						
						//-- Average the hand center coordinates (we divide each coordinate by the size of the defects vector multiplied by the three coordinates we sum in each iteration)
												
						rough_palm_center.x/=defects.size()*3;
						rough_palm_center.y/=defects.size()*3;
						
			
						//--
						std::vector< std::pair <double,int> > distancevec;
						for(int i=0;i<palm_contour.size();i++)
							distancevec.push_back(std::make_pair(distance(rough_palm_center,palm_contour[i]),i));
							
							
						//-- Sort the vector to obtain the largest circles first
						sort(distancevec.begin(),distancevec.end());

						//-- Compute the circle for each palm contour obtained until you find one with a radius larger than 0
						std::pair<cv::Point,double> soln_circle;
						for(int i=0;i+2<distancevec.size();i++)
						{
							cv::Point p1=palm_contour[distancevec[i+0].second];
							cv::Point p2=palm_contour[distancevec[i+1].second];
							cv::Point p3=palm_contour[distancevec[i+2].second];
							soln_circle=circle(p1,p2,p3);//Final palm center,radius
							if(soln_circle.second!=0)
								break;
						}

						//Find avg palm centers for the last few ROIs to stabilize its centers, also find the avg radius
						std::vector<std::pair<cv::Point,double> > palm_centers;
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

						//-- The palm center is drawn
						cv::circle(ROI,palm_center,5,cv::Scalar(0,0,255),20);


						//
						//Detect fingers by finding cv::Points that form an almost isosceles triangle with certain thesholds

						for(int j=0;j<defects.size();j++)
						{
							int startidx=defects[j][0]; cv::Point startingPoint( final_contours[0][startidx] );
							int endidx=defects[j][1]; cv::Point endingPoint( final_contours[0][endidx] );
							int faridx=defects[j][2]; cv::Point ptFar( final_contours[0][faridx] );
							//X o--------------------------o Y
							double Xdistance=sqrt(distance(palm_center,ptFar));
							double Ydistance=sqrt(distance(palm_center,startingPoint));
							double length=sqrt(distance(ptFar,startingPoint));

							double retLength=sqrt(distance(endingPoint,ptFar));
							//Play with these thresholds to improve performance
							if(length<=3*radius&&Ydistance>=0.4*radius&&length>=10&&retLength>=10&&std::max(length,retLength)/std::min(length,retLength)>=0.8)
								if(std::min(Xdistance,Ydistance)/std::max(Xdistance,Ydistance)<=0.8)
								{
									if((Xdistance>=0.1*radius&&Xdistance<=1.3*radius&&Xdistance<Ydistance)||(Ydistance>=0.1*radius&&Ydistance<=1.3*radius&&Xdistance>Ydistance))
										line( ROI, endingPoint, ptFar, cv::Scalar(0,255,0), 1 ),n_fingers++;
								}


						}
						
						
						//-- Catch possible aliens
						if (n_fingers>5)
							n_fingers=5; 

						FingerDetector::number_of_fingers=n_fingers; 				
					}
				}

	}
}

double FingerDetector:: distance (cv::Point x, cv::Point y)
{
	return (x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y);	
}

int FingerDetector::getFingers()
{	return FingerDetector::number_of_fingers; }


std::pair<cv::Point,double> FingerDetector::circle(cv::Point p1, cv::Point p2, cv::Point p3)
{
	double offset = pow(p2.x,2) +pow(p2.y,2);
	double bc =   ( pow(p1.x,2) + pow(p1.y,2) - offset )/2.0;
	double cd =   (offset - pow(p3.x, 2) - pow(p3.y, 2))/2.0;
	double det =  (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y); 
	double TOL = 0.0000001;
	if (abs(det) < TOL) 
		return std::make_pair(cv::Point(0,0),0);

	double idet = 1/det;
	double centerx =  (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery =  (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt( pow(p2.x - centerx,2) + pow(p2.y-centery,2));

	return std::make_pair(cv::Point(centerx,centery),radius);
}

