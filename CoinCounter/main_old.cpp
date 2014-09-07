#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


int sliderPos = 70;
Mat image, img, img_th, plate_img;
void processImage(int, void*);

int thresh = 120;
int max_thresh = 255;
RNG rng(12345);
void thresh_callback(int, void* );

void findEllipses(int, void*);

int selected_point = 0;
int selected_contour = 0;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<vector<Point> > seglist;
vector<vector<Point> > curves;
vector<Scalar> split_reasons;
vector<RotatedRect> ellipses;
vector<vector<Point> > ellipse_points;

void draw();

Scalar colors[256];


// TODO: clean up all
// basic method: thresh or canny + contours + ellipse finding
// find plate with coins, tfrom plate image to orthoganal view
// re-run canny

int main()
{

	for (int i = 0; i < 256; i++)
	{
		colors[i] = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
	}

	char * output = "out10.png";
    //Mat img_gray, canny img;
	img = imread("sample10.png");
    cvtColor(img, image, CV_BGR2GRAY);
	//if (!img.empty()) cv::imshow("OpenCV",img);
	


	//
	// Finding checkerboard - fail

	//std::vector<cv::Point2f> corners;
	//cv::Size patsize(4,7);
	//bool found = cv::findChessboardCorners(img, patsize, corners);
	//cv::drawChessboardCorners(img, patsize, corners, found);
	
	//
	// Hough Circles - fail
	//
	/*
	double threshold = 80;
	cv::cvtColor(img, img_gray, CV_BGR2GRAY);
	cv::GaussianBlur(img_gray, img_gray, cv::Size(9, 9), 2, 2);
	cv::Canny(img_gray, canny, threshold/2.0, threshold);
	cv::imshow("canny", canny);
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 1, img_gray.rows/32, threshold, 35, 0, 0);
	for( size_t i = 0; i < circles.size(); i++ )
	{
	   cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	   int radius = cvRound(circles[i][2]);
	   cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	   cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	 }
	 */

	// next try - ellipses ... 

	// find ellipses in images, use minor dimensions to unskew, pattern match (need to look into methods) to find coin
	// then use major dimension with know actual size to calculate distance.

	// READ SPEC FIRST

	int64 start = getTickCount();

	blur( image, image, Size(3,3) );


	//adaptiveThreshold(image, img_th, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, 0);
	//threshold(image, img_th, 100, 255, THRESH_BINARY);

	//imshow("thresh", img_th);

	namedWindow("Canny", CV_WINDOW_AUTOSIZE);
	//createTrackbar( "Canny thresh:", "Contours", &thresh, max_thresh, findEllipses );
	findEllipses(0,0);

	double time = (getTickCount() - start)/getTickFrequency();
	printf("elapsed time: %f", time);
	/*imshow("source", image);
    namedWindow("result", 1);

	createTrackbar( " Canny thresh:", "source", &thresh, max_thresh, thresh_callback );
	thresh_callback( 0, 0 );

	

    // Create toolbars. HighGUI use.
    createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
    processImage(0, 0);*/


	//imshow("OpenCV",img);

	imwrite(output, img);

    
	while(1)
	{
		int key = waitKey();
		switch (key)
		{
		case 2490368: //up
			selected_contour++;
			//if (selected_contour >= seglist.size()) selected_contour = seglist.size()-1;
			selected_point = 0;
			draw();
			break;
		case 2621440: //down
			selected_contour--;
			if (selected_contour < 0) selected_contour = 0;
			selected_point = 0;
			draw();
			break;
		case 2424832: //left
			selected_point--;
			if (selected_point < 0) selected_point = 0;
			draw();
			break;
		case 2555904: //right
			selected_point++;
			//if (selected_point >= seglist[selected_contour].size()) selected_point = seglist[selected_contour].size()-1;
			draw();
			break;
		case 27://esc
			return EXIT_SUCCESS;
		}
		printf("%d\n", selected_contour);
	}

    return EXIT_SUCCESS;
}

void draw()
{
	Mat drawing1 = Mat::zeros(image.size(), CV_8UC3);
	Mat drawing2 = Mat::zeros(image.size(), CV_8UC3);
	Mat drawing3 = Mat::zeros(image.size(), CV_8UC3);

	// Draw contours
	for(int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 20)
		{
			int line_width = (i == selected_contour) ? 3 : 1;
			drawContours(drawing1, contours, i, colors[i%256], line_width, 8, hierarchy, 0);
		}
	}

	// draw edge verts
	for(int i = 0; i < seglist.size(); i++)
	{
		for (int j = 0; j < seglist[i].size(); j++)
		{
			Scalar col = (i == selected_contour && j == selected_point) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
			int size = (i == selected_contour) ? 5 : 2;
			circle(drawing1, seglist[i][j], size, col);
		}
	}
	
	// Draw curves
	for(int i = 0; i < curves.size(); i++)
	{
		int line_width = (i == selected_contour) ? 3 : 1;
		int size = (i == selected_contour) ? 5 : 2;
		for(int j = 0; j < curves[i].size()-1; j++)
		{
			line(drawing2, curves[i][j], curves[i][j+1], colors[i%256], line_width);

			Scalar col = (i == selected_contour && j == selected_point) ? Scalar(0, 255, 0) : split_reasons[i];
			circle(drawing2, curves[i][j], size, col);
		}
		Scalar col = (i == selected_contour && (curves[i].size()-1) == selected_point) ? Scalar(0, 255, 0) : split_reasons[i];
		circle(drawing2, curves[i][curves[i].size()-1], size, col);
	}

	// Draw ellipses
	// TODO: also draw angle directions
	for (int i = 0; i < ellipses.size(); i++)
	{
		//ellipse(img, ellipses[i], Scalar(0,0,255), 1, CV_AA);
        ellipse(img, ellipses[i].center, ellipses[i].size*0.5f, ellipses[i].angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        Point2f vtx[4];
        ellipses[i].points(vtx);
        for( int j = 0; j < 4; j++ )
            line(img, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);

		/*for (int j = 0; j < ellipse_points[i].size(); j++)
		{
			circle(img, ellipse_points[i][j], 2, Scalar(255,0,0));
		}*/
	}

	/// Show in a window
	imshow("Contours", drawing1);
	imshow("Curves", drawing2);
	imshow("Ellipses", img);
}

float maxLineDeviation(vector<Point> points, int first, int last, int *index)
{
	int count = last - first;
	if (count <= 1)
	{
		*index = last;
		return 0.f;
	}

	// find line coefficients (of form a*x + b*y + c = 0)
	float a = points[first].y - points[last].y;
	float b = points[last].x - points[first].x;
	float c = points[first].x * points[last].y - points[last].x * points[first].y;

	float length = sqrt(a*a + b*b);

	float d_max = 0.f;
	int idx = last;

	if (length < 0.0001f) // same point at start and finish (i.e. loop)
	{
		for (int i = first; i <= last; i++)
		{
			float dx = points[i].x - points[first].x;
			float dy = points[i].y - points[first].y;
			float d = dx*dx + dy*dy;
			if (d > d_max)
			{
				d_max = d;
				idx = i;
			}
		}
		d_max = sqrt(d_max);
	}
	else
	{
		for (int i = first; i <= last; i++)
		{
			float d = abs(a*points[i].x + b*points[i].y + c)/length;
			if (d > d_max)
			{
				d_max = d;
				idx = i;
			}
		}
	}
	*index = idx;
	return d_max;
}

// TODO: running canny on colour image gives many more edges, how to utilize this? 

// also need to take into consideration rotation of ellipses when find plate!

void findEllipses(int, void*)
{
	Mat canny_output;
	float tol = 2.f;

	selected_point = 0;
	selected_contour = 0;

	/// Detect edges using canny
	Canny(image, canny_output, thresh, thresh*2, 3);

	/*Point neighbours[8] = { Point(-1,-1),
							Point( 0,-1),
							Point( 1,-1),
							Point( 1, 0),
							Point( 1, 1),
							Point( 0, 1),
							Point(-1, 1),
							Point(-1, 0) };*/

	imshow("Canny", canny_output);
	/*imwrite("cannyout.png", canny_output);
	Mat cimg = canny_output;
	for(int row = 0; row < cimg.rows; ++row) {
		uchar* p = cimg.ptr(row);
		for(int col = 0; col < cimg.cols; ++col) {
			if (*p)
			{
				if (row > 0 && row < cimg.rows-1 && col > 0 && col < cimg.cols-1) // no need to check border conditions
				cimg.at(
			}
			p++;
		}
	}*/

	contours.clear();
	hierarchy.clear();
	seglist.clear();
	curves.clear();
	ellipses.clear();
	ellipse_points.clear();

	/*vector<Vec3f> circles;
	HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, 10, thresh*2, 40, 10, 100);
	for( size_t i = 0; i < circles.size(); i++ )
	{
	   Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	   int radius = cvRound(circles[i][2]);
	   // circle center
	   circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
	   // circle outline
	   circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	 }

	imshow("hough", img);*/

	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));

	/*Mat cimage = Mat::zeros(canny_output.size(), CV_8UC3);

    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 20 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*4 )
            continue;

		if ( MAX(box.size.width, box.size.height) > 200 )
			continue;

		if ( MIN(box.size.width, box.size.height) < 20 )
			continue;

		drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);

        ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
        ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ )
            line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);
    }
	imshow("Contours", cimage);*/
	
	for(int i = 0; i < contours.size(); i++)
	{
		seglist.push_back(vector<Point>());
		if (contours[i].size() > 20)
		{
			int first = 0, last = contours[i].size()-1;
			seglist.back().push_back(Point(contours[i][first]));

			while (first < last)
			{
				int idx;
				float dev = maxLineDeviation(contours[i], first, last, &idx);

				while (dev > tol)
				{
					last = idx;
					dev = maxLineDeviation(contours[i], first, last, &idx);
				}

				seglist.back().push_back(Point(contours[i][last]));

				first = last;
				last = contours[i].size()-1;
			}
		}
	}

	// seperate edge contours into individual curves
	for(int i = 0; i < seglist.size(); i++)
	{
		if (seglist[i].size())
		{
			curves.push_back(vector<Point>());
			split_reasons.push_back(Scalar(0, 0, 255));
		}
		int k = 0;
		for (int j = 0; j < seglist[i].size(); j++)
		{
			curves.back().push_back(Point(seglist[i][j]));
			bool split = false;
			// curvature condition
			if (k > 1 && j < (seglist[i].size()-1)) {
				Point a = seglist[i][j-1] - seglist[i][j-2];
				Point b = seglist[i][j] - seglist[i][j-2];
				Point c = seglist[i][j+1] - seglist[i][j-2];
				
				float a1 = atan2f(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
				float a2 = atan2f(a.x*c.y - a.y*c.x, a.x*c.x + a.y*c.y);

				split = split || (a1*a2 < 0) || (abs(a2)-abs(a1) < 0);

				if (split) split_reasons.push_back(Scalar(255,0,0));
			}

			// this just seems to cause false splits...
			/*if (!split && k > 0 && j < (seglist[i].size()-2)) {
				Point a = seglist[i][j+1] - seglist[i][j+2];
				Point b = seglist[i][j] - seglist[i][j+2];
				Point c = seglist[i][j-1] - seglist[i][j+2];
				
				float a3 = atan2f(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
				float a4 = atan2f(a.x*c.y - a.y*c.x, a.x*c.x + a.y*c.y);

				split = split || (a3*a4 < 0) || (abs(a4)-abs(a3) < 0);

				if (split) split_reasons.push_back(Scalar(255,255,0));
			}*/

			// length condition
			/*if (!split && k > 0 && j < (seglist[i].size()-1)) {
				Point seg1 = seglist[i][j] - seglist[i][j-1];
				Point seg2 = seglist[i][j] - seglist[i][j+1];

				float l1 = seg1.x*seg1.x + seg1.y*seg1.y;
				float l2 = seg2.x*seg2.x + seg2.y*seg2.y;

				split = split || (l1 > 16*l2) || (16*l1 < l2);

				if (split) split_reasons.push_back(Scalar(255,0,255));
			}*/

			//angle condition
			/*if (!split && k > 1 && j < (seglist[i].size()-2)) {
				Point a = seglist[i][j-2] - seglist[i][j-1];
				Point b = seglist[i][j] - seglist[i][j-1];
				Point c = seglist[i][j+2] - seglist[i][j+1];
				Point d = seglist[i][j] - seglist[i][j+1];
				float la = a.x*a.x + a.y*a.y;
				float lb = b.x*b.x + b.y*b.y;
				float lc = c.x*c.x + c.y*c.y;
				float ld = d.x*d.x + d.y*d.y;

				float b1 = acos(a.dot(b)/sqrtf(la*lb));
				float b3 = acos(c.dot(d)/sqrtf(lc*ld));
				float b2 = acos((-b).dot(-c)/sqrtf(lb*lc));

				float theta_thresh = 0.785f; // ~45 deg
				split = split || (abs(b1) - abs(b2) > theta_thresh) || (abs(b3) - abs(b2) > theta_thresh);
			}*/

			if (split)
			{
				curves.push_back(vector<Point>());
				curves.back().push_back(Point(seglist[i][j]));
				k = 0;
			}
			k++;
		}
	}

	// TODO: join adjoining curves ... ?

	// TODO: join global matching curves ... ? 


	Point checkpoints[21] = { Point(0,0),
							  Point(1,0),
							  Point(-1,0),
							  Point(0,1),
							  Point(0,-1),
							  Point(1,1),
							  Point(1,-1),
							  Point(-1,1),
							  Point(-1,-1),
							  Point(2,0),
							  Point(-2,0),
							  Point(0,2),
							  Point(0,-2),
							  Point(1,2),
							  Point(1,-2),
							  Point(-1,2),
							  Point(-1,-2),
							  Point(2,1),
							  Point(2,-1),
							  Point(-2,1),
							  Point(-2,-1) };
	float point_weights[21];
	for (int i = 0; i < 21; i++)
	{
		point_weights[i] = checkpoints[i].x * checkpoints[i].x + checkpoints[i].y * checkpoints[i].y;
	}

	// try fitting ellipses to points
	vector<RotatedRect> temp_ellipses;
	vector<RotatedRect> possible_plates;
	vector<vector<RotatedRect> > plate_support;
	for (int i = 0; i < curves.size(); i++)
	{
		if (curves[i].size() > 4)
		{
			RotatedRect box = fitEllipse(curves[i]);


			// check ellipse size/shape constraints
			if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*2 ) // max AR
				continue;
			if ( MIN(box.size.width, box.size.height) < 4 ) // coin min size
				continue;
			if ( MAX(box.size.width, box.size.height) > 500 ) // plate max size - necessary...?
				continue;

			//ellipses.push_back(box);

			if ( MIN(box.size.width, box.size.height) > 80 ) // plate min size
			{
				// possible plate - check validity

				vector<Point> e_points;
				ellipse2Poly(box.center, box.size*0.5f, box.angle, 0, 360, 10, e_points);
				if (e_points.size() != 37) printf("what... e_points.size() = %d\n", e_points.size());
				float error = 0;
				for (int j = 0; j < e_points.size(); j++)
				{
					bool found = false;
					bool onscreen = false;
					for (int k = 0; k < 21; k++)
					{
						Point p = e_points[j] + checkpoints[k];
						if (p.x >= 0 && p.y >= 0 && p.x < canny_output.cols && p.y < canny_output.rows)
						{
							onscreen = true;
							if (canny_output.at<uchar>(p))
							{
								found = true;
								error += point_weights[k];
								break;
							}
						}
					}
					if (!found & onscreen) error += 3.f;
				}

				if (error < 40.f) ///... should be less, but its not detecting sometimes...
				{
					possible_plates.push_back(box);
					plate_support.push_back(vector<RotatedRect>());
				}
			}
			else
			{
				// not a plate, possibly a coin
				temp_ellipses.push_back(box);
			}

			// check validity of ellipse by checking edge map
			// possible method - use ellipse2Poly to get a vec of points, check d^2 from each point to edge (or just weighted voting?)

			//ellipse_points.push_back(vector<Point>());
		}
	}

	if (possible_plates.size() == 0) 
	{
		printf("crap... no plates\n");
	}

	for (int i = 0; i < temp_ellipses.size(); i++)
	{
		for (int j = 0; j < possible_plates.size(); j++)
		{
			Point offset = temp_ellipses[i].center - possible_plates[j].center;
			float d_sq = offset.x*offset.x+offset.y*offset.y;
			float max_radius = MAX(possible_plates[j].size.height, possible_plates[j].size.width)*0.5f; 
			float max_coin_radius = MAX(temp_ellipses[i].size.height, temp_ellipses[i].size.width)*0.5f;
			// hacky - should actually be checking to see if ellipse is completely within the other one
			if (d_sq < max_radius * max_radius)
			{
				if (max_coin_radius < max_radius*0.3f) // make sure coin is signifigantly smaller than plate
				{
					// ellipse is (roughly) within the possible plate
					float plate_AR = (float)possible_plates[j].size.height / (float)possible_plates[j].size.width;
					float ellipse_AR = (float)temp_ellipses[i].size.height / (float)temp_ellipses[i].size.width;
					if (ellipse_AR > plate_AR*0.75f && ellipse_AR < plate_AR*1.25f)
					{
						// ellipses have similar aspect ratios (likely co-planer)
						// should also test angle (only if aspect ratio is not close to 1.0)

						plate_support[j].push_back(temp_ellipses[i]); // add this ellipse to this plate group
						break; // only add to first prospective plate
					}
				}
			}
		}
	}

	int best_plate = 0;
	int max_coins = 0;
	for (int i = 0; i < plate_support.size(); i++)
	{
		if (plate_support[i].size() > max_coins)
		{
			best_plate = i;
			max_coins = plate_support[i].size();
		}
	}

	if (possible_plates.size())
	{
		ellipses.push_back(possible_plates[best_plate]);

		for (int i = 0; i < plate_support[best_plate].size(); i++)
		{
			ellipses.push_back(plate_support[best_plate][i]);
		}
	}


	//TODO: set ROI before tfrom

	int size = MAX(possible_plates[best_plate].size.width, possible_plates[best_plate].size.height);

	plate_img = Mat::zeros(size,size,CV_8UC3);

	Point2f plate_corners[4];
	Point2f img_corners[4] = {Point2f(0,0), Point2f(size,0), Point2f(size,size), Point2f(0,size)};
	possible_plates[best_plate].points(plate_corners);
	Mat tform = cv::getPerspectiveTransform(plate_corners, img_corners);
	cv::warpPerspective(img, plate_img, tform, plate_img.size(), INTER_LINEAR);

	imshow("warped", plate_img);

	Mat plate_grey;
	cvtColor(plate_img, plate_grey, CV_BGR2GRAY);

	Mat plate_edges, plate_thresh;
	int param1 = 240;
	cv::Canny(plate_img, plate_edges, param1/2, param1);

	cv::adaptiveThreshold(plate_grey, plate_thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, 0);
	imshow("plate thresh", plate_thresh);

	vector<Vec3f> circles;
	HoughCircles(plate_grey, circles, CV_HOUGH_GRADIENT, 1, 6, param1, 20, 3, size/5);
	for( size_t i = 0; i < circles.size(); i++ )
	{
	   Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	   int radius = cvRound(circles[i][2]);
	   // circle center
	   circle( plate_img, center, 3, Scalar(0,255,0), -1, 8, 0 );
	   // circle outline
	   circle( plate_img, center, radius, Scalar(0,0,255), 3, 8, 0 );
	 }

	imshow("hough", plate_img);

	
	imshow("plate edges", plate_edges);

	draw();
}



// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void processImage(int /*h*/, void*)
{
    vector<vector<Point> > contours;
    Mat bimage = image >= sliderPos;
	imshow("bimage", bimage);
    findContours(bimage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    Mat cimage = Mat::zeros(bimage.size(), CV_8UC3);

    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;
        drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);

        ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
        ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ )
            line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);
    }

    imshow("result", cimage);
}



/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}