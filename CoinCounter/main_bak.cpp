#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;

#define MIN_CONTOUR_SIZE 10
#define MIN_PLATE_SIZE 100

#define PI 3.141592653589793

enum COIN_TYPES { _5_CENT, _10_CENT, _20_CENT, _50_CENT, _1_DOLLAR, _2_DOLLAR, NUM_COIN_TYPES };
const char * const COIN_NAMES[NUM_COIN_TYPES] = { "5c", "10c", "20c", "50c", "$1", "$2" };
const float COIN_SIZES[NUM_COIN_TYPES] = { 19.41f, 23.6f, 28.65f, 32.f, 25.f, 20.5f };
const float COIN_VALUES[NUM_COIN_TYPES] = { 0.05f, 0.1f, 0.2f, 0.5f, 1.f, 2.f };

enum GC_METHOD { GC_CANNY, GC_CANNY_GREY, GC_THRESH, GC_ADAPT_THRESH };
enum CHECKPOINTS_TYPE { CP_NONE, CP_POSSIBLE_PLATES, CP_ALL };
enum ELLIPSE_TYPE {EL_INVALID = 0, EL_VALID, EL_POSSIBLE_PLATE};

void GetContours(cv::Mat &image, cv::Mat &edges, vector<vector<cv::Point> > &contours, GC_METHOD method, int param, int step_num);
void FindPlate(cv::Mat &image, vector<vector<cv::Point> > &contours, vector<cv::RotatedRect> &output, vector<cv::Point2d> &frame_points, bool simple_contours = true, CHECKPOINTS_TYPE cp_type = CP_NONE, float cp_err_thresh = 0.f, cv::Mat &edges = cv::Mat(), int canny_thresh = 0.f, int step_num = 1);
ELLIPSE_TYPE ValidateEllipse(cv::RotatedRect e, float min_dia = 2.f, float max_dia = 1000.f, float min_AR = 0.5f, float max_AR = 2.f, CHECKPOINTS_TYPE checkPoints = CP_NONE, cv::Mat &edges = cv::Mat(), float err_thresh = 0.f, float *err = NULL);
void SegmentContours(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, float max_dev);
void FindCurves(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels, cv::Size img_size, float max_dev, int step_num);
float maxLineDeviation(vector<cv::Point> &points, int first, int last, int *index);
void GetCurves(vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels);

cv::Scalar colors[256];

cv::Size image_size;

float point_weights[21];
const cv::Point checkpoints[21] = { 
							cv::Point( 0, 0), cv::Point( 1, 0), cv::Point(-1, 0), cv::Point( 0, 1),
							cv::Point( 0,-1), cv::Point( 1, 1), cv::Point( 1,-1), cv::Point(-1, 1),
							cv::Point(-1,-1), cv::Point( 2, 0), cv::Point(-2, 0), cv::Point( 0, 2),
							cv::Point( 0,-2), cv::Point( 1, 2), cv::Point( 1,-2), cv::Point(-1, 2),
							cv::Point(-1,-2), cv::Point( 2, 1), cv::Point( 2,-1), cv::Point(-2, 1),
							cv::Point(-2,-1) };


void drawCurves(cv::Size size, vector<vector<cv::Point> > curves, int selected)
{
	cv::Mat img_curves = cv::Mat::zeros(size*2, CV_8UC3);
	for(int i = 0; i < curves.size(); i++)
	{
		int linesize = i == selected ? 3 : 1;
		if (curves[i].size())
		{
			for (int j = 0; j < curves[i].size()-1; j++)
				line(img_curves, curves[i][j]*2, curves[i][j+1]*2, colors[i%256], linesize);
			for (int j = 0; j < curves[i].size(); j++)
				circle(img_curves, curves[i][j]*2, 2, cv::Scalar(0,0,255));
		}
	}

	cv::imshow("curves interactive", img_curves);
}

float getAngularSeperation(cv::Point2d a, cv::Point2d b, cv::Mat cam_inv)
{
	cv::Mat p1 = cv::Mat(cv::Matx31d(a.x, a.y, 1.f));
	cv::Mat p2 = cv::Mat(cv::Matx31d(b.x, b.y, 1.f));
	p1 = cam_inv * p1;
	p2 = cam_inv * p2;
	return acos(p1.dot(p2)/(cv::norm(p1)*cv::norm(p2)));
}

void cameraCalibrationRoutine();

int main()
{
	//cameraCalibrationRoutine();
	//return 0;


	

	cv::VideoCapture webcam(0);
	if (!webcam.isOpened()) return EXIT_FAILURE;
	
	cv::Mat cameraMatrix, distCoeffs;
	cv::FileStorage camera_calib_file("camera_calibration.yml", cv::FileStorage::READ);
	camera_calib_file["cameraMatrix"] >> cameraMatrix;
	camera_calib_file["distCoeffs"] >> distCoeffs;
	camera_calib_file.release();

	cv::Mat map1, map2, temp_img;
	webcam >> temp_img;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, temp_img.size(), CV_32FC1, map1, map2);

	/*int key = 0;
	while (key != 27)
	{
		cv::Mat frame, out_img;
		webcam >> frame;

		//cv::undistort(frame, out_img, cameraMatrix, distCoeffs);
		cv::remap(frame, out_img, map1, map2, cv::INTER_CUBIC);
		cv::imshow("cam", frame);
		cv::imshow("Calibration", out_img);
		key = cv::waitKey(10);
	}

	return 0;*/


	cv::RNG rng(12345);
	for (int i = 0; i < 256; i++)
		colors[i] = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));

	for (int i = 0; i < 21; i++)
		point_weights[i] = checkpoints[i].x * checkpoints[i].x + checkpoints[i].y * checkpoints[i].y;

	// load image
	cv::Mat img_edges, img_orig;// = cv::imread("sample10.png");
	webcam >> img_orig;
	image_size = img_orig.size();

	int64 start_time = cv::getTickCount();

	cv::remap(img_orig, img_orig, map1, map2, cv::INTER_CUBIC);

	// get contours (multiple possible ways)
	vector<vector<cv::Point> > contours;
	GetContours(img_orig, img_edges, contours, GC_CANNY, 200, 1);

	// find plate
	vector<cv::Point2d> frame_points(5);
	vector<cv::RotatedRect> plate_and_coins;
	FindPlate(img_orig, contours, plate_and_coins, frame_points, true, CP_POSSIBLE_PLATES, 2.5f, img_edges, 200, 2);

	vector<vector<cv::Point> > plate_curves;
	cv::Mat img_plate;
	if (plate_and_coins.size() > 0)
	{
		double time0 = cv::getTickCount();

		// warp to orthogonal view
		/*int plate_size = MAX(plate_and_coins[0].size.width, plate_and_coins[0].size.height);
		img_plate = cv::Mat::zeros(plate_size,plate_size,CV_8UC3);

		cv::Point2f plate_corners[4];
		cv::Point2f img_corners[4] = {cv::Point2f(0,0), cv::Point2f(plate_size,0), cv::Point2f(plate_size,plate_size), cv::Point2f(0,plate_size)};
		plate_and_coins[0].points(plate_corners);
		cv::Mat tform = cv::getPerspectiveTransform(plate_corners, img_corners);
		cv::warpPerspective(img_orig, img_plate, tform, img_plate.size(), cv::INTER_LINEAR);*/

		cv::Rect roi = plate_and_coins[0].boundingRect() & cv::Rect(0,0,img_orig.size().width, img_orig.size().height);
		cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
		cv::RotatedRect mask_ellipse = plate_and_coins[0];
		mask_ellipse.center.x -= roi.x;
		mask_ellipse.center.y -= roi.y;
		cv::ellipse(mask, mask_ellipse, cv::Scalar(255), -1);
		img_orig(roi).copyTo(img_plate, mask);
		
		double time = (cv::getTickCount() - time0)/cv::getTickFrequency();
		printf("Step 4: %f\n", time);

		imshow("Step 4: extract and warp plate ellipse", img_plate);

		/*vector<cv::Mat> ch;
		cv::split(img_plate, ch);
		cv::imshow("b", ch[0]);
        cv::imshow("g", ch[1]);
        cv::imshow("r", ch[2]);

		cv::blur(ch[0], ch[0], cv::Size(5,5));
		cv::blur(ch[1], ch[1], cv::Size(5,5));
		cv::blur(ch[2], ch[2], cv::Size(5,5));

		cv::Mat temp;
		char chan[3] = { 'b', 'g', 'r' };
		char buf[32];

		vector<cv::Vec3f> coin_circles;
		for (int i = 0; i < 3; i++)
		{
			coin_circles.clear();
			cv::HoughCircles(ch[i], coin_circles, CV_HOUGH_GRADIENT, 1, 5, 180, 20.f, 5, plate_size*0.3f);
			
			temp = ch[i].clone();
			for( size_t i = 0; i < coin_circles.size(); i++ )
			{
				cv::circle(temp, cv::Point(coin_circles[i][0], coin_circles[i][1]), coin_circles[i][2], cv::Scalar(255,0,0), 1, 8);
			}

			sprintf(buf, "%c hough circles", chan[i]);
			cv::imshow(buf, temp);

		}

		for (int thresh = 40; thresh < 255; thresh += 40)
		{
			for (int i = 0; i < 3; i++)
			{
				cv::threshold(ch[i], temp, thresh, 255, CV_THRESH_BINARY);
				sprintf(buf, "%c th = %d", chan[i], thresh);
				cv::imshow(buf, temp);
			}
		}*/

		float plate_dia = MAX(plate_and_coins[0].size.height, plate_and_coins[0].size.width);
		float plate_min_dia = MIN(plate_and_coins[0].size.height, plate_and_coins[0].size.width);
		float plate_AR = plate_and_coins[0].size.height / plate_and_coins[0].size.width;
		float plate_angle = plate_and_coins[0].angle;

		// TODO: Step 6: run full ellipse/circle finding on small image
		cv::Mat img_plate_edges;
		vector<vector<cv::Point> > plate_contours2;
		GetContours(img_plate, img_plate_edges, plate_contours2, GC_THRESH, 150, 12);

		vector<vector<cv::Point> > plate_contours;
		GetContours(img_plate, img_plate_edges, plate_contours, GC_CANNY, 150, 5);

		// find coins...
		//vector<vector<cv::Point> > plate_curves;
		vector<vector<cv::Point> > plate_curve_pixels;
		FindCurves(plate_contours, plate_curves, plate_curve_pixels, img_plate.size(), 1.2f, 6);

		struct Coin {
			Coin(cv::RotatedRect _e, float _err, vector<cv::Point> *_pts) : e(_e), err(_err), pts(_pts) {};
			cv::RotatedRect e;
			float err;
			vector<cv::Point> *pts;
		};

		cv::Mat img_coin_ellipses = img_plate.clone();
		//vector<cv::RotatedRect> possible_coins;
		vector<Coin> possible_coins;
		double d2r = PI / 180.0;
		// TODO: fix all this duplicated code...
		for (int i = 0; i < plate_curve_pixels.size(); i++)
		{
			if (plate_curve_pixels[i].size() > 4) // need 5 or more points to fit ellipse
			{
				cv::RotatedRect ellipse = cv::fitEllipse(plate_curve_pixels[i]);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, .06f*plate_dia, .15f*plate_dia, .8f*plate_AR, 1.25f*plate_AR, CP_ALL, img_plate_edges, 1.5f, &error);
				if (e_type > 0)
				{
					float e_AR = ellipse.size.height / ellipse.size.width;
					if (e_AR < 0.85f || e_AR > 1.15f) {
						float angle_diff = plate_angle - ellipse.angle;
						angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
						if (abs(angle_diff) > 10.f) continue;
					}

					possible_coins.push_back(Coin(ellipse, error, &plate_curve_pixels[i]));
					cv::ellipse(img_coin_ellipses, ellipse.center, ellipse.size*0.5f, ellipse.angle, 0, 360, cv::Scalar(0,255,0), 1, CV_AA);

					float a = ellipse.angle*d2r;
					float h = ellipse.size.width*0.5f;
					line(img_coin_ellipses, ellipse.center, cv::Point(ellipse.center.x+cos(a)*h, ellipse.center.y+sin(a)*h), cv::Scalar(0,0,255));
				}
			}
		}

		for (int i = 0; i < plate_contours.size(); i++)
		{
			if (plate_contours[i].size() > 4) // need 5 or more points to fit ellipse
			{
				cv::RotatedRect ellipse = cv::fitEllipse(plate_contours[i]);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, .06f*plate_dia, .15f*plate_dia, .8f*plate_AR, 1.25f*plate_AR, CP_ALL, img_plate_edges, 1.5f, &error);
				if (e_type > 0)
				{
					float e_AR = ellipse.size.height / ellipse.size.width;
					if (e_AR < 0.85f || e_AR > 1.15f) {
						float angle_diff = plate_angle - ellipse.angle;
						angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
						if (abs(angle_diff) > 10.f) continue;
					}

					possible_coins.push_back(Coin(ellipse, error, &plate_contours[i]));
					cv::ellipse(img_coin_ellipses, ellipse.center, ellipse.size*0.5f, ellipse.angle, 0, 360, cv::Scalar(0,255,0), 1, CV_AA);

					float a = ellipse.angle*d2r;
					float h = ellipse.size.width*0.5f;
					line(img_coin_ellipses, ellipse.center, cv::Point(ellipse.center.x+cos(a)*h, ellipse.center.y+sin(a)*h), cv::Scalar(0,0,255));
				}
			}
		}

		for (int i = 0; i < plate_contours2.size(); i++)
		{
			if (plate_contours2[i].size() > 4) // need 5 or more points to fit ellipse
			{
				cv::RotatedRect ellipse = cv::fitEllipse(plate_contours2[i]);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, .06f*plate_dia, .15f*plate_dia, .8f*plate_AR, 1.25f*plate_AR, CP_ALL, img_plate_edges, 1.5f, &error);
				if (e_type > 0)
				{
					float e_AR = ellipse.size.height / ellipse.size.width;
					if (e_AR < 0.85f || e_AR > 1.15f) {
						float angle_diff = plate_angle - ellipse.angle;
						angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
						if (abs(angle_diff) > 10.f) continue;
					}

					possible_coins.push_back(Coin(ellipse, error, &plate_contours2[i]));
					cv::ellipse(img_coin_ellipses, ellipse.center, ellipse.size*0.5f, ellipse.angle, 0, 360, cv::Scalar(0,255,0), 1, CV_AA);

					float a = ellipse.angle*d2r;
					float h = ellipse.size.width*0.5f;
					line(img_coin_ellipses, ellipse.center, cv::Point(ellipse.center.x+cos(a)*h, ellipse.center.y+sin(a)*h), cv::Scalar(0,0,255));
				}
			}
		}

		cv::imshow("Step 6c: fit ellipses to coin curves", img_coin_ellipses);

		float min_dist = .02f*plate_dia;
		min_dist *= min_dist;
		float min_s_err = min_dist * 2; // ???? this is just a random guess...

		// consolidate coin ellipses
		vector<vector<Coin> > grouped_coins;
		for (int i = 0; i < possible_coins.size(); i++)
		{
			bool found_match = false;
			for (int j = 0; j < grouped_coins.size(); j++)
			{
				cv::Size2f s_err = possible_coins[i].e.size - grouped_coins[j][0].e.size;
				float s_sq_err = s_err.height * s_err.height + s_err.width * s_err.width;
				cv::Point2f d = possible_coins[i].e.center - grouped_coins[j][0].e.center;
				float d_sq = d.x * d.x + d.y*d.y;
				if (d_sq < min_dist && s_sq_err < min_s_err)
				{
					grouped_coins[j].push_back(possible_coins[i]);
					found_match = true;
					break;
				}
			}
			if (!found_match)
			{
				// add new group
				grouped_coins.push_back(vector<Coin>());
				grouped_coins.back().push_back(possible_coins[i]);
			}
		}

		vector<cv::RotatedRect> coins;
		for (int i = 0; i < grouped_coins.size(); i++)
		{
			if (grouped_coins.size() == 1)
			{
				coins.push_back(grouped_coins[i][0].e);
			}
			else // multiple coins
			{
				// iterate through, find lowest error, and concat point vecs
				float lowest_error = grouped_coins[i][0].err;
				int best_coin = 0;
				vector<cv::Point> all_points;
				all_points.insert(all_points.end(), grouped_coins[i][0].pts->begin(), grouped_coins[i][0].pts->end());
				for (int j = 1; j < grouped_coins[i].size(); j++)
				{
					all_points.insert(all_points.end(), grouped_coins[i][j].pts->begin(), grouped_coins[i][j].pts->end());
					if (grouped_coins[i][j].err < lowest_error)
					{
						lowest_error = grouped_coins[i][j].err;
						best_coin = j;
					}
				}

				cv::RotatedRect ellipse = cv::fitEllipse(all_points);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, .06f*plate_dia, .15f*plate_dia, .8f*plate_AR, 1.25f*plate_AR, CP_ALL, img_plate_edges, 1.5f, &error);
				if (e_type > 0 && error <= lowest_error)
				{
					float e_AR = ellipse.size.height / ellipse.size.width;
					bool valid = true;
					if (e_AR < 0.85f || e_AR > 1.15f) {
						float angle_diff = plate_angle - ellipse.angle;
						angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
						if (abs(angle_diff) > 10.f) valid = false;
					}
					if (valid)
					{
						// push back the conglomerate ellipse
						coins.push_back(ellipse);
						continue;
					}
				}

				//conglomerate ellipse was not as good as best coin
				coins.push_back(grouped_coins[i][best_coin].e);
			}
		}

		cv::Mat img_final_coins = img_plate.clone();
		for (int i = 0; i < coins.size(); i++)
		{
			cv::ellipse(img_final_coins, coins[i].center, coins[i].size*0.5f, coins[i].angle, 0, 360, cv::Scalar(0,255,0), 1, CV_AA);

			float a = coins[i].angle*d2r;
			float h = coins[i].size.width*0.5f;
			line(img_final_coins, coins[i].center, cv::Point(coins[i].center.x+cos(a)*h, coins[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
			char num[5];
			sprintf(num, "%d",i);
			cv::putText(img_final_coins, num, coins[i].center, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0));
		}
		cv::imshow("Step 6d: group coins", img_final_coins);


		// Step 7: extract individual coins and classify based on colour & size (& features...?)

		cv::Mat img_plate_hsv;
		cv::cvtColor(img_plate, img_plate_hsv, CV_BGR2HSV);

		// hist details
		int ch[] = {0, 1};
		int histSize[] = { 45, 64 };
		float hranges[] = { 0, 180 };
		float sranges[] = { 0, 256 };
		const float* ranges[] = { hranges, sranges };

		// read in from file
		cv::FileStorage hist_storage("histograms.yml", cv::FileStorage::READ);
		cv::MatND gold_hist, silver_hist;

		if (hist_storage.isOpened())
		{
			hist_storage["silver"] >> silver_hist;
			hist_storage["gold"] >> gold_hist;

			if (silver_hist.rows != histSize[0] || silver_hist.cols != histSize[1] || gold_hist.rows != histSize[0] || gold_hist.cols != histSize[1])
			{
				printf("Histogram from file has unexpected size!");
				cv::waitKey();
				return EXIT_FAILURE;
			}
		}

		hist_storage.release();

//#define CLASSIFY_TRAINING
#ifdef CLASSIFY_TRAINING

		for (int i = 0; i < coins.size(); i++)
		{
			cv::Mat coin_mask = cv::Mat::zeros(img_plate_hsv.size(), CV_8UC1);
			cv::ellipse(coin_mask, coins[i], cv::Scalar(255), -1);

			cv::Mat coin_preview;
			img_plate.copyTo(coin_preview, coin_mask);
			cv::imshow("coin", coin_preview);

			int k = cv::waitKey();
			if (k == 2424832) // left
			{
				// silver
				cv::calcHist(&img_plate_hsv, 1, ch, coin_mask, silver_hist, 2, histSize, ranges, true, true);
				printf("coin %d classified as silver\n", i);
			}
			else if (k == 2555904) // right
			{
				// gold
				cv::calcHist(&img_plate_hsv, 1, ch, coin_mask, gold_hist, 2, histSize, ranges, true, true);
				printf("coin %d classified as gold\n", i);
			}
			else
			{
				printf("coin %d not classified\n", i);
			}
		}
		
		// save hists to file and exit
		hist_storage.open("histograms.yml", cv::FileStorage::WRITE);

		hist_storage << "silver" << silver_hist;
		hist_storage << "gold" <<  gold_hist;

		hist_storage.release();

		cv::imshow("silver_hist", silver_hist);
		cv::imshow("gold_hist", gold_hist);

		cv::waitKey();

		return EXIT_SUCCESS;
		

#endif


		cv::normalize(gold_hist, gold_hist, 1.0, cv::NORM_L1);
		cv::normalize(silver_hist, silver_hist, 1.0, cv::NORM_L1);

		cv::Mat coin_colour_confidences(coins.size(), NUM_COIN_TYPES, CV_32F);

		for (int i = 0; i < coins.size(); i++)
		{
			cv::Mat coin_mask = cv::Mat::zeros(img_plate_hsv.size(), CV_8UC1);
			cv::ellipse(coin_mask, coins[i], cv::Scalar(255), -1);
			
			cv::MatND hist;
			cv::calcHist(&img_plate_hsv, 1, ch, coin_mask, hist, 2, histSize, ranges);
			cv::normalize(hist, hist, 1.0, cv::NORM_L1);

			// TODO: compare hists
			double gold_err = cv::compareHist(gold_hist, hist, CV_COMP_CHISQR);
			double silver_err = cv::compareHist(silver_hist, hist, CV_COMP_CHISQR);

			for (int j = 0; j < _1_DOLLAR; j++)
				coin_colour_confidences.at<float>(i, j) = silver_err;
			for (int j = _1_DOLLAR; j < NUM_COIN_TYPES; j++)
				coin_colour_confidences.at<float>(i, j) = gold_err;

			printf("coin %d:  gold_err=%f  silver_err=%f\n", i, gold_err, silver_err);
		}
		
		float plate_actual_dia = 212.f;

		cv::Mat cam_inv = cameraMatrix.inv();
		float plate_angle_rad = plate_angle * d2r;
		cv::Point2f plate_major_axis = cv::Point2f(cos(plate_angle_rad-PI/2), sin(plate_angle_rad-PI/2));
		cv::Point2f plate_minor_axis = cv::Point2f(cos(plate_angle_rad), sin(plate_angle_rad));

		// 1) get plate distance - use: focal_length(pix) / plate_major_axis(pix) = plate_distance(mm) / plate_actual_dia(mm) ==> plate_dist = f*dia/major_axis
		float f = cameraMatrix.at<double>(0,0);
		float plate_dist = f*plate_actual_dia/plate_dia; // this seems to be returning values a little less than reality, why? - can it be fixed by a simple constant offset / multiplier ... ?
		printf("Plate Distance 1 = %f mm\n", plate_dist);
		// try alternate method: get two rays corresponding to each side of the plate, get angle between them and halve it (or just angle between centre and one side?)
		cv::Point2f edge1 = plate_and_coins[0].center + plate_dia / 2.f * plate_major_axis;
		cv::Point2f edge2 = plate_and_coins[0].center - plate_dia / 2.f * plate_major_axis;
		cv::Mat test = img_orig.clone();
		cv::circle(test, plate_and_coins[0].center, 3, cv::Scalar(0,0,255));
		cv::circle(test, edge1, 3, cv::Scalar(0,255,255));
		cv::circle(test, edge2, 3, cv::Scalar(255,0,255));
		
		

		float angle = getAngularSeperation(edge1, edge2, cam_inv)/2.f;
		float plate_dist2 = plate_actual_dia*0.5f/tan(angle);
		printf("Plate Distance 2 = %f mm\n", plate_dist2);

		// 2) get plate angle relative to camera axis: theta = plate_minor_axis / 2 * pix2deg_factor;  plate_angle = 180 - theta - asin(sin(theta) * plate_dist / (actual_plate_dia * 0.5f))
		//float pix2deg_factor 
		edge1 = plate_and_coins[0].center + plate_min_dia / 2.f * plate_minor_axis;
		edge2 = plate_and_coins[0].center - plate_min_dia / 2.f * plate_minor_axis;
		cv::circle(test, edge1, 3, cv::Scalar(255,255,0));
		cv::circle(test, edge2, 3, cv::Scalar(255,0,0));
		float theta = getAngularSeperation(edge1, edge2, cam_inv)/2.f;
		float q = asin(sin(theta) * plate_dist2 / (plate_actual_dia * 0.5f));
		float plate_skew = theta + q;
		printf("Plate Skew = %f deg\n", plate_skew / d2r);

		const int num_coins = coins.size();
		cv::Mat coin_size_confidences(coins.size(), NUM_COIN_TYPES, CV_32F);
		cv::Point2f plate_centre = plate_and_coins[0].center;
		vector<cv::Mat> coin_locations;
		for (int i = 0; i < coins.size(); i++)
		{
			// 3) calculate coin distance: coin_dist = plate_dist * sin(plate_angle) / sin(180-plate_angle-minor_axis_deg); coin_dist = coin_dist / cos(major_axis_deg);
			cv::Point2f a = coins[i].center + cv::Point2f(roi.tl()) - plate_centre;
			float maj_comp = a.dot(plate_major_axis);
			float min_comp = a.dot(plate_minor_axis);
			cv::Point2f maj_axis_comp = maj_comp * plate_major_axis;
			cv::Point2f min_axis_comp = min_comp * plate_minor_axis;
			float maj_axis_angle = getAngularSeperation(plate_centre, plate_centre + maj_axis_comp, cam_inv) * ((maj_comp > 0) - (maj_comp < 0));
			float min_axis_angle = getAngularSeperation(plate_centre, plate_centre + min_axis_comp, cam_inv) * ((min_comp > 0) - (min_comp < 0));
			float coin_dist = plate_dist2 * sin(plate_skew) / sin(PI - plate_skew - min_axis_angle); // this mult factor seems to be slightly less than it should be!!
			coin_dist = coin_dist / cos(maj_axis_angle);

			cv::circle(test, plate_centre + maj_axis_comp + min_axis_comp, 2, cv::Scalar(0,255,0));
			
			// 4) get coin size: coin_dia = coin_dist(mm) * coin_major_dia(pix) / f(pix);
			float coin_dia = coin_dist * MAX(coins[i].size.width, coins[i].size.height) / f;

			for (int j = 0; j < NUM_COIN_TYPES; j++)
			{
				float dia_err = coin_dia - COIN_SIZES[j];
				coin_size_confidences.at<float>(i, j) = dia_err * dia_err;
			}

			cv::Point3d p = cv::Point3d(coins[i].center + cv::Point2f(roi.tl())); p.z = 1.0;
			cv::Mat dir = cv::Mat(p);
			dir = cam_inv * dir;
			dir *= coin_dist/cv::norm(dir);
			coin_locations.push_back(dir);

			printf("coin %d: min_a=%f maj_a=%f dist=%f dia=%f\n", i, min_axis_angle, maj_axis_angle, coin_dist, coin_dia);
		}


		cv::Mat confidence = coin_size_confidences.mul(coin_colour_confidences*0.2f);

		vector<int> coin_types;
		for (int i = 0; i < coins.size(); i++)
		{
			float min_err = 50.f; // change this to act as a cut off for bad matches
			float min_err2 = 50.f;
			int min_idx = -1;
			int min_idx2 = -1;
			for (int j = 0; j < NUM_COIN_TYPES; j++)
			{
				float c = confidence.at<float>(i,j);
				if (c < min_err2)
				{
					if (c < min_err)
					{
						min_err2 = min_err;
						min_idx2 = min_idx;
						min_err = c;
						min_idx = j;
					}
					else
					{
						min_err2 = c;
						min_idx2 = j;
					}
				}
			}

			coin_types.push_back(min_idx);
			if (min_idx >= 0 && min_idx2 >= 0)
				printf("coin %d is a %s (%f) or maybe a %s (%f)\n", i, COIN_NAMES[min_idx], min_err, COIN_NAMES[min_idx2], min_err2);
			else if (min_idx >= 0)
				printf("coin %d is a %s (%f)\n", i, COIN_NAMES[min_idx], min_err);
			else
				printf("coin %d is not recognised as a valid coin.\n", i);
		}	

		vector<cv::Point3d> obj_points; 
		obj_points.push_back(cv::Point3d(0,0,0));
		obj_points.push_back(cv::Point3d(50,0,0));
		obj_points.push_back(cv::Point3d(80,0,0));
		obj_points.push_back(cv::Point3d(0,50,0));
		obj_points.push_back(cv::Point3d(0,80,0));
		
		for (int i = 0; i < frame_points.size(); i++)
		{
			char buf[5];
			sprintf(buf, "%d", i);
			cv::putText(test, buf, frame_points[i], cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,255));
		}

		cv::Mat rvec, tvec;
		cv::Mat distCoeffs_none;
		cv::solvePnP(obj_points, frame_points, cameraMatrix, distCoeffs_none, rvec, tvec);

		vector<cv::Point3d> virtual_frame_points; 
		virtual_frame_points.push_back(cv::Point3d(0,0,0));
		virtual_frame_points.push_back(cv::Point3d(80,0,0));
		virtual_frame_points.push_back(cv::Point3d(0,80,0));
		virtual_frame_points.push_back(cv::Point3d(0,0,80));

		vector<cv::Point2d> virtual_frame_image_points; 
		cv::projectPoints(virtual_frame_points, rvec, tvec, cameraMatrix, distCoeffs_none, virtual_frame_image_points);

		cv::line(test, virtual_frame_image_points[0], virtual_frame_image_points[1], cv::Scalar(0,0,255), 2);
		cv::line(test, virtual_frame_image_points[0], virtual_frame_image_points[2], cv::Scalar(0,255,0), 2);
		cv::line(test, virtual_frame_image_points[0], virtual_frame_image_points[3], cv::Scalar(255,0,0), 2);

		cv::Mat rmat;
		cv::Rodrigues(rvec, rmat);

		// calc cam relative to frame
		cv::Mat cam_r = rmat.t();
		cv::Mat cam_t = -cam_r*tvec;

		// TODO: output camera orientations as well
		printf("Camera is at (%f,%f,%f)\n", cam_t.at<double>(0), cam_t.at<double>(1), cam_t.at<double>(2)); 

		vector<cv::Point3f> coin_locations_in_frame;
		for	(int i = 0; i < coins.size(); i++)
		{
			//if (coin_types[i] < 0) continue;

			cv::Mat p = cam_r*coin_locations[i] + cam_t;
			coin_locations_in_frame.push_back(cv::Point3f(p.at<double>(0),p.at<double>(1),p.at<double>(2)));

			printf("coin %d is at (%f,%f,%f)\n", i, p.at<double>(0),p.at<double>(1),p.at<double>(2));
		}

		vector<cv::Point2f> coin_locations_image;
		cv::projectPoints(coin_locations_in_frame, rvec, tvec, cameraMatrix, distCoeffs, coin_locations_image);

		for	(int i = 0; i < coin_locations_image.size(); i++)
		{
			cv::line(test, virtual_frame_image_points[0], coin_locations_image[i], cv::Scalar(255,0,255), 1);
		}

		imshow("tyest", test);

		// TODO: Step 8: detect notes using sift/surf, use position on plate to calculate distance (see coin equivilant) --- meh


		// Step 9: sum value
		float total_coin_value = 0.f;
		int type_count[NUM_COIN_TYPES] = {0};
		int coin_count = 0;
		for (int i = 0; i < coin_types.size(); i++)
		{
			if (coin_types[i] >= 0)
			{
				total_coin_value += COIN_VALUES[coin_types[i]];
				type_count[coin_types[i]]++;
				coin_count++;
				// TODO: also draw on image
			}
		}

		printf("\nTOTAL COIN VALUE: $%.2f", total_coin_value);
		if (coin_count) printf(" ( ");
		for (int i = NUM_COIN_TYPES-1; i >= 0; i--)
			if (type_count[i])
				printf("%dx%s ", type_count[i], COIN_NAMES[i]);
		if (coin_count) printf(")");
		printf("\n\n");


		
		// THINGS STILL TO DO:
		// - fix/improve camera calibration
		// - clean up code and add cmd line args
		// - if no plates found then search more rigourously
		// - tweak/train/calibrate as necessary
		// - matlab colour calib

		// Implementation notes:
		//		- Task 1 in matlab...
		//		- Task 2 run seperate camera calib function (invoked by keypress/input)
		//		- Tasks 3 & 4 & 5 -> output camera location, followed by coin locations & values & total value (on image) (invoked by keypress, can run continuously)
	}

	double total_time = (cv::getTickCount() - start_time)/cv::getTickFrequency();
	printf("TOTAL TIME ELAPSED: %f\n", total_time);

	int selected = 0;
	while(1)
	{
		int key = cv::waitKey();
		switch (key)
		{
		case 2490368: //up
			selected++;
			if (selected >= plate_curves.size()) selected = plate_curves.size()-1;
			drawCurves(img_plate.size(), plate_curves, selected);
			break;
		case 2621440: //down
			selected--;
			if (selected < 0) selected = 0;
			drawCurves(img_plate.size(), plate_curves, selected);
			break;
		case 27://esc
			return EXIT_SUCCESS;
		}
	}
}

void GetContours(cv::Mat &image, cv::Mat &edges, vector<vector<cv::Point> > &contours, GC_METHOD method, int param, int step_num)
{
	cv::Mat img_thresh, img_blur, img_grey;
	cv::Mat *out_img;

	int64 time0 = cv::getTickCount();

	if (method != GC_CANNY) cv::cvtColor(image, img_grey, CV_BGR2GRAY);

	switch (method)
	{
	case GC_CANNY:
		// gaussian ... ?
		blur(image, img_blur, cv::Size(3,3));
		cv::Canny(img_blur, edges, param/2, param, 3);
		out_img = &edges;
		break;

	case GC_CANNY_GREY:
		blur(img_grey, img_blur, cv::Size(3,3));
		cv::Canny(img_blur, edges, param/2, param, 3);
		out_img = &edges;
		break;

	case GC_THRESH:
		cv::threshold(img_grey, img_thresh, param, 255, cv::THRESH_BINARY);
		out_img = &img_thresh;
		break;

	case GC_ADAPT_THRESH:
		cv::adaptiveThreshold(img_grey, img_thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 0);
		out_img = &img_thresh;
		break;
	}

	double time = (cv::getTickCount() - time0)/cv::getTickFrequency();
	printf("Step %da: %f\n", step_num, time);
	char buf[256];
	sprintf(buf, "Step %da: prep image for findContours", step_num);
	cv::imshow(buf, *out_img);

	time0 = cv::getTickCount();

	// get contours
	findContours(*out_img, contours, CV_RETR_LIST , CV_CHAIN_APPROX_NONE);

	time = (cv::getTickCount() - time0)/cv::getTickFrequency();
	printf("Step %db: %f\n", step_num, time);

	cv::Mat img_cont = cv::Mat::zeros(image.size(), CV_8UC3);

	// Draw contours
	for(int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > MIN_CONTOUR_SIZE)
		{
			drawContours(img_cont, contours, i, colors[i%256], 1, 8);
		}
	}

	sprintf(buf, "Step %db: Find Contours", step_num);
	cv::imshow(buf, img_cont);
}

struct Plate
{
	Plate(float er, cv::RotatedRect el) { err = er; ellipse = el; }
	float err;
	cv::RotatedRect ellipse;
};

bool plate_sort (Plate a, Plate b) { return (a.err<b.err); }

void FindPlate(cv::Mat &image, vector<vector<cv::Point> > &contours, vector<cv::RotatedRect> &output, vector<cv::Point2d> &frame_points, bool simple, CHECKPOINTS_TYPE cp_type, float cp_err_thresh, cv::Mat &edges, int canny_thresh, int step_num)
{
	vector<cv::RotatedRect> ellipses;
	vector<Plate> possible_plates;
	vector<vector<cv::RotatedRect> > plate_coins;
	char buf[256];

	double time0 = cv::getTickCount();

	if (cp_type > 0 && edges.size() != image.size()) // if we need an edge map, but don't have one then generate it
	{
		cv::Canny(image, edges, canny_thresh/2.f, canny_thresh);
	}

	if (simple)
	{
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() > MIN_CONTOUR_SIZE)
			{
				cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, 4.f, image.size().width, 0.5f, 2.f, cp_type, edges, cp_err_thresh, &error);
				if (e_type > 0)
				{
					ellipses.push_back(ellipse);
					if (e_type == EL_POSSIBLE_PLATE)
					{
						possible_plates.push_back(Plate(error, ellipse));
						plate_coins.push_back(vector<cv::RotatedRect>());
					}
				}
			}
		}
	}
	else
	{
		
		vector<vector<cv::Point> > curves;
		vector<vector<cv::Point> > curve_pixels;
		FindCurves(contours, curves, curve_pixels, image.size(), 2.f, step_num);

		for (int i = 0; i < curves.size(); i++)
		{
			if (curves[i].size() > 4) // need 5 or more points to fit ellipse
			{
				cv::RotatedRect ellipse = cv::fitEllipse(curves[i]);
				float error = 0.f;
				ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, 4.f, image.size().height, 0.5f, 2.f, cp_type, edges, cp_err_thresh, &error);
				if (e_type > 0)
				{
					ellipses.push_back(ellipse);
					if (e_type == EL_POSSIBLE_PLATE)
					{
						possible_plates.push_back(Plate(error, ellipse));
						plate_coins.push_back(vector<cv::RotatedRect>());
					}
				}
			}
		}
	}

	double time = (cv::getTickCount() - time0)/cv::getTickFrequency();
	printf("Step %d: %f\n", step_num, time);


	// show all ellipses
	cv::Mat img_ellipses = cv::Mat::zeros(image_size, CV_8UC3);
	double d2r = PI / 180.0;
	for (int i = 0; i < possible_plates.size(); i++)
	{
        ellipse(img_ellipses, possible_plates[i].ellipse.center, possible_plates[i].ellipse.size*0.5f, possible_plates[i].ellipse.angle, 0, 360, cv::Scalar(255,0,0), 3, CV_AA);
	}
	for (int i = 0; i < ellipses.size(); i++)
	{
        ellipse(img_ellipses, ellipses[i].center, ellipses[i].size*0.5f, ellipses[i].angle, 0, 360, cv::Scalar(0,255,255), 1, CV_AA);
        cv::Point2f vtx[4];
        ellipses[i].points(vtx);
        for( int j = 0; j < 4; j++ )
            line(img_ellipses, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
		float a = ellipses[i].angle*d2r;
		float h = ellipses[i].size.width*0.5f;
		line(img_ellipses, ellipses[i].center, cv::Point(ellipses[i].center.x+cos(a)*h, ellipses[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
	}

	sprintf(buf, simple ? "Step %d: fit ellipses to contours" : "Step %dc: fit ellipses to curves", step_num);
	imshow(buf, img_ellipses);

	// now that we have a list of all ellipses and possible plates figure out which plate(s) are valid
	if (possible_plates.size() == 0) 
	{
		printf("No possible plates found!\n");
		return;
	}

	time0 = cv::getTickCount();

	// sort plates by error
	sort(possible_plates.begin(), possible_plates.end(), plate_sort);

	for (int i = 0; i < ellipses.size(); i++)
	{
		for (int j = 0; j < possible_plates.size(); j++)
		{
			// TODO: optimize this so it is not being recalculated all the time
			float max_plate_radius = MAX(possible_plates[j].ellipse.size.height, possible_plates[j].ellipse.size.width)*0.5f; 
			float max_coin_radius = MAX(ellipses[i].size.height, ellipses[i].size.width)*0.5f;
			float plate_AR = (float)possible_plates[j].ellipse.size.height / (float)possible_plates[j].ellipse.size.width;
			float ellipse_AR = (float)ellipses[i].size.height / (float)ellipses[i].size.width;

			if (max_coin_radius > max_plate_radius*0.2f) continue; // make sure coin is signifigantly smaller than plate
			if (ellipse_AR < plate_AR*0.75f || ellipse_AR > plate_AR*1.25f) continue; // make sure AR is similar

			// check rotation (ignore if AR ~1.0)
			if (plate_AR < 0.85f || plate_AR > 1.15f)
			{
				float angle_diff = possible_plates[j].ellipse.angle - ellipses[i].angle;
				angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
				if (abs(angle_diff) > 15.f) continue;
			}

			// check if within other ellipse (roughly will do for the moment)
			cv::Point offset = ellipses[i].center - possible_plates[j].ellipse.center;
			float d_sq = offset.x*offset.x+offset.y*offset.y;
			if (d_sq > max_plate_radius*max_plate_radius) continue;

			plate_coins[j].push_back(ellipses[i]); // add this ellipse to this plate group
			break; // only add to first prospective plate
		}
	}

	int best_plate = 0;
	int max_coins = 0;
	for (int i = 0; i < plate_coins.size(); i++)
	{
		if (plate_coins[i].size() > max_coins)
		{
			best_plate = i;
			max_coins = plate_coins[i].size();
		}
	}

	output.push_back(possible_plates[best_plate].ellipse);

	for (int i = 0; i < plate_coins[best_plate].size(); i++)
	{
		output.push_back(plate_coins[best_plate][i]);
	}

	time = (cv::getTickCount() - time0)/cv::getTickFrequency();
	printf("Step %d: %f\n", ++step_num, time);

	// show results
	cv::Mat img_plateresults = image.clone();
	for (int i = 0; i < output.size(); i++)
	{
        ellipse(img_plateresults, output[i].center, output[i].size*0.5f, output[i].angle, 0, 360, cv::Scalar(0,255,255), 1, CV_AA);
        cv::Point2f vtx[4];
        output[i].points(vtx);
        for( int j = 0; j < 4; j++ )
            line(img_plateresults, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
	}


	// FIND FRAME
	// TODO: should also check angle
	for (int i = 0; i < ellipses.size(); i++)
	{
		for (int j = 0; j < ellipses.size(); j++)
		{
			if (i == j) continue;
			
			cv::Point2f d = ellipses[i].center - ellipses[j].center;
			if ((d.x*d.x + d.y*d.y) < 5.f)
			{
				cv::Size2f s = ellipses[i].size*2.f - ellipses[j].size;
				if ((s.height*s.height+s.width*s.width) < 10.f)
				{
					for (int k = 0; k < ellipses.size(); k++)
					{
						if (i == k || j == k) continue;
						
						d = ellipses[i].center - ellipses[j].center;
						if ((d.x*d.x + d.y*d.y) < 5.f)
						{
							s = ellipses[i].size*3.f - ellipses[k].size;
							if ((s.height*s.height+s.width*s.width) < 15.f)
							{
								// at this point we are pretty confident that this is the frame - there are three concentric ellipses with appropriate sizes
								frame_points[0] = ellipses[i].center;
								ellipse(img_plateresults, ellipses[i], cv::Scalar(0,0,255), 2);

								bool p1=false,p2=false,p3=false,p4=false;

								// find surronding ellipses (axis markers)
								for (int l = 0; l < ellipses.size(); l++)
								{
									d = ellipses[i].center - ellipses[l].center;
									cv::Size2f s1 = ellipses[i].size*0.75f - ellipses[l].size;
									cv::Size2f s2 = ellipses[i].size - ellipses[l].size;
									float dist = norm(d);
									if (dist > MIN(ellipses[i].size.width, ellipses[i].size.height)*2.2f && dist < MAX(ellipses[i].size.width, ellipses[i].size.height)*2.8f
										&& (s1.height*s1.height+s1.width*s1.width) < 40.f)
									{
										cv::Vec3b col = image.at<cv::Vec3b>(ellipses[l].center); // should prob avg multiple pixels...
										if (col[1] > col[2]) // green
										{
											frame_points[3] = ellipses[l].center;
											ellipse(img_plateresults, ellipses[l], cv::Scalar(255,0,0), 1);
											p3 = true;
										}
										else // red
										{
											frame_points[1] = ellipses[l].center;
											ellipse(img_plateresults, ellipses[l], cv::Scalar(255,0,0), 1);
											p1 = true;
										}
									}
									if (dist > MIN(ellipses[i].size.width, ellipses[i].size.height)*3.5f && dist < MAX(ellipses[i].size.width, ellipses[i].size.height)*4.5f
										&& (s2.height*s2.height+s2.width*s2.width) < 60.f)
									{
										cv::Vec3b col = image.at<cv::Vec3b>(ellipses[l].center); // should prob avg multiple pixels...
										if (col[1] > col[2]) // green
										{
											frame_points[4] = ellipses[l].center;
											ellipse(img_plateresults, ellipses[l], cv::Scalar(255,0,0), 1);
											p4 = true;
										}
										else // red
										{
											frame_points[2] = ellipses[l].center;
											ellipse(img_plateresults, ellipses[l], cv::Scalar(255,0,0), 1);
											p2 = true;
										}
									}

									if (p1 && p2 && p3 && p4) goto done;
								}
							}
						}
					}
				}
			}
		}
	}

	printf("Did not find full frame!\n");

done:

	sprintf(buf, "Step %d: sort ellipses into plates and choose best candidate", step_num);
	imshow(buf, img_plateresults);

}

void SegmentContours(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, float max_dev)
{
	for(int i = 0; i < contours.size(); i++)
	{
		seglist.push_back(vector<cv::Point>());
		seg_idxs.push_back(vector<int>());
		if (contours[i].size() > MIN_CONTOUR_SIZE)
		{
			int first = 0, last = contours[i].size()-1;
			seglist.back().push_back(cv::Point(contours[i][first]));
			seg_idxs.back().push_back(first);

			while (first < last)
			{
				int idx;
				float dev = maxLineDeviation(contours[i], first, last, &idx);

				while (dev > max_dev)
				{
					last = idx;
					dev = maxLineDeviation(contours[i], first, last, &idx);
				}

				seglist.back().push_back(cv::Point(contours[i][last]));
				seg_idxs.back().push_back(last);

				first = last;
				last = contours[i].size()-1;
			}
		}
	}
}

float maxLineDeviation(vector<cv::Point> &points, int first, int last, int *index)
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

	if (a == 0.f && b == 0.f) // same point at start and finish (i.e. loop)
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

void FindCurves(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels, cv::Size img_size, float max_dev, int step_num)
{
	vector<vector<cv::Point> > seglist;
	vector<vector<int> > seg_idxs;
	SegmentContours(contours, seglist, seg_idxs, max_dev);

	// show segments
	cv::Mat img_segments = cv::Mat::zeros(img_size*2, CV_8UC3);
	for(int i = 0; i < seglist.size(); i++)
	{
		if (seglist[i].size())
		{
			for (int j = 0; j < seglist[i].size()-1; j++)
				line(img_segments, seglist[i][j]*2, seglist[i][j+1]*2, colors[i%256], 1);
			for (int j = 0; j < seglist[i].size(); j++)
				circle(img_segments, seglist[i][j]*2, 2, cv::Scalar(0,0,255));
		}
	}

	char buf[256];
	sprintf(buf, "Step %da: split contours into segments", step_num);
	imshow(buf, img_segments);
		

	GetCurves(seglist, seg_idxs, contours, curves, curve_pixels);

	// show curves
	cv::Mat img_curves = cv::Mat::zeros(img_size*2, CV_8UC3);
	for(int i = 0; i < curves.size(); i++)
	{
		if (curves[i].size())
		{
			for (int j = 0; j < curves[i].size()-1; j++)
				line(img_curves, curves[i][j]*2, curves[i][j+1]*2, colors[i%256], 1);
			for (int j = 0; j < curves[i].size(); j++)
				circle(img_curves, curves[i][j]*2, 2, cv::Scalar(0,0,255));
		}
	}
	cv::Mat img_curve_pixels = cv::Mat::zeros(img_size, CV_8UC3);
	for(int i = 0; i < curve_pixels.size(); i++)
	{
		if (curve_pixels[i].size())
		{
			for (int j = 0; j < curve_pixels[i].size(); j++)
				img_curve_pixels.at<cv::Vec3b>(curve_pixels[i][j]) = cv::Vec3b(colors[i%256].val[0], colors[i%256].val[1], colors[i%256].val[2]);
		}
	}

	sprintf(buf, "Step %db: split segmented contours into curves", step_num);
	imshow(buf, img_curves);

	sprintf(buf, "Step %db2: split segmented contours into curves", step_num);
	imshow(buf, img_curve_pixels);

}


// TODO: improve curve splitting! & add curve joining....
void GetCurves(vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels)
{
	// seperate edge contours into individual curves
	for(int i = 0; i < seglist.size(); i++)
	{
		if (seg_idxs[i].size() == 0 || seg_idxs[i].back() < 5) // no point in process segments that are less than 5 pixels long as we need 5 points to match ellipse
			continue;

		curves.push_back(vector<cv::Point>());
		curve_pixels.push_back(vector<cv::Point>());

		int pix_idx = 0;
		int k = 0;
		for (int j = 0; j < seglist[i].size(); j++)
		{
			curves.back().push_back(cv::Point(seglist[i][j]));
			while (pix_idx <= seg_idxs[i][j]) curve_pixels.back().push_back(contours[i][pix_idx++]);
			bool split = false;
			// curvature condition
			if (k > 1 && j < (seglist[i].size()-1)) {
				cv::Point a = seglist[i][j-1] - seglist[i][j-2];
				cv::Point b = seglist[i][j] - seglist[i][j-2];
				cv::Point c = seglist[i][j+1] - seglist[i][j-2];
				
				float a1 = atan2f(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
				float a2 = atan2f(a.x*c.y - a.y*c.x, a.x*c.x + a.y*c.y);

				split = split || (a1*a2 < 0) || (abs(a2)-abs(a1) < 0);
			}

			// this just seems to cause false splits...
			if (!split && k > 0 && j < (seglist[i].size()-2)) {
				cv::Point a = seglist[i][j+1] - seglist[i][j+2];
				cv::Point b = seglist[i][j] - seglist[i][j+2];
				cv::Point c = seglist[i][j-1] - seglist[i][j+2];
				
				float a3 = atan2f(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
				float a4 = atan2f(a.x*c.y - a.y*c.x, a.x*c.x + a.y*c.y);

				split = split || (a3*a4 < 0) || (abs(a4)-abs(a3) < 0);

				//if (split) split_reasons.push_back(Scalar(255,255,0));
			}

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
				curves.push_back(vector<cv::Point>());
				curve_pixels.push_back(vector<cv::Point>());

				curves.back().push_back(cv::Point(seglist[i][j]));
				curve_pixels.back().push_back(contours[i][seg_idxs[i][j]]);

				k = 0;
			}
			k++;
		}
	}
}


ELLIPSE_TYPE ValidateEllipse(cv::RotatedRect e, float min_dia, float max_dia, float min_AR, float max_AR, CHECKPOINTS_TYPE checkPoints, cv::Mat &edges, float err_thresh, float *err)
{
	// check ellipse size/shape constraints
	float dia = MAX(e.size.width, e.size.height);
	if ( dia < min_dia || dia > max_dia ) return EL_INVALID; // check valid size

	float AR = e.size.height / e.size.width;
	if ( AR > max_AR || AR < min_AR ) return EL_INVALID; // check valid AR

	bool possible_plate = MAX(e.size.width, e.size.height) > MIN_PLATE_SIZE;

	if (checkPoints == CP_ALL || (checkPoints == CP_POSSIBLE_PLATES && possible_plate))
	{
		vector<cv::Point> e_points;
		ellipse2Poly(e.center, e.size*0.5f, e.angle, 0, 360, 10, e_points);

		float error = 0;
		for (int j = 0; j < e_points.size(); j++)
		{
			bool found = false;
			bool onscreen = false;
			for (int k = 0; k < 21; k++)
			{
				cv::Point p = e_points[j] + checkpoints[k];
				if (p.x >= 0 && p.y >= 0 && p.x < edges.cols && p.y < edges.rows)
				{
					onscreen = true;
					if (edges.at<uchar>(p))
					{
						found = true;
						error += point_weights[k];
						break;
					}
				}
			}
			if (!found & onscreen) error += 10.f;
		}

		error /= e_points.size();
		if (err) *err = error;

		if (error > err_thresh)
			return EL_INVALID;
	}

	return possible_plate ? EL_POSSIBLE_PLATE : EL_VALID;
}















void cameraCalibrationRoutine()
{
	cv::VideoCapture webcam(0);
	if (!webcam.isOpened()) return;

	cv::Mat cameraMatrix, distCoeffs;
	vector<vector<cv::Point2f> > imagePoints;	

	int numRequiredFrames = 20;

	int64 nextImage_tick = cv::getTickCount();
	int64 img_frequency = cv::getTickFrequency()*2;

	cv::Size boardSize(4,11);
	cv::Size image_size;
	float squareSize = 17.8f;

	while (1)
	{
		cv::Mat img;
		webcam >> img;
		
		cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

		image_size = img.size();

		//-----  If no more image, or got enough, then stop calibration and show result -------------
		if (imagePoints.size() >= numRequiredFrames)
		{
			vector<vector<cv::Point3f> > objectPoints(1);
			for( int i = 0; i < boardSize.height; i++ )
				for( int j = 0; j < boardSize.width; j++ )
					objectPoints[0].push_back(cv::Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
			objectPoints.resize(imagePoints.size(), objectPoints[0]);

			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			//cameraMatrix.at<double>(0,0) = 1.0;
			distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
			vector<cv::Mat> rvecs, tvecs;

			cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_PRINCIPAL_POINT);
			break;
		}

        vector<cv::Point2f> pointBuf;

		if (cv::getTickCount() >= nextImage_tick)
		{
			nextImage_tick += img_frequency;
			if (findCirclesGrid(img, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID))
			{
				drawChessboardCorners(img, boardSize, cv::Mat(pointBuf), false);

				//if (cv::getTickCount() >= nextImage_tick)
				{
					imagePoints.push_back(pointBuf);
					//nextImage_tick += img_frequency;
					cv::bitwise_not(img, img);
				}
			}
		}

		char buf[256];
		sprintf(buf, "Images remaing: %d", numRequiredFrames - imagePoints.size());
		cv::putText(img, buf, cv::Point( 20, 20), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
		imshow("Calibration", img);
		if (cv::waitKey(10) == 27) return;
	}

	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, image_size, CV_32FC1, map1, map2);

	cv::FileStorage camera_calib_file("camera_calibration.yml", cv::FileStorage::WRITE);
	camera_calib_file << "cameraMatrix" << cameraMatrix;
	camera_calib_file << "distCoeffs" << distCoeffs;
	camera_calib_file.release();

	while (1)
	{
		// display undistored image
		cv::Mat img, out_img;
		webcam >> img;
		//cv::remap(img, out_img, map1, map2, cv::INTER_CUBIC);
		cv::undistort(img, out_img, cameraMatrix, distCoeffs);
		imshow("Calibration", out_img);
		if (cv::waitKey(10) == 27) return;
	}

}