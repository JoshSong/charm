#include "ImageProcessor.h"

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>

#include <pthread.h>

using namespace std;

#define MIN_CONTOUR_SIZE 20
#define LINE_MAX_DEVIATION 2.f
#define MIN_PLATE_SIZE 100
#define CANNY_THRESH 200.0
#define TABLE_HEIGHT 50.f 
#define PI 3.141592653589793
#define CONFIDENCE_THRESH 0.01f

//enum MODE { MODE_NORMAL, MODE_CALIBRATE, MODE_TRAINCLASSIFY };
enum DEBUG_MODE { DEBUG_NONE = 0, DEBUG_TOSCREEN, DEBUG_TOFILE };
enum GC_METHOD { GC_CANNY, GC_CANNY_GREY, GC_THRESH, GC_ADAPT_THRESH };
enum CHECKPOINTS_TYPE { CP_NONE_, CP_POSSIBLE_PLATES, CP_ALL };
enum ELLIPSE_TYPE {EL_INVALID = 0, EL_VALID, EL_POSSIBLE_PLATE};
//enum COIN_TYPES { _5_CENT, _10_CENT, _20_CENT, _50_CENT, _1_DOLLAR, _2_DOLLAR, NUM_COIN_TYPES };
const char * const COIN_NAMES[NUM_COIN_TYPES] = { "5c", "10c", "20c", "50c", "$1", "$2" };
const float COIN_SIZES[NUM_COIN_TYPES] = { 19.41f, 23.6f, 28.65f, 32.f, 25.f, 20.5f };
const float COIN_VALUES[NUM_COIN_TYPES] = { 0.05f, 0.1f, 0.2f, 0.5f, 1.f, 2.f };

struct Plate {
	Plate(float er, cv::RotatedRect el) { err = er; ellipse = el; }
	float err;
	cv::RotatedRect ellipse;
};

struct Coin {
	cv::RotatedRect ellipse;
	cv::Point3d location;
	float diameter;
	float type_errs[NUM_COIN_TYPES];
	int count;
	int type;
	float confidence;
};

struct ErrorEllipse {
	ErrorEllipse(cv::RotatedRect _e, float _err) : e(_e), err(_err) {};
	cv::RotatedRect e;
	float err;
};

struct EllipseValidationParameters {
	float min_dia, max_dia, min_AR, max_AR, cp_err_thresh;
	CHECKPOINTS_TYPE cp_type;
	cv::Mat *edge_map;
};

struct HistogramInfo {
	int ch[2];
	int histSize[2];
	float hranges[2];
	float sranges[2];
	const float* ranges[2];
	cv::MatND gold_hist, silver_hist;
	cv::FileStorage hist_storage;
};

void cameraCalibrationRoutine(cv::VideoCapture &webcam);
void GetContours(cv::Mat &image, cv::Mat &edges, vector<vector<cv::Point> > &contours, GC_METHOD method, double param);
bool FindFrameAndCoins(cv::Mat &image, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &output_image, vector<vector<cv::Point> > &contours, vector<Coin> &output, float &table_rotation, EllipseValidationParameters evp);
void GetCameraPositionFromFrame(cv::Mat &cameraMatrix, cv::Mat &distCoeffs, vector<cv::Point2d> &frame_points, cv::Mat &outut_image, cv::Mat &cam_r, cv::Mat &cam_t);
bool GetCoinClassifyErrors(cv::Mat &img, vector<Coin> &coins, HistogramInfo &histInfo, float sizeVsColWeighting, bool training);
void SegmentContours(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, float max_dev);
float maxLineDeviation(vector<cv::Point> &points, int first, int last, int *index);
void FindCurves(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels, cv::Size img_size, float max_dev);
void GroupSegmentsIntoCurves(vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels);
double getAngularSeperation(cv::Point2d a, cv::Point2d b, cv::Mat cam_inv);
void FindCoins(cv::Mat &img_plate, vector<cv::RotatedRect> &plate_and_coins, vector<cv::RotatedRect> &coins);
void FitEllipses(vector<vector<cv::Point> > &pointsToFit, int minReqPoints, EllipseValidationParameters evp, vector<ErrorEllipse> &ellipses);
ELLIPSE_TYPE ValidateEllipse(cv::RotatedRect e, EllipseValidationParameters evp, float *err);

const double d2r = PI / 180.0;
int debug_mode = DEBUG_TOFILE;
cv::Scalar colors[256];
float point_weights[21];
const cv::Point checkpoints[21] = { 
							cv::Point( 0, 0), cv::Point( 1, 0), cv::Point(-1, 0), cv::Point( 0, 1),
							cv::Point( 0,-1), cv::Point( 1, 1), cv::Point( 1,-1), cv::Point(-1, 1),
							cv::Point(-1,-1), cv::Point( 2, 0), cv::Point(-2, 0), cv::Point( 0, 2),
							cv::Point( 0,-2), cv::Point( 1, 2), cv::Point( 1,-2), cv::Point(-1, 2),
							cv::Point(-1,-2), cv::Point( 2, 1), cv::Point( 2,-1), cv::Point(-2, 1),
							cv::Point(-2,-1) };




void DebugOutput(const char *title, cv::Mat img, bool include_step = true, bool reset_step = false)
{
	static int step = 0;
	static int frame = 0;
	if (debug_mode)
	{
		if (reset_step) step = 0;
		char buf[512];
		if (include_step)
			sprintf(buf, "%ds.%d - %s.png", ++frame, ++step, title);
		else
			sprintf(buf, "%s.png", title);
		if (debug_mode == DEBUG_TOSCREEN)
			cv::imshow(buf, img);
		else if (debug_mode == DEBUG_TOFILE)
			cv::imwrite(buf, img);
	}
}


bool setupHistogram(HistogramInfo &histInfo, const char *histoFile, int mode)
{
	histInfo.ch[0] = 0;
	histInfo.ch[1] = 1;
	histInfo.histSize[0] = 32;
	histInfo.histSize[1] = 32;
	histInfo.hranges[0] = 0;
	histInfo.hranges[1] = 256;
	histInfo.sranges[0] = 0;
	histInfo.sranges[1] = 256;
	histInfo.ranges[0] = histInfo.hranges;
	histInfo.ranges[1] = histInfo.sranges;

	// read in comparison histograms from file
	histInfo.hist_storage.open(histoFile, cv::FileStorage::READ);

	if (histInfo.hist_storage.isOpened())
	{
		histInfo.hist_storage["silver"] >> histInfo.silver_hist;
		histInfo.hist_storage["gold"] >> histInfo.gold_hist;

		if (histInfo.silver_hist.rows != histInfo.histSize[0] || histInfo.silver_hist.cols != histInfo.histSize[1] || histInfo.gold_hist.rows != histInfo.histSize[0] || histInfo.gold_hist.cols != histInfo.histSize[1])
		{
			printf("Histogram from file has unexpected size!");
			return false;
		}
	}
	else if (mode != ImageProcessor::MODE_TRAINCLASSIFY)
	{
		// if we can't open the file, and we're not going to train now, then we cannot compare colour to anything... so error and quite
		printf("Histogram file not found/accessible!");
		return false;
	}

	if (mode != ImageProcessor::MODE_TRAINCLASSIFY)
	{
		// if we are not train (i.e. appending to histograms) then we should normalize them for comparision
		cv::normalize(histInfo.gold_hist, histInfo.gold_hist, 1.0, cv::NORM_L1);
		cv::normalize(histInfo.silver_hist, histInfo.silver_hist, 1.0, cv::NORM_L1);
	}

	histInfo.hist_storage.release();
	
	return true;
}

bool readCamCalib(cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
	// read camera cailbration file
	bool read_success = false;
	cv::FileStorage camera_calib_file("camera_calibration.yml", cv::FileStorage::READ);
	if (camera_calib_file.isOpened())
	{
		camera_calib_file["cameraMatrix"] >> cameraMatrix;
		camera_calib_file["distCoeffs"] >> distCoeffs;
		read_success = true;
	}
	
	camera_calib_file.release();

	return read_success;
}

bool running = false;
pthread_mutex_t camMutex;
cv::Mat camImage;
LONGLONG imageTimestamp;

void *webcamCapture(void *cam)
{
	cv::VideoCapture *webcam = (cv::VideoCapture *)cam;

	while(running) { 
		pthread_mutex_lock(&camMutex);
		imageTimestamp = millisecondsNow(); 
		webcam->read(camImage); 
		pthread_mutex_unlock(&camMutex);

		cv::imshow("vid", camImage); 
		cv::waitKey(15); }

	return NULL;
}

LONGLONG ImageProcessor::run(int mode, int numFrames, std::vector<Object> &output_coins, float &tableSpeed)
{
	// set mode based on cmd line arg
	/*int mode = MODE_NORMAL;
	if (argc > 1)
	{
		if (strcmp(argv[1], "calib") == 0) mode = MODE_CALIBRATE;
		else if (strcmp(argv[1], "train") == 0) mode = MODE_TRAINCLASSIFY;
		else if (strcmp(argv[1], "debug") == 0) debug_mode = DEBUG_TOSCREEN;
	}*/

	///////////////////////////////////////////////////////////////////////////////////////
	// TODO: fix/move this!!!
	cv::RNG rng(12345);
	for (int i = 0; i < 256; i++)
		colors[i] = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));

	for (int i = 0; i < 21; i++)
		point_weights[i] = (float)(checkpoints[i].x * checkpoints[i].x + checkpoints[i].y * checkpoints[i].y);
	/////////////////////////////////////////////////////////////////////////////////////////

	cv::Mat cameraMatrix, distCoeffs;
	if (!readCamCalib(cameraMatrix, distCoeffs) || mode == MODE_CALIBRATE) { // if there is no camera calib file, or if we are explicitly in calib mode then do calib and quit.
		calibrate();
		return -1;
	}


	// try to open webcam
	cv::VideoCapture webcam(0);
	if (!webcam.isOpened()) {
		printf("Camera not detected or failed to initilize!");
		return -1;
	}
	
	// create undistort map to compensate for camera instrinsics/distortion
	cv::Mat map1, map2, temp_img;
	webcam.read(temp_img);
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, temp_img.size(), CV_32FC1, map1, map2);

	if (temp_img.size().area() == 0 || map1.size().area() == 0 || map2.size().area() == 0) { 
		printf("No webcam image recieved!\n");
		return -1; 
	}

	HistogramInfo histInfo;
	if (!setupHistogram(histInfo, "histograms.yml", mode)) { cv::waitKey(); return EXIT_FAILURE; }
	webcam.read(camImage);
	
	pthread_mutex_init(&camMutex, NULL);
	running = true;
	pthread_t cam_thread;
	pthread_create(&cam_thread, NULL, webcamCapture, (void*)&webcam);

	int default_debug_mode = debug_mode;

	vector<Coin> persistant_coins;
	int frame_count = 0;
	LONGLONG lastCamTime = -1;
	float lastTableAngle;
	float avgTableSpeed = 0;
	float table_rotation = 0;
	while (frame_count < numFrames)
	{
		int64 start_time = cv::getTickCount();

		// read next image
		cv::Mat img, img_edges, img_output;
		
		pthread_mutex_lock(&camMutex);
		LONGLONG camTime = imageTimestamp;
		img = camImage.clone(); // TODO: how to get exact image capture time?
		pthread_mutex_unlock(&camMutex);

		if (camTime <= 0 || img.empty()) continue;

		// apply remap to compensate for distortion
		cv::remap(img, img, map1, map2, cv::INTER_CUBIC);
		
		// setup an output image for visualization
		img_output = img.clone();
		
		DebugOutput("Get image and undistort", img, true, true);

		// get contours (multiple possible ways)
		vector<vector<cv::Point> > contours;
		GetContours(img, img_edges, contours, GC_CANNY, CANNY_THRESH);

		// find plate and plate
		if (img_edges.size() != img.size()) // if edges image was not created in GetContours then create it now becasue it will be need for the next step
			cv::Canny(img, img_edges, CANNY_THRESH/2.f, CANNY_THRESH);

		// setup args for FindFrameAndCoins, then call it to find the frame marker and all possible coin ellipses
		vector<Coin> possible_coins;
		EllipseValidationParameters evp;
		evp.min_dia = 20.f;
		evp.max_dia = (float)img.size().height;
		evp.min_AR = 0.5f;
		evp.max_AR = 2.0f;
		evp.cp_type = CP_ALL;
		evp.cp_err_thresh = 2.5f;
		evp.edge_map = &img_edges;
		if (FindFrameAndCoins(img, cameraMatrix, distCoeffs, img_output, contours, possible_coins, table_rotation, evp))
		{
			// classify coins by find size and colour erros
			if (!GetCoinClassifyErrors(img, possible_coins, histInfo, 3.f, mode == MODE_TRAINCLASSIFY))
				break; // if it returns false then we were in training mode and now training is complete, so exit

			if (mode == MODE_TRAINCLASSIFY) continue; // in train mode we don't do anything else.


			// integrate these possible coins with the persistant list
			int num_persistant_coins = persistant_coins.size();
			for (int i = 0; i < possible_coins.size(); i++)
			{
				bool match_found = false;
				for (int j = 0; j < num_persistant_coins; j++)
				{
					cv::Point3f dist = possible_coins[i].location - persistant_coins[j].location;
					if (dist.x*dist.x + dist.y*dist.y < 100.f) // if dist < 10mm then assume this is the same coin
					{
						persistant_coins[j].ellipse = possible_coins[i].ellipse;
						persistant_coins[j].location = (persistant_coins[j].location*persistant_coins[j].count + possible_coins[i].location) * (1.f / (persistant_coins[j].count+1));
						persistant_coins[j].diameter = (persistant_coins[j].diameter*persistant_coins[j].count + possible_coins[i].diameter) / (persistant_coins[j].count+1);
						persistant_coins[j].count++;
						
						float min_err = (persistant_coins[j].type_errs[0] += possible_coins[i].type_errs[0]);
						int type = 0;
						for (int k = 1; k < NUM_COIN_TYPES; k++) {
							persistant_coins[j].type_errs[k] += possible_coins[i].type_errs[k];
							if (persistant_coins[j].type_errs[k] < min_err) {
								min_err = persistant_coins[j].type_errs[k];
								type = k;
							}
						}
						persistant_coins[j].type = type;
						persistant_coins[j].confidence = 1.f / (min_err/(float)persistant_coins[j].count + 1.f);

						cv::Scalar col = (persistant_coins[j].confidence > CONFIDENCE_THRESH) ? cv::Scalar(0,255,0) : cv::Scalar(0,200,200);
						cv::ellipse(img_output, persistant_coins[j].ellipse, col);
						char buf[16]; sprintf(buf, "%d", j);
						cv::putText(img_output, buf, persistant_coins[j].ellipse.center, cv::FONT_HERSHEY_TRIPLEX, 0.5f, cv::Scalar(200,200,0));

						match_found = true;
						break;
					}
				}
				if (!match_found) { // add to list
					persistant_coins.push_back(possible_coins[i]);
					persistant_coins.back().count = 1;
					
					float min_err = persistant_coins.back().type_errs[0];
					int type = 0;
					for (int k = 1; k < NUM_COIN_TYPES; k++) {
						if (persistant_coins.back().type_errs[k] < min_err) {
							min_err = persistant_coins.back().type_errs[k];
							type = k;
						}
					}
					persistant_coins.back().type = type;
					persistant_coins.back().confidence = 1.f / (min_err + 1.f);
				}
			}

			// determine table velocity
			float table_speed = 0;
			if (frame_count > 1)
			{
				float dt = (float)((camTime - lastCamTime)/1000.f);
				float angle_diff = table_rotation - lastTableAngle;
				if (angle_diff < -0.1f) angle_diff += 2*PI; // TODO: hmmmmm...............
				table_speed = angle_diff/dt;
				avgTableSpeed = (avgTableSpeed*(frame_count-2) + table_speed) / (float)(frame_count-1);
				//printf("frame %d, dt %f, angle diff %f table rot %f \n", frame_count, dt, angle_diff, table_rotation);
			}
			//printf("frame %d, speed: %f, avg: %f\n", frame_count, table_speed, avgTableSpeed);
			lastCamTime = camTime;
			lastTableAngle = table_rotation;
			frame_count++;

			// create output window
			cv::Mat window(cv::Size(1200,480), CV_8UC3); 
			cv::rectangle(window, cv::Rect(640,0,1200-640,480), cv::Scalar(200,200,200), -1);
			img_output.copyTo(window(cv::Rect(0,0,640,480)));

			// sum value
			float total_coin_value = 0.f;
			char coin_type_text[256];
			int type_count[NUM_COIN_TYPES] = {0};
			int minCoinFrameFound = (numFrames / 2);
			if (minCoinFrameFound == 0) minCoinFrameFound++;
			for (int i = 0; i < persistant_coins.size(); i++)
			{
				float conf = persistant_coins[i].confidence;
				if (persistant_coins[i].count > minCoinFrameFound)
				{
					if (conf > CONFIDENCE_THRESH)
					{
						total_coin_value += COIN_VALUES[persistant_coins[i].type];
						type_count[persistant_coins[i].type]++;
					}
				}
				sprintf(coin_type_text, "Coin %d: %s (conf: %f, count: %d, dia: %.2f, loc:[%.1f, %.1f, %.1f])", i, COIN_NAMES[persistant_coins[i].type], conf, persistant_coins[i].count, persistant_coins[i].diameter, persistant_coins[i].location.x, persistant_coins[i].location.y, persistant_coins[i].location.z); 
				cv::putText(window, coin_type_text, cv::Point(680, 60+i*15), cv::FONT_HERSHEY_TRIPLEX, 0.4f, cv::Scalar(0,0,0));
			}

			char total_coin_text[512];
			sprintf(total_coin_text, "TOTAL COIN VALUE: $%.2f", total_coin_value);
			cv::putText(window, total_coin_text, cv::Point(680, 20), cv::FONT_HERSHEY_TRIPLEX, 0.6f, cv::Scalar(0,0,0));

			sprintf(total_coin_text, "( %dx%s %dx%s %dx%s %dx%s %dx%s %dx%s )", type_count[0], COIN_NAMES[0], 
																				type_count[1], COIN_NAMES[1], 
																				type_count[2], COIN_NAMES[2], 
																				type_count[3], COIN_NAMES[3], 
																				type_count[4], COIN_NAMES[4], 
																				type_count[5], COIN_NAMES[5] );
			cv::putText(window, total_coin_text, cv::Point(680, 38), cv::FONT_HERSHEY_TRIPLEX, 0.4f, cv::Scalar(0,0,0));

		
			double total_time = (cv::getTickCount() - start_time)/cv::getTickFrequency();
			char buf[256];
			sprintf(buf, "Processing time: %f", total_time);
			cv::putText(window, buf, cv::Point(680, 460), cv::FONT_HERSHEY_TRIPLEX, 0.4f, cv::Scalar(0,0,0));

			sprintf(buf, "Table Speed: %.4f (avg: %.4f)", table_speed, avgTableSpeed);
			cv::putText(window, buf, cv::Point(680, 430), cv::FONT_HERSHEY_TRIPLEX, 0.4f, cv::Scalar(0,0,0));

			DebugOutput("FINAL RESULT", window);

			cv::imshow("CoinCounter", window);

			if (frame_count == numFrames) cv::imwrite("coins_output.png", window);
		}

		if (debug_mode == DEBUG_TOFILE) debug_mode = default_debug_mode;
		int k = cv::waitKey(20);
		if (k == ' ') debug_mode = DEBUG_TOFILE;
		if (k == 27) break;
	}

	//TODO: move to class and have it return last time and coin position in last frame translated to absolute x & y

	running = false;
	pthread_join(cam_thread, NULL);
	webcam.release();

	int minCoinFrameFound = (numFrames / 2);
	if (minCoinFrameFound == 0) minCoinFrameFound++;
	float theta = table_rotation;
	for (int i = 0; i < persistant_coins.size(); i++)
	{
		if (persistant_coins[i].count >= minCoinFrameFound)
		{
			Object out;
			float x =  persistant_coins[i].location.x;
			float y =  persistant_coins[i].location.y;
			out.x = cos(theta) * x - sin(theta) * y;
			out.y = sin(theta) * x + cos(theta) * y;
			out.diameter = persistant_coins[i].diameter;	

			float conf = persistant_coins[i].confidence;
			if (conf < CONFIDENCE_THRESH)
				out.type = -1;
			else
				out.type = persistant_coins[i].type;

			output_coins.push_back(out);
		}
	}

	tableSpeed = avgTableSpeed;
	return lastCamTime;
}


bool GetCoinClassifyErrors(cv::Mat &img, vector<Coin> &coins, HistogramInfo &histInfo, float sizeVsColWeighting, bool training)
{
	// Now that we have a solid list of coins we can try to classify them, first using color

	if (coins.size() == 0) return true; // no coins, can't do anything...

	// convert image to hsv in preperation for histograms
	cv::Mat img_YCrCb;
	cv::cvtColor(img, img_YCrCb, CV_BGR2YCrCb);

	if (training)
	{
		bool exiting = false;
		for (unsigned int i = 0; i < coins.size(); i++)
		{
			cv::Mat coin_preview, plate_preview, coin_mask = cv::Mat::zeros(img_YCrCb.size(), CV_8UC1);
			cv::RotatedRect e_mask = coins[i].ellipse;
			e_mask.size = e_mask.size * 0.8f;
			cv::ellipse(coin_mask, e_mask, cv::Scalar(255), -1);
			img.copyTo(coin_preview, coin_mask);
			img.copyTo(plate_preview);
			cv::ellipse(plate_preview, coins[i].ellipse, cv::Scalar(255,255,255));
			cv::imshow("plate", plate_preview);
			cv::imshow("coin", coin_preview);


			int k = cv::waitKey();

			switch (k)
			{
			case 2424832: // left (silver)
				cv::calcHist(&img_YCrCb, 1, histInfo.ch, coin_mask, histInfo.silver_hist, 2, histInfo.histSize, histInfo.ranges, true, true);
				break;

			case 2555904: // right (gold)
				cv::calcHist(&img_YCrCb, 1, histInfo.ch, coin_mask, histInfo.gold_hist, 2, histInfo.histSize, histInfo.ranges, true, true);
				break;

			case 27: // esc
				exiting = true;
				break;
			}
		}
		
		cv::imshow("coin", cv::Mat::zeros(img_YCrCb.size(), CV_8UC1));
		if (histInfo.silver_hist.size().width && histInfo.silver_hist.size().height) cv::imshow("silver_hist", histInfo.silver_hist);
		if (histInfo.gold_hist.size().width && histInfo.gold_hist.size().height) cv::imshow("gold_hist", histInfo.gold_hist);

		// save hists to file and exit
		histInfo.hist_storage.open("histograms.yml", cv::FileStorage::WRITE);
		histInfo.hist_storage << "silver" << histInfo.silver_hist;
		histInfo.hist_storage << "gold" <<  histInfo.gold_hist;
		histInfo.hist_storage.release();

		cv::waitKey(); // wait so we can see hists

		return !exiting; // skip everything else, in training mode all we do is train
	}

	// calculate coin classification confidence based on colour
	for (unsigned int i = 0; i < coins.size(); i++)
	{
		cv::Mat coin_mask = cv::Mat::zeros(img_YCrCb.size(), CV_8UC1);
		cv::RotatedRect e_mask = coins[i].ellipse;
		e_mask.size = e_mask.size * 0.8f;
		cv::ellipse(coin_mask, e_mask, cv::Scalar(255), -1);
			
		cv::MatND hist;
		cv::calcHist(&img_YCrCb, 1, histInfo.ch, coin_mask, hist, 2, histInfo.histSize, histInfo.ranges);
		cv::normalize(hist, hist, 1.0, cv::NORM_L1);

		// compare to gold/silver hists
		float gold_err = (float)cv::compareHist(histInfo.gold_hist, hist, CV_COMP_CHISQR);
		float silver_err = (float)cv::compareHist(histInfo.silver_hist, hist, CV_COMP_CHISQR);

		for (int j = 0; j < _1_DOLLAR; j++)
			coins[i].type_errs[j] = silver_err;
		for (int j = _1_DOLLAR; j < NUM_COIN_TYPES; j++)
			coins[i].type_errs[j] = gold_err;

		//printf("coin %d:  gold_err=%f  silver_err=%f\n", i, gold_err, silver_err);
	}

	
	for (unsigned int i = 0; i < coins.size(); i++)
	{
		for (int j = 0; j < NUM_COIN_TYPES; j++) {
			float dia_err = coins[i].diameter - COIN_SIZES[j];
			coins[i].type_errs[j] = coins[i].type_errs[j] + sizeVsColWeighting * dia_err * dia_err;
		}
	}

	return true;
}

void GetContours(cv::Mat &image, cv::Mat &edges, vector<vector<cv::Point> > &contours, GC_METHOD method, double param)
{
	cv::Mat img_thresh, img_blur, img_grey;
	cv::Mat *out_img;

	if (method != GC_CANNY) cv::cvtColor(image, img_grey, CV_BGR2GRAY);

	switch (method)
	{
	case GC_CANNY:
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
	
	DebugOutput("Prep image for findContours", *out_img);

	// get contours
	cv::findContours(*out_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); 

	if (debug_mode)
	{
		// Draw contours
		cv::Mat img_cont = cv::Mat::zeros(image.size(), CV_8UC3);
		for(int i = 0; i < contours.size(); i++)
			if (contours[i].size() > MIN_CONTOUR_SIZE)
				drawContours(img_cont, contours, i, colors[i%256], 1, 8);

		DebugOutput("Find Contours", img_cont);
	}
}


bool FindFrameAndCoins(cv::Mat &image, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &output_image, vector<vector<cv::Point> > &contours, vector<Coin> &output, float &table_rotation, EllipseValidationParameters evp)
{
	vector<ErrorEllipse> possible_ellipses;
	
	// break the contours into individual curves
	vector<vector<cv::Point> > curves;
	vector<vector<cv::Point> > curve_pixels;
	FindCurves(contours, curves, curve_pixels, image.size(), LINE_MAX_DEVIATION);

	// try fitting both the original contours and the curves to ellipses
	FitEllipses(contours, MIN_CONTOUR_SIZE, evp, possible_ellipses);
	FitEllipses(curves, 5, evp, possible_ellipses); // TODO: use curve pixels as well? (or instead)

	if (debug_mode)
	{
		// show all ellipses
		cv::Mat img_ellipses = cv::Mat::zeros(image.size(), CV_8UC3);
		
		for (int i = 0; i < possible_ellipses.size(); i++)
		{
			ellipse(img_ellipses, possible_ellipses[i].e, cv::Scalar(0,255,255), 1, CV_AA);
			cv::Point2f vtx[4];
			possible_ellipses[i].e.points(vtx);
			for( int j = 0; j < 4; j++ )
				line(img_ellipses, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
			float a = possible_ellipses[i].e.angle*(float)d2r;
			float h = possible_ellipses[i].e.size.width*0.5f;
			line(img_ellipses, possible_ellipses[i].e.center, cv::Point2f(possible_ellipses[i].e.center.x+cos(a)*h, possible_ellipses[i].e.center.y+sin(a)*h), cv::Scalar(0,0,255));
		}

		DebugOutput("Fit ellipses to contours & curves", img_ellipses);
	}


	// consolidate duplicate ellipses
	float min_dist = 50.f;
	float min_s_err = 200.f; 
	vector<ErrorEllipse> good_ellipses;
	for (int i = 0; i < possible_ellipses.size(); i++)
	{
		bool found_match = false;
		for (int j = 0; j < good_ellipses.size(); j++)
		{
			cv::Size2f s_err = possible_ellipses[i].e.size - good_ellipses[j].e.size;
			float s_sq_err = s_err.height * s_err.height + s_err.width * s_err.width;
			cv::Point2f d = possible_ellipses[i].e.center - good_ellipses[j].e.center;
			float d_sq = d.x * d.x + d.y*d.y;
			if (d_sq < min_dist && s_sq_err < min_s_err) {
				if (possible_ellipses[i].err < good_ellipses[j].err) {
					good_ellipses[j] = possible_ellipses[i];
				}
				found_match = true;
				break;
			}
		}
		if (!found_match) {
			good_ellipses.push_back(possible_ellipses[i]);
		}
	}

	vector<cv::RotatedRect> ellipses;
	ellipses.reserve(good_ellipses.size());
	for (int i = 0; i < good_ellipses.size(); i++)
		ellipses.push_back(good_ellipses[i].e);


	if (debug_mode)
	{
		// show ellipses after filtering
		cv::Mat img_ellipses = cv::Mat::zeros(image.size(), CV_8UC3);
		
		for (int i = 0; i < ellipses.size(); i++)
		{
			ellipse(img_ellipses, ellipses[i], cv::Scalar(0,255,255), 1, CV_AA);
			cv::Point2f vtx[4];
			ellipses[i].points(vtx);
			for( int j = 0; j < 4; j++ )
				line(img_ellipses, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
			float a = ellipses[i].angle*(float)d2r;
			float h = ellipses[i].size.width*0.5f;
			line(img_ellipses, ellipses[i].center, cv::Point2f(ellipses[i].center.x+cos(a)*h, ellipses[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
		}

		DebugOutput("Filter out duplicate ellipses", img_ellipses);
	}

	// FIND FRAME
	bool frame_found = false;
	bool frame_centre_found = false;
	vector<cv::Point2d> frame_points(4);
	vector<cv::RotatedRect> frame_ellipses;
	vector<cv::RotatedRect> outer_frame_ellipses;
	vector<cv::RotatedRect> possible_stationary_markers;
	cv::RotatedRect frame_centre;
	for (int i = 0; i < ellipses.size(); i++)
	{
		float AR = ellipses[i].size.height / ellipses[i].size.width;
		bool check_angle = (AR < 0.8f || AR > 1.25f);
		for (int j = 0; j < ellipses.size(); j++)
		{
			if (i == j) continue;
			
			cv::Point2f d = ellipses[i].center - ellipses[j].center;
			float AR2 = ellipses[j].size.height / ellipses[j].size.width;
			float a = ellipses[i].angle - ellipses[j].angle; a += (a > 180) ? -360 : (a < -180) ? 360 : 0;
			if ((d.x*d.x + d.y*d.y) < 64.f && AR2 > AR*0.6f && AR2 < AR*1.5f && (!check_angle || abs(a) < 15.f))
			{
				// two concentric ellipses found
				float rw = ellipses[j].size.width / ellipses[i].size.width;
				float rh = ellipses[j].size.height / ellipses[i].size.height;
				float avg_r = (rw+rh)*0.5f;

				if (avg_r > 1.3f && avg_r < 1.7f) // if it is ~1.5x as big then this is most likely a outer frame marker
				{
					frame_ellipses.push_back(ellipses[i]);
					frame_ellipses.push_back(ellipses[j]);
					outer_frame_ellipses.push_back(ellipses[i]);// inner circle. Should we save both???
				}

				if (avg_r > 2.5f && avg_r < 3.5f) // if ~3x as big then could be the stationary marker
				{
					possible_stationary_markers.push_back(ellipses[i]);
				}

				if (avg_r > 1.7f && avg_r < 2.3f) // if it is twice as big then this is probably the centre marker
				{
					//cv::ellipse(img_ellipses, ellipses[i], cv::Scalar(255,0,0), 5);
					//cv::ellipse(img_ellipses, ellipses[j], cv::Scalar(255,0,0), 5);
					for (int k = 0; k < ellipses.size(); k++)
					{
						if (i == k || j == k) continue;
						
						d = ellipses[i].center - ellipses[k].center;
						cv::Size2f s = ellipses[i].size*3.f - ellipses[k].size;
						a = ellipses[i].angle - ellipses[k].angle; a += (a > 180) ? -360 : (a < -180) ? 360 : 0;
						float dist = d.x*d.x + d.y*d.y;
						float size_diff = s.height*s.height+s.width*s.width;
						if ((dist) < 64.f && (size_diff) < 640.f && (!check_angle || abs(a) < 20.f))
						{
							// at this point we are pretty confident that this is the frame - there are three concentric ellipses with appropriate sizes
							frame_centre_found = true;
							frame_points[0] = ellipses[i].center;
							frame_centre = ellipses[k];

							frame_ellipses.push_back(ellipses[i]);
							frame_ellipses.push_back(ellipses[j]);
							frame_ellipses.push_back(ellipses[k]);
						}
					}
				}
			}
		}
	}

	// find surronding ellipses (axis markers)
	if (frame_centre_found)
	{
		cv::Rect bounds = cv::Rect(cv::Point(), image.size());
		float min_r = MIN(frame_centre.size.height, frame_centre.size.width)/2.f;
		float maj_r = MAX(frame_centre.size.height, frame_centre.size.width)/2.f;
		float AR = frame_centre.size.height / frame_centre.size.width;
		bool check_angle = (AR < 0.8f || AR > 1.25f);
		bool p1=false,p2=false,p3=false;
		for (int l = 0; l < outer_frame_ellipses.size(); l++)
		{
			cv::Point2f d = outer_frame_ellipses[l].center - frame_centre.center;
			float dist = (float)norm(d);
			if (dist < min_r*7.f/3.f || dist > maj_r*10.f/3.f) continue; // rough bounds to filter

			//float AR2 = outer_frame_ellipses[l].size.height / outer_frame_ellipses[l].size.width;
			//if (AR2 < AR*0.8f || AR2 > AR*1.25f) continue;

			//cv::Size2f s = frame_centre.size*0.333333f - outer_frame_ellipses[l].size;
			//if ((s.height*s.height+s.width*s.width) > 120.f) continue; // this is not very good because circles are far apart and perspective changes size...

			float a = frame_centre.angle - outer_frame_ellipses[l].angle; a += (a > 180) ? -360 : (a < -180) ? 360 : 0;
			if (check_angle && abs(a) > 20.f) continue;
			
			if (bounds.contains(outer_frame_ellipses[l].center)) {
				
				cv::Vec3b col = image.at<cv::Vec3b>(outer_frame_ellipses[l].center); // TODO: should prob avg multiple pixels...
				
				// TODO: more thorough colour checking???
				if (col[2] > col[0] && col[2] > col[1]) { // red
					frame_points[1] = outer_frame_ellipses[l].center; p1 = true;
				} else if (col[1] > col[0] && col[1] > col[2]) { // green
					frame_points[2] = outer_frame_ellipses[l].center; p2 = true;
				} else if (col[0] > col[1] && col[0] > col[2]) { // blue
					frame_points[3] = outer_frame_ellipses[l].center; p3 = true;
				}
			}
		}
		frame_found = p1 && p2 && p3;
		if (!frame_found) printf("frame not found - r: %d, g: %d, b: %d\n", p1, p2, p3);
	}
	else
	{
		printf("frame centre not found!\n");
	}	

	

	if (debug_mode)
	{
		cv::Mat img_ellipses = image.clone();
		cv::circle(img_ellipses, frame_centre.center, 3, cv::Scalar(255,100,100));
		for (int i = 0; i < frame_ellipses.size(); i++)
		{
			ellipse(img_ellipses, frame_ellipses[i], cv::Scalar(255,255,100), 2);
			float a = frame_ellipses[i].angle*(float)d2r;
			float h = frame_ellipses[i].size.width*0.5f;
			line(img_ellipses, frame_ellipses[i].center, cv::Point2f(frame_ellipses[i].center.x+cos(a)*h, frame_ellipses[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
		}

		for (int i = 0; i < outer_frame_ellipses.size(); i++)
		{
			ellipse(img_ellipses, outer_frame_ellipses[i], cv::Scalar(0,100,255), 1);
			float a = outer_frame_ellipses[i].angle*(float)d2r;
			float h = outer_frame_ellipses[i].size.width*0.5f;
			line(img_ellipses, outer_frame_ellipses[i].center, cv::Point2f(outer_frame_ellipses[i].center.x+cos(a)*h, outer_frame_ellipses[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
		}

		for (int i = 0; i < possible_stationary_markers.size(); i++)
		{
			ellipse(img_ellipses, possible_stationary_markers[i], cv::Scalar(200,50,200), 5);
			float a = possible_stationary_markers[i].angle*(float)d2r;
			float h = possible_stationary_markers[i].size.width*0.5f;
			line(img_ellipses, possible_stationary_markers[i].center, cv::Point2f(possible_stationary_markers[i].center.x+cos(a)*h, possible_stationary_markers[i].center.y+sin(a)*h), cv::Scalar(0,0,255));
		}

		DebugOutput("Find frame ellipses", img_ellipses);
	}

	if (!frame_found) return false; // no frame found, we cannot do anything useful, just return so we can grab the next frame

	// get frame transform
	vector<cv::Point3d> obj_points; // create real world point array based on frame marker geometery
	obj_points.push_back(cv::Point3d(0,0,0));
	obj_points.push_back(cv::Point3d(85,0,0));
	obj_points.push_back(cv::Point3d(85*cos(4.f*PI/3.f),85*sin(4.f*PI/3.f),0));
	obj_points.push_back(cv::Point3d(85*cos(2.f*PI/3.f),85*sin(2.f*PI/3.f),0));

	// correlate that with the image locations for those points to find the frame position relative to camera
	cv::Mat rvec, tvec; // rotation and translation vecs
	cv::solvePnP(obj_points, frame_points, cameraMatrix, cv::Mat(), rvec, tvec);

	// draw virtual frame
	vector<cv::Point3d> virtual_frame_points; 
	virtual_frame_points.push_back(cv::Point3d(0,0,0));
	virtual_frame_points.push_back(cv::Point3d(80,0,0));
	virtual_frame_points.push_back(cv::Point3d(0,80,0));
	virtual_frame_points.push_back(cv::Point3d(0,0,80));
	vector<cv::Point2d> virtual_frame_image_points; 
	cv::projectPoints(virtual_frame_points, rvec, tvec, cameraMatrix, cv::Mat(), virtual_frame_image_points);

	cv::circle(output_image, frame_points[0], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[1], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[2], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[3], 3, cv::Scalar(255,255,255));

	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[1], cv::Scalar(0,0,255), 2);
	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[2], cv::Scalar(0,255,0), 2);
	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[3], cv::Scalar(255,0,0), 2);

	cv::Mat rmat;
	cv::Rodrigues(rvec, rmat);

	// calc cam relative to frame
	cv::Mat cam_r = rmat.t();
	cv::Mat cam_t = -cam_r*tvec;

	// also get camera matrix inverse
	cv::Mat cam_inv = cameraMatrix.inv(); // TODO: move as much stuff like this (that is static between frames) outside the loop to increase effecincy!!

	// create a mask for the turntable
	vector<cv::Point3d> circle_points;
	vector<cv::Point2d> transformed_circle_points;
	vector<cv::Point2f> transformed_circle_pointsf;
	// create a circle of points relative to table frame
	int num_circle_points = 16;
	circle_points.reserve(num_circle_points); 
	for (int i = 0; i < num_circle_points; i++) {
		double a = (double)i*2.f*PI/(double)16;
		circle_points.push_back(100.0*cv::Point3d(cos(a), sin(a), 0.0));
	}

	// project circle points to get the turntable ellipse in image plane
	cv::projectPoints(circle_points, rvec, tvec, cameraMatrix, cv::Mat(), transformed_circle_points);

	transformed_circle_pointsf.reserve(num_circle_points);
	for (int i = 0; i < num_circle_points; i++) {
		transformed_circle_pointsf.push_back(cv::Point2f(transformed_circle_points[i]));
	}

	// apply the ellipse mask
	cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
	cv::RotatedRect mask_ellipse = cv::fitEllipse(transformed_circle_pointsf);
	cv::ellipse(mask, mask_ellipse, cv::Scalar(255), -1);
	cv::Rect bounds = mask_ellipse.boundingRect();

	if (debug_mode)
	{
		cv::Mat img_plate;
		image.copyTo(img_plate, mask);
		DebugOutput("Mask plate", img_plate);
	}


	// find absolute table rotation (using stationary marker)
	table_rotation = -1.f;
	for(int i = 0; i < possible_stationary_markers.size(); i++)
	{
		cv::Point2f c = possible_stationary_markers[i].center;
		if (mask.at<unsigned char>(c) != 0) 	continue; // Inside plate, therefore ignore it

		cv::Point3d im_p = cv::Point3d(c); im_p.z = 1.0;
		cv::Mat dir = cv::Mat(im_p);
		dir = cam_inv * dir;
		dir = cam_r*dir;
		cv::Point3d u = cv::Point3d(dir);
		//u = u*(1.0/norm(u)); // necessary???
		//cv::Point3d n = cv::Point3d(0.0,0.0,1.0);
		//cv::Point3d p0 = cv::Point3d(cam_t) + cv::Point3d(0.0, 0.0, TABLE_HEIGHT);

		//double s = -n.dot(p0) / n.dot(u);
		double s = -(cam_t.at<double>(2)+TABLE_HEIGHT) / u.z;

		float dist = (float)s;

		// get marker size: dia = dist(mm) * major_dia(pix) / f(pix);
		float f = (float)cameraMatrix.at<double>(0,0);
		float dia = dist * MAX(possible_stationary_markers[i].size.width, possible_stationary_markers[i].size.height) / f;

		if (dia < 5.f || dia > 15.f) continue; // not the right size, ignore it 

		cv::Point3d pos = cv::Point3d(cam_t) + u*s;

		float stationary_marker_zero_angle = -(float)PI/4.f;
		float actual_angle = atan2(pos.y, pos.x);
		float a = stationary_marker_zero_angle - actual_angle;
		
		if (a < 0.f) table_rotation = a + 2*PI;
		else if (a >= 2*PI) table_rotation = a - 2*PI;
		else table_rotation = a;

		//cout << "stationary marker pos: " << pos << ", table angle: " << table_rotation << endl;

		cv::circle(output_image, c, 3, cv::Scalar(255, 200, 200));

		break; // there should only be one stationary marker 
	}


	// filter out invalid ellipses

	float AR = frame_centre.size.height / frame_centre.size.width;
	bool check_angle = (AR < 0.8f || AR > 1.25f);	

	float plate_angle_rad = frame_centre.angle * (float)d2r;
	cv::Point2f plate_major_axis = cv::Point2f(cos(plate_angle_rad-(float)PI/2.f), sin(plate_angle_rad-(float)PI/2.f));
	cv::Point2f plate_minor_axis = cv::Point2f(cos(plate_angle_rad), sin(plate_angle_rad));

	int invalid_pos_count = 0;

	// remove all ellipses outside plate radius or not aligned with plate and all frame ellipses
	for (int i = 0; i < ellipses.size(); i++)
	{
		cv::Point2f c = ellipses[i].center;
		if (!bounds.contains(c) || mask.at<unsigned char>(c) == 0) 	continue; // Outside plate, therefore ignore it

		//float AR2 = ellipses[i].size.height / ellipses[i].size.width;
		//if (AR2 < AR*0.8f || AR2 > AR*1.5f) continue; // AR differs from plate, ignore it

		float a = frame_centre.angle - ellipses[i].angle; a += (a > 180) ? -360 : (a < -180) ? 360 : 0;
		if (check_angle && abs(a) > 20.f) continue; // angle differs from plate, ignore it

		bool isFrame = false;
		for (int j = 0; j < frame_ellipses.size(); j++)
		{
			cv::Point2f d = frame_ellipses[j].center - c;
			if ((d.x*d.x + d.y*d.y) < min_dist) { isFrame = true; break; }
		}
		if (isFrame) continue; // ellipse is part of the frame marker, so ignore it


		cv::Point2f point_on_plate = ellipses[i].center - frame_centre.center;
		//float maj_comp = point_on_plate.dot(plate_major_axis);
		float min_comp = point_on_plate.dot(plate_minor_axis);
		//cv::Point2f maj_axis_comp = maj_comp * plate_major_axis;
		cv::Point2f min_axis_comp = min_comp * plate_minor_axis;
		//float maj_axis_angle = (float)getAngularSeperation(plate.center, plate.center + maj_axis_comp, cam_inv) * ((maj_comp > 0) - (maj_comp < 0));
		float min_axis_angle = (float)getAngularSeperation(frame_centre.center, frame_centre.center + min_axis_comp, cam_inv) * ((min_comp > 0) - (min_comp < 0));

		// get distance to coin
		float cam_x = cam_t.at<double>(0);
		float cam_y = cam_t.at<double>(1);
		float cam_z = cam_t.at<double>(2);
		float cam_xy = sqrt(cam_x*cam_x+cam_y*cam_y);
		float phi = atan(cam_xy/cam_z);
		float coin_dist = cam_z / cos(phi - min_axis_angle);
			
		// get coin size: coin_dia = coin_dist(mm) * coin_major_dia(pix) / f(pix);
		float f = (float)cameraMatrix.at<double>(0,0);
		float coin_dia = coin_dist * MAX(ellipses[i].size.width, ellipses[i].size.height) / f;

		if (coin_dia < 15.f || coin_dia > 40.f) continue; // outside range of normal coin sizes, ignore it

		// convert location from camera space to table space
		cv::Point3d p = cv::Point3d(ellipses[i].center); p.z = 1.0;
		cv::Mat dir = cv::Mat(p);
		dir = cam_inv * dir;
		dir *= (double)coin_dist/cv::norm(dir);
		cv::Mat pos = cam_r*dir + cam_t;
		cv::Point3d coin_pos = cv::Point3d(pos);

		// check coin_pos is valid
		if (coin_pos.x*coin_pos.x+coin_pos.y*coin_pos.y > 10000.f || abs(coin_pos.z) > 10.f) { invalid_pos_count++; continue; }

		Coin coin;
		coin.ellipse = ellipses[i];
		coin.diameter = coin_dia;
		coin.location = coin_pos;

		// passed all the checks, add to output list
		output.push_back(coin);
	}

	if (debug_mode)
	{
		// show results
		cv::Mat img_plateresults;
		image.copyTo(img_plateresults, mask);
		for (int i = 0; i < output.size(); i++)
		{
			ellipse(img_plateresults, output[i].ellipse, cv::Scalar(0,255,255));
			cv::Point2f vtx[4];
			output[i].ellipse.points(vtx);
			for( int j = 0; j < 4; j++ )
				line(img_plateresults, vtx[j], vtx[(j+1)%4], cv::Scalar(0,255,0), 1, CV_AA);
		}

		for (int i = 0; i < frame_points.size(); i++)
		{
			cv::circle(img_plateresults, frame_points[i], 3, cv::Scalar(0,255,255));
		}

		DebugOutput("Find frame and coin ellipses", img_plateresults);
	}

	// if there were more invalud positions than valid then its pretty likely that the frame was erroneous.
	if (invalid_pos_count > output.size()) frame_found = false;

	return frame_found;
}



/*void FitEllipses(vector<vector<cv::Point> > &pointsToFit, int minReqPoints, EllipseValidationParameters evp, vector<cv::RotatedRect> &ellipses, vector<Plate> &possible_plates)
{
	for (int i = 0; i < pointsToFit.size(); i++)
	{
		if (pointsToFit[i].size() >= minReqPoints)
		{
			cv::RotatedRect ellipse = cv::fitEllipse(pointsToFit[i]);
			float error = 0.f;
			ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, evp, &error);
			if (e_type > 0)
			{
				ellipses.push_back(ellipse);
				if (e_type == EL_POSSIBLE_PLATE)
				{
					possible_plates.push_back(Plate(error, ellipse));
				}
			}
		}
	}
}*/

void FitEllipses(vector<vector<cv::Point> > &pointsToFit, int minReqPoints, EllipseValidationParameters evp, vector<ErrorEllipse> &ellipses)
{
	for (int i = 0; i < pointsToFit.size(); i++)
	{
		if (pointsToFit[i].size() >= minReqPoints)
		{
			cv::RotatedRect ellipse = cv::fitEllipse(pointsToFit[i]);
			float error = 0.f;
			ELLIPSE_TYPE e_type = ValidateEllipse(ellipse, evp, &error);
			if (e_type > 0) {
				ellipses.push_back(ErrorEllipse(ellipse, error));
			}
		}
	}
}


/*void FitCoins(vector<vector<cv::Point> > &pointsToFit, int minReqPoints, EllipseValidationParameters evp, float angle, float angle_err, vector<Coin> &coins)
{
	for (int i = 0; i < pointsToFit.size(); i++)
	{
		if (pointsToFit[i].size() >= minReqPoints)
		{
			cv::RotatedRect ellipse = cv::fitEllipse(pointsToFit[i]);
			float error = 0.f;
			if (ValidateEllipse(ellipse, evp, &error))
			{
				float e_AR = ellipse.size.height / ellipse.size.width;
				if (e_AR < 0.8f || e_AR > 1.25f) {
					float angle_diff = angle - ellipse.angle;
					angle_diff += (angle_diff > 180) ? -360 : (angle_diff < -180) ? 360 : 0;
					if (abs(angle_diff) > angle_err) continue;
				}

				coins.push_back(Coin(ellipse, error, &pointsToFit[i]));
			}
		}
	}
}*/


ELLIPSE_TYPE ValidateEllipse(cv::RotatedRect e, EllipseValidationParameters evp, float *err)
{
	// check ellipse size/shape constraints
	float dia = MAX(e.size.width, e.size.height);
	if ( dia < evp.min_dia || dia > evp.max_dia ) return EL_INVALID; // check valid size

	float AR = e.size.height / e.size.width;
	if ( AR > evp.max_AR || AR < evp.min_AR ) return EL_INVALID; // check valid AR

	bool possible_plate = MAX(e.size.width, e.size.height) > MIN_PLATE_SIZE;

	if (evp.cp_type == CP_ALL || (evp.cp_type == CP_POSSIBLE_PLATES && possible_plate))
	{
		vector<cv::Point> e_points;
		ellipse2Poly(e.center, e.size*0.5f, (int)e.angle, 0, 360, 10, e_points);

		float error = 0; 
		int num_onscreen = 0;
		for (int j = 0; j < e_points.size(); j++)
		{
			bool found = false;
			bool onscreen = false;
			for (int k = 0; k < 21; k++) // TODO: is it necessary to use this many points... might be quicker if max k is reduced...
			{
				cv::Point p = e_points[j] + checkpoints[k];
				if (p.x >= 0 && p.y >= 0 && p.x < evp.edge_map->cols && p.y < evp.edge_map->rows)
				{
					onscreen = true;
					num_onscreen++;
					if (evp.edge_map->at<uchar>(p))
					{
						found = true;
						error += point_weights[k];
						break;
					}
				}
			}
			if (!found & onscreen) error += 10.f;
		}

		if (num_onscreen < e_points.size()/4) return EL_INVALID;

		error /= e_points.size();
		if (err) *err = error;

		if (error > evp.cp_err_thresh)
			return EL_INVALID;
	}

	return possible_plate ? EL_POSSIBLE_PLATE : EL_VALID;
}



void GetCameraPositionFromFrame(cv::Mat &cameraMatrix, cv::Mat &distCoeffs, vector<cv::Point2d> &frame_points, cv::Mat &output_image, cv::Mat &cam_r, cv::Mat &cam_t)
{
	// create real world point array based on frame marker geometery
	vector<cv::Point3d> obj_points; 
	obj_points.push_back(cv::Point3d(0,0,0));
	obj_points.push_back(cv::Point3d(85,0,0));
	obj_points.push_back(cv::Point3d(85*cos(4.f*PI/3.f),85*sin(4.f*PI/3.f),0));
	obj_points.push_back(cv::Point3d(85*cos(2.f*PI/3.f),85*sin(2.f*PI/3.f),0));

	// correlate that with the image locations for those points to find the frame position relative to camera
	cv::Mat rvec, tvec; // rotation and translation vecs
	cv::solvePnP(obj_points, frame_points, cameraMatrix, cv::Mat(), rvec, tvec);

	vector<cv::Point3d> virtual_frame_points; 
	virtual_frame_points.push_back(cv::Point3d(0,0,0));
	virtual_frame_points.push_back(cv::Point3d(80,0,0));
	virtual_frame_points.push_back(cv::Point3d(0,80,0));
	virtual_frame_points.push_back(cv::Point3d(0,0,80));

	vector<cv::Point2d> virtual_frame_image_points; 
	cv::projectPoints(virtual_frame_points, rvec, tvec, cameraMatrix, cv::Mat(), virtual_frame_image_points);

	cv::circle(output_image, frame_points[0], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[1], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[2], 3, cv::Scalar(255,255,255));
	cv::circle(output_image, frame_points[3], 3, cv::Scalar(255,255,255));

	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[1], cv::Scalar(0,0,255), 2);
	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[2], cv::Scalar(0,255,0), 2);
	cv::line(output_image, virtual_frame_image_points[0], virtual_frame_image_points[3], cv::Scalar(255,0,0), 2);

	cv::Mat rmat;
	cv::Rodrigues(rvec, rmat);

	// calc cam relative to frame
	cam_r = rmat.t();
	cam_t = -cam_r*tvec;
}


double getAngularSeperation(cv::Point2d a, cv::Point2d b, cv::Mat cam_inv)
{
	cv::Mat p1 = cv::Mat(cv::Matx31d(a.x, a.y, 1.f));
	cv::Mat p2 = cv::Mat(cv::Matx31d(b.x, b.y, 1.f));
	p1 = cam_inv * p1;
	p2 = cam_inv * p2;
	return acos(p1.dot(p2)/(cv::norm(p1)*cv::norm(p2)));
}


void FindCurves(vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels, cv::Size img_size, float max_dev)
{
	vector<vector<cv::Point> > seglist;
	vector<vector<int> > seg_idxs;
	SegmentContours(contours, seglist, seg_idxs, max_dev);

	if (debug_mode)
	{
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
		DebugOutput("Split contours into segments", img_segments);
	}	

	// group segments into sections of similar curvature
	GroupSegmentsIntoCurves(seglist, seg_idxs, contours, curves, curve_pixels);

	if (debug_mode)
	{
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

		DebugOutput("Split segmented contours into curves (segments)", img_curves);

		DebugOutput("Split segmented contours into curves (pixels)", img_curve_pixels);
	}
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
	float a = (float)(points[first].y - points[last].y);
	float b = (float)(points[last].x - points[first].x);
	float c = (float)(points[first].x * points[last].y - points[last].x * points[first].y);

	float length = sqrt(a*a + b*b);

	float d_max = 0.f;
	int idx = last;

	if (a == 0.f && b == 0.f) // same point at start and finish (i.e. loop)
	{
		for (int i = first; i <= last; i++)
		{
			float dx = (float)(points[i].x - points[first].x);
			float dy = (float)(points[i].y - points[first].y);
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


// TODO: improve curve splitting! & add curve joining....
void GroupSegmentsIntoCurves(vector<vector<cv::Point> > &seglist, vector<vector<int> > &seg_idxs, vector<vector<cv::Point> > &contours, vector<vector<cv::Point> > &curves, vector<vector<cv::Point> > &curve_pixels)
{
	// seperate edge contours into individual curves
	for(int i = 0; i < seglist.size(); i++)
	{
		if (seg_idxs[i].size() == 0 || seg_idxs[i].back() < 5) // no point in processing segments that are less than 5 pixels long as we need 5 points to match ellipse
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
				
				float a1 = atan2f((float)(a.x*b.y - a.y*b.x), (float)(a.x*b.x + a.y*b.y));
				float a2 = atan2f((float)(a.x*c.y - a.y*c.x), (float)(a.x*c.x + a.y*c.y));

				split = split || (a1*a2 < 0) || (abs(a2)-abs(a1) < 0);
			}

			// this just seems to cause false splits...
			if (!split && k > 0 && j < (seglist[i].size()-2)) {
				cv::Point a = seglist[i][j+1] - seglist[i][j+2];
				cv::Point b = seglist[i][j] - seglist[i][j+2];
				cv::Point c = seglist[i][j-1] - seglist[i][j+2];
				
				float a3 = atan2f((float)(a.x*b.y - a.y*b.x), (float)(a.x*b.x + a.y*b.y));
				float a4 = atan2f((float)(a.x*c.y - a.y*c.x), (float)(a.x*c.x + a.y*c.y));

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













void ImageProcessor::calibrate()
{
	// try to open webcam
	cv::VideoCapture webcam(0);
	if (!webcam.isOpened()) {
		printf("Camera not detected or failed to initilize!");
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	vector<vector<cv::Point2f> > imagePoints;	

	int numRequiredFrames = 20;

	int64 nextImage_tick = cv::getTickCount();
	int64 img_frequency = (int64)cv::getTickFrequency()*3;

	cv::Size boardSize(4,11);
	cv::Size image_size;
	float squareSize = 15.8f;

	vector<cv::Mat> rvecs, tvecs;

	while (1)
	{
		cv::Mat img;
		webcam >> img;
		
		cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

		image_size = img.size();

		if (imagePoints.size() >= numRequiredFrames)
		{
			vector<vector<cv::Point3f> > objectPoints(1);
			for( int i = 0; i < boardSize.height; i++ )
				for( int j = 0; j < boardSize.width; j++ )
					objectPoints[0].push_back(cv::Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
			objectPoints.resize(imagePoints.size(), objectPoints[0]);

			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

			cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_PRINCIPAL_POINT);
			break;
		}

        vector<cv::Point2f> pointBuf;

		
		if (findCirclesGrid(img, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID))
		{
			drawChessboardCorners(img, boardSize, cv::Mat(pointBuf), true);
			if (cv::getTickCount() >= nextImage_tick)
			{
				nextImage_tick += img_frequency;
				imagePoints.push_back(pointBuf);
				cv::bitwise_not(img, img);
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
	for (int i = 0; i < rvecs.size(); i++)
	{
		cv::Mat rmat;
		cv::Rodrigues(rvecs[i], rmat);
		cv::Mat mat(4, 4, CV_64F);
		rmat.copyTo(mat(cv::Rect(0,0,3,3)));
		tvecs[i].copyTo(mat(cv::Rect(3,0,1,3)));
		mat.at<double>(3,3) = 1.0;
		char buf[128];
		sprintf(buf, "Image %d Extrinsic Matrix", i);
		camera_calib_file << buf << mat;
	}
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