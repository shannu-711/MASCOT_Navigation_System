// Reference Marker Index = 0
// Target sapcecraft Index = 1
// Chaser spacecraft Index = 7


// Import the aruco module in OpenCV
#define _USE_MATH_DEFINES
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <string>       // for std::string
#include <sstream>
#include <fstream>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

std::vector<cv::Point2f> Reconstruction(const cv::Matx33f& cameraMatrix, const std::vector<cv::Vec3d>& tvecs, const std::vector<cv::Point2f>& centers)
{
	std::vector<cv::Point2f> worldCoordinates;
	int Z;

	// Convert camera matrix to double precision for inverse calculation
	cv::Mat inverseCameraMatrix;
	cv::invert(cv::Mat(cameraMatrix), inverseCameraMatrix, cv::DECOMP_SVD);
	cv::Mat inverseCameraMatrixDouble;
	inverseCameraMatrix.convertTo(inverseCameraMatrixDouble, CV_64F);

	for (size_t i = 0; i < centers.size(); i++)
	{
		Z = tvecs.at(i)[2];
		// Convert image point to homogeneous coordinates
		cv::Mat imagePointHomogeneous = (cv::Mat_<double>(3, 1) << centers[i].x, centers[i].y, 1.0);

		// Transform image point to camera coordinates
		cv::Mat cameraCoordinates = Z * inverseCameraMatrixDouble * imagePointHomogeneous;

		cv::Mat extrinsicMatrix = (cv::Mat_<double>(3, 3) <<
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1);	// CameraFrame to Inertial Frame

		// Invert extrinsic matrix
		cv::Mat inverseExtrinsicMatrix;
		cv::invert(extrinsicMatrix, inverseExtrinsicMatrix, cv::DECOMP_SVD);

		// Calculate world coordinates
		cv::Mat worldPoint = inverseExtrinsicMatrix * cameraCoordinates;

		// Extract X and Y coordinates and store as Point2f
		cv::Point2f worldCoordinate;
		worldCoordinate.x = worldPoint.at<double>(0, 0) * 100;		// *100 is to convert m to cm
		worldCoordinate.y = worldPoint.at<double>(1, 0) * 100;

		worldCoordinates.push_back(worldCoordinate);
	}
	return worldCoordinates;
}


//	Calculating center co-ordinates from corner co-ordinates
std::vector<cv::Point2f> corners2centers(const std::vector<std::vector<cv::Point2f>>& corners)
{
	std::vector<cv::Point2f> centers(corners.size());
	for (std::size_t i = 0; i < corners.size(); i++)
	{
		centers.at(i) = (corners.at(i).at(0) + corners.at(i).at(2)) / 2;
	}
	return centers;
}

std::vector<float> zAngle(const std::vector<cv::Vec3d>& rvecs)
{
	std::vector<float> zAngle(rvecs.size());
	std::vector<cv::Mat> rMats(rvecs.size());
	for (std::size_t i = 0; i < rvecs.size(); i++)
	{
		cv::Rodrigues(rvecs.at(i), rMats.at(i));	// converting rvec to rotation matrix
		//	Calculating yaw angle from rotation matrix by 3-2-1  Euler angle sequence
		zAngle.at(i) = -atan2(rMats.at(i).at<double>(0, 1), rMats.at(i).at<double>(0, 0)) * 180 / M_PI;
	}
	return zAngle;
}

std::vector<int> IndicesofId(const std::vector<int>& ids1, const std::vector<int>& ids2, int ID)
{
	int IdIndex1 = -1, IdIndex2 = -1;	// Index of marker in the vector of ids

	for (std::size_t id = 0; id < ids1.size(); id++)
	{
		if (ids1.at(id) == ID)
		{
			IdIndex1 = id;
			break;
		}
	}
	for (int id = 0; id < ids2.size(); id++)
	{
		if (ids2.at(id) == ID)
		{
			IdIndex2 = id;
			break;
		}
	}

	std::vector<int> Indices = { IdIndex1, IdIndex2 };

	return Indices;
}

std::vector<cv::Point2f> InertialCoordinates(std::vector<cv::Point2f>& CamWorldCoordinates, cv::Point2f& RefCenter)
{
	std::vector<cv::Point2f> InertialCoordinates(0);
	for (std::size_t i = 0; i < CamWorldCoordinates.size(); i++)
	{
		InertialCoordinates.push_back(CamWorldCoordinates.at(i) - RefCenter);
	}
	return InertialCoordinates;
}

std::vector<cv::Point2f> FinalCoordinates(std::vector<int>& ids1, std::vector<int>& ids2, std::vector<cv::Point2f>& InertialCordinates1, std::vector<cv::Point2f>& InertialCordinates2)
{
	cv::Point2f temp(0);
	float a1 = .5, a2 = .5;	// weights of both cameras
	std::vector<cv::Point2f> finalCordinates(0);

	std::vector<int> TargetIndices = IndicesofId(ids1, ids2, 1);
	if (TargetIndices.at(0) == -1 && TargetIndices.at(1) == -1)	// if not detected by both cameras
	{
		std::cout << "ERROR!!!" << std::endl << "One spacecraft is out of arena!" << std::endl;
	}
	else if (TargetIndices.at(0) != -1 && TargetIndices.at(1) != -1)	// if detected by both cameras
	{
		temp = a1 * InertialCordinates1.at(TargetIndices.at(0)) + a2 * InertialCordinates2.at(TargetIndices.at(1));
		finalCordinates.push_back(temp);
	}
	//	if detected by only one camera
	else if (TargetIndices.at(0) != -1) {
		temp = InertialCordinates1.at(TargetIndices.at(0));
		finalCordinates.push_back(temp);
	}
	else {
		temp = InertialCordinates2.at(TargetIndices.at(1));
		finalCordinates.push_back(temp);
	}

	std::vector<int> ChaserIndices = IndicesofId(ids1, ids2, 7);
	if (ChaserIndices.at(0) == -1 && ChaserIndices.at(1) == -1)	// if not detected by both cameras
	{
		std::cout << "ERROR!!!" << std::endl << "One spacecraft is out of arena!" << std::endl;
	}
	else if (ChaserIndices.at(0) != -1 && ChaserIndices.at(1) != -1)	// if detected by both cameras
	{
		temp = a1 * InertialCordinates1.at(ChaserIndices.at(0)) + a2 * InertialCordinates2.at(ChaserIndices.at(1));
		finalCordinates.push_back(temp);
	}
	//	if detected by only one camera
	else if (ChaserIndices.at(0) != -1) {
		temp = InertialCordinates1.at(ChaserIndices.at(0));
		finalCordinates.push_back(temp);
	}
	else {
		temp = InertialCordinates2.at(ChaserIndices.at(1));
		finalCordinates.push_back(temp);
	}
	return finalCordinates;
}

std::vector<float> FinalZAngle(std::vector<int>& ids1, std::vector<int>& ids2, std::vector<float>& zAngle1, std::vector<float>& zAngle2, float RefAngle1, float RefAngle2)
{
	std::vector<float> InertialAngle1(ids1.size()), InertialAngle2(ids2.size());
	for (std::size_t i = 0; i < ids1.size(); i++)
	{
		InertialAngle1.at(i) = zAngle1.at(i) - RefAngle1;
	}
	for (std::size_t i = 0; i < ids2.size(); i++)
	{
		InertialAngle2.at(i) = zAngle2.at(i) - RefAngle2;
	}

	float temp = 0;
	float a1 = .5, a2 = .5;
	std::vector<float> zAngleVec(0);

	std::vector<int> TargetIndices = IndicesofId(ids1, ids2, 1);
	if (TargetIndices.at(0) == -1 && TargetIndices.at(1) == -1)	// if not detected by both cameras
	{
		std::cout << "ERROR!!!" << std::endl << "One spacecraft is out of arena!" << std::endl;
	}
	else if (TargetIndices.at(0) != -1 && TargetIndices.at(1) != -1)	// if detected by both cameras
	{
		temp = a1 * InertialAngle1.at(TargetIndices.at(0)) + a2 * InertialAngle2.at(TargetIndices.at(1));
		zAngleVec.push_back(temp);
	}
	//	if detected by only one camera
	else if (TargetIndices.at(0) != -1) {
		temp = InertialAngle1.at(TargetIndices.at(0));
		zAngleVec.push_back(temp);
	}
	else {
		temp = InertialAngle2.at(TargetIndices.at(1));
		zAngleVec.push_back(temp);
	}

	std::vector<int> ChaserIndices = IndicesofId(ids1, ids2, 7);
	if (ChaserIndices.at(0) == -1 && ChaserIndices.at(1) == -1)	// if not detected by both cameras
	{
		std::cout << "ERROR!!!" << std::endl << "One spacecraft is out of arena!" << std::endl;
	}
	else if (ChaserIndices.at(0) != -1 && ChaserIndices.at(1) != -1)	// if detected by both cameras
	{
		temp = a1 * InertialAngle1.at(ChaserIndices.at(0)) + a2 * InertialAngle2.at(ChaserIndices.at(1));
		zAngleVec.push_back(temp);
	}
	//	if detected by only one camera
	else if (ChaserIndices.at(0) != -1) {
		temp = InertialAngle1.at(ChaserIndices.at(0));
		zAngleVec.push_back(temp);
	}
	else {
		temp = InertialAngle2.at(ChaserIndices.at(1));
		zAngleVec.push_back(temp);
	}
	return zAngleVec;
}

float pixelToCm(float markerLengthCm, const std::vector<cv::Point2f>& corners) {
	// Calculate the Euclidean distance between two corners of the ArUco marker
	float pixelDistance = norm(corners[0] - corners[1]);

	// Calculate and return the conversion factor (pixels per cm)
	return pixelDistance / markerLengthCm;
}

void detectAndProcessMarkers(const Mat& image, std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners) {
	Mat imageCopy;
	image.copyTo(imageCopy);

	if (ids.size() > 0) {
		aruco::drawDetectedMarkers(imageCopy, corners, ids);

		for (size_t i = 0; i < ids.size(); i++) {
			Point2f srcPoints[4];
			Point2f dstPoints[4] = { Point2f(0, 0), Point2f(100, 0), Point2f(100, 100), Point2f(0, 100) };

			for (int j = 0; j < 4; ++j) {
				srcPoints[j] = corners[i][j];
			}

			Mat MarkerImage;
			Mat markerTransform = getPerspectiveTransform(srcPoints, dstPoints);
			warpPerspective(image, MarkerImage, markerTransform, Size(100, 100));

			int cellSize = MarkerImage.cols / 8; // Assuming DICT_6X6_50 markers

			for (int row = 0; row < 8; ++row) {
				for (int col = 0; col < 8; ++col) {
					Rect cellRect(col * cellSize, row * cellSize, cellSize, cellSize);
					Mat cell = MarkerImage(cellRect);

					// Check if the cell is white (or mostly white)
					Scalar meanColor = mean(cell);
					if (meanColor[0] > 128) {
						// Mark white cells with a red rectangle
						rectangle(MarkerImage, cellRect, Scalar(0, 0, 255), 2);
					}
				}
			}

			imshow("Processed Marker", MarkerImage);
		}
	}

	imshow("Detected Markers", imageCopy);
}

int main(int argc, char** argv) {
	int ReferenceID = 0, TargetID = 1, ChaserID = 7;
	bool initialisationFlag = true;

	cv::Point2f RefCenter1, RefCenter2; // Center of reference marker in both cameras
	float RefMarkerAngle1 = 0, RefMarkerAngle2 = 0;

	// Creating aruco marker
	// cv::Mat markerImage;
	// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	// cv::aruco::generateImageMarker(dictionary, 0, 300, markerImage, 1);
	// cv::imwrite("marker0.png", markerImage);

	// Capturing frames from camera and detecting aruco markers
	cv::VideoCapture inputVideo1, inputVideo2;
	inputVideo1.open(1);
	inputVideo2.open(4);

	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);

	// Camera matrix and distortion coefficients from camera calibration
	cv::Matx33f cameraMatrix1(417.01608, 0, 639.5, 0, 417.01608, 359.5, 0, 0, 1); // Right
	cv::Matx33f cameraMatrix2(417.01608, 0, 639.5, 0, 417.01608, 359.5, 0, 0, 1); // Left
	cv::Vec<float, 5> distCoeffs1(-0.0742538, 0.00960546, 0, 0, 0);
	cv::Vec<float, 5> distCoeffs2(0.0756637, -0.0163972, 0, 0, 0);

	float markerLength = 0.159;		// in m

	// Set coordinate system
	cv::Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
	objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

	// Open a file to write the output
	std::ofstream outputFile1("coordinates.txt");

	if (!outputFile1.is_open()) {
		std::cerr << "Error opening file for writing!" << std::endl;
		return -1;
	}

	std::ofstream outputFile2("angle.txt");

	if (!outputFile2.is_open()) {
		std::cerr << "Error opening file for writing!" << std::endl;
		return -1;
	}

	while (inputVideo1.grab() && inputVideo2.grab()) {
		cv::Mat image1, image2, imageCopy1, imageCopy2;

		inputVideo1.retrieve(image1);
		inputVideo2.retrieve(image2);

		image1.copyTo(imageCopy1);
		image2.copyTo(imageCopy2);

		std::vector<int> ids1, ids2; // ids of detected markers in both cameras

		std::vector<std::vector<cv::Point2f>> corners1, rejected1; // vector of vector of corners of detected valid aruco markers and rejected shapes
		std::vector<std::vector<cv::Point2f>> corners2, rejected2;

		detector.detectMarkers(image1, corners1, ids1, rejected1);
		detector.detectMarkers(image2, corners2, ids2, rejected2);

		std::vector<cv::Point2f> centers1(corners1.size()); // vector of centers of valid detected markers in pixels and in cms
		std::vector<cv::Point2f> centers2(corners2.size());

		if (ids1.size() > 0 || ids2.size() > 0) // if at least one marker detected
		{
			cv::aruco::drawDetectedMarkers(imageCopy1, corners1, ids1);
			cv::aruco::drawDetectedMarkers(imageCopy2, corners2, ids2);

			int nMarkers1 = corners1.size(); // number of markers
			int nMarkers2 = corners2.size();

			std::vector<cv::Vec3d> rvecs1(nMarkers1), tvecs1(nMarkers1); // rotation and translation vectors for each marker
			std::vector<cv::Vec3d> rvecs2(nMarkers2), tvecs2(nMarkers2);

			// Calculating pose for each marker and Drawing axis for each marker
			for (int i = 0; i < nMarkers1; i++) {
				solvePnP(objPoints, corners1.at(i), cameraMatrix1, distCoeffs1, rvecs1.at(i), tvecs1.at(i));
				cv::drawFrameAxes(imageCopy1, cameraMatrix1, distCoeffs1, rvecs1[i], tvecs1[i], markerLength * 1.5f);
			}
			for (int i = 0; i < nMarkers2; i++) {
				solvePnP(objPoints, corners2.at(i), cameraMatrix2, distCoeffs2, rvecs2.at(i), tvecs2.at(i));
				cv::drawFrameAxes(imageCopy2, cameraMatrix2, distCoeffs2, rvecs2[i], tvecs2[i], markerLength * 1.5f);
			}

			// calculating centers of each marker
			centers1 = corners2centers(corners1);
			centers2 = corners2centers(corners2);

			std::vector<cv::Point2f> CamWorldCoordinates1 = Reconstruction(cameraMatrix1, tvecs1, centers1);
			std::vector<cv::Point2f> CamWorldCoordinates2 = Reconstruction(cameraMatrix2, tvecs2, centers2);

			std::vector<float> zAngle1 = zAngle(rvecs1);
			std::vector<float> zAngle2 = zAngle(rvecs2);

			std::vector<int> ReferenceIndices = IndicesofId(ids1, ids2, ReferenceID);

			if (initialisationFlag)
			{
				std::cout << "Initializing..." << std::endl;
				if (ReferenceIndices.at(0) == -1 || ReferenceIndices.at(1) == -1)
				{
					cout << "ERROR!!!" << std::endl << "Reference Marker is not detected!!" << std::endl;
					break;
				}
				else
				{
					// setting reference center as the center of reference marker
					RefCenter1 = CamWorldCoordinates1.at(ReferenceIndices.at(0));
					RefCenter2 = CamWorldCoordinates2.at(ReferenceIndices.at(1));

					// setting reference marker angle
					RefMarkerAngle1 = zAngle1.at(ReferenceIndices.at(0));
					RefMarkerAngle2 = zAngle2.at(ReferenceIndices.at(1));

					std::cout << "Initialization Done" << std::endl;
				}
				initialisationFlag = false;  // setting flag to false after executing once
			}

			std::vector<cv::Point2f> InertialCoordinates1 = InertialCoordinates(CamWorldCoordinates1, RefCenter1);
			std::vector<cv::Point2f> InertialCoordinates2 = InertialCoordinates(CamWorldCoordinates2, RefCenter2);

			std::vector<cv::Point2f> finalCoordinates(0);
			std::vector<float> finalZAngle(0);
			finalCoordinates = FinalCoordinates(ids1, ids2, InertialCoordinates1, InertialCoordinates2);
			finalZAngle = FinalZAngle(ids1, ids2, zAngle1, zAngle2, RefMarkerAngle1, RefMarkerAngle2);

			std::vector<int> TargetIndices = IndicesofId(ids1, ids2, TargetID);
			std::vector<int> ChaserIndices = IndicesofId(ids1, ids2, ChaserID);

			/*cout << "Cam1" << endl;
			cout << norm(rvecs1.at(ReferenceIndices.at(0))) << rvecs1.at(ReferenceIndices.at(0))/ norm(rvecs1.at(ReferenceIndices.at(0))) << endl;
			cout << norm(rvecs1.at(TargetIndices.at(0))) << rvecs1.at(TargetIndices.at(0)) / norm(rvecs1.at(TargetIndices.at(0))) << endl;
			cout << norm(rvecs1.at(ChaserIndices.at(0))) << rvecs1.at(ChaserIndices.at(0)) / norm(rvecs1.at(ChaserIndices.at(0))) << endl;
			cout << "Cam2" << endl;
			cout << norm(rvecs2.at(ReferenceIndices.at(1))) << rvecs2.at(ReferenceIndices.at(1)) / norm(rvecs2.at(ReferenceIndices.at(1))) << endl;
			cout << norm(rvecs2.at(TargetIndices.at(1))) << rvecs2.at(TargetIndices.at(1)) / norm(rvecs2.at(TargetIndices.at(1))) << endl;
			cout << norm(rvecs2.at(ChaserIndices.at(1))) << rvecs2.at(ChaserIndices.at(1)) / norm(rvecs2.at(ChaserIndices.at(1))) << endl;*/

			// Write the final coordinates and yaw angles to the output file

			for (const auto& coord : finalCoordinates) {
				outputFile1 << coord.x << "\t" << coord.y << "\t" << std::endl;
			}
			for (const auto& angle : finalZAngle) {
				outputFile2 << angle << std::endl;
			}

			/*float px2cm = pixelToCm(10.5, corners1.at(TargetIndices.at(0)));
			cout << px2cm << endl;*/

			/*std::cout << "Coordinates w.r.t. inertial marker :" << std::endl;
			std::cout << finalCoordinates << std::endl;
			std::cout << "Yaw Angle w.r.t. inertial marker :[" << finalZAngle.at(0) << ',' << finalZAngle.at(1) << ']' << std::endl;*/
		}

		cv::Mat image3;
		image1.copyTo(image3);
		detectAndProcessMarkers(image3, ids1, corners1);

		cv::Mat image4;
		image2.copyTo(image4);
		detectAndProcessMarkers(image4, ids2, corners2);

		cv::imshow("out1", imageCopy1);
		cv::imshow("out2", imageCopy2);
		char key = (char)cv::waitKey(1);
		if (key == 27) break;
	}

	// Close the file
	outputFile1.close();
	outputFile2.close();

	return 0;
}