#include "pointcloud_extractor.h"
#include "Utilities.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <opencv2/core/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace cv;


PointcloudExtractor::PointcloudExtractor(k4a_calibration_t calibration, string dstPath) : DataExtractorBase(dstPath + "\\pointclouds\\")
{
	depthWidth = calibration.depth_camera_calibration.resolution_width;
	depthHeight = calibration.depth_camera_calibration.resolution_height;
	mTransformationHandle = k4a_transformation_create(&calibration);

	depthImage = nullptr;
	colorImage = nullptr;

	VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		depthWidth,
		depthHeight,
		depthWidth * 3 * (int)sizeof(int16_t),
		&pointcloudImage), "Create Point Cloud Image failed!");

	VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		depthWidth,
		depthHeight,
		depthWidth * 4 * (int)sizeof(uint8_t),
		&convColor), "Creation of mapped color image failed!");
}

void PointcloudExtractor::extractData(k4a_capture_t captureHandle)
{
	depthImage = k4a_capture_get_depth_image(captureHandle);
	colorImage = k4a_capture_get_color_image(captureHandle);

	k4a_result_t result = k4a_transformation_color_image_to_depth_camera(mTransformationHandle, depthImage, colorImage, convColor);
	if (result != K4A_RESULT_SUCCEEDED) {
		cerr << "Cant map color image to depth image" << endl;
	}

	VERIFY(k4a_transformation_depth_image_to_point_cloud(
		mTransformationHandle,
		depthImage,
		K4A_CALIBRATION_TYPE_DEPTH,
		pointcloudImage), "Transform depth image to point clouds failed!");

	int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(pointcloudImage);
	uint8_t* color_image_data = k4a_image_get_buffer(convColor);

	std::vector<color_point_t> points;
	for (int i = 0; i < depthWidth * depthHeight; i++)
	{
		color_point_t point;
		point.xyz[0] = point_cloud_image_data[3 * i + 0];
		point.xyz[1] = point_cloud_image_data[3 * i + 1];
		point.xyz[2] = point_cloud_image_data[3 * i + 2];
		if (point.xyz[2] == 0)
		{
			continue;
		}

		point.rgb[0] = color_image_data[4 * i + 0];
		point.rgb[1] = color_image_data[4 * i + 1];
		point.rgb[2] = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
		{
			continue;
		}

		points.push_back(point);
	}

	uint64_t timestamp = k4a_image_get_device_timestamp_usec(colorImage);
	write_point_cloud(points, _dstPath + to_string(timestamp) + ".ply");
}

void PointcloudExtractor::write_point_cloud(vector<color_point_t> points, string file_name)
{
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex" << " " << points.size() << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "property uchar red" << std::endl;
	ofs << "property uchar green" << std::endl;
	ofs << "property uchar blue" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (size_t i = 0; i < points.size(); ++i)
	{
		// image data is BGR
		ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
		ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
		ss << std::endl;
	}
	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
