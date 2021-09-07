#include "image_extractor.h"

#include <iostream>
#include <filesystem>

#include <opencv2/core/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace cv;

ImageExtractor::ImageExtractor(string dstPath) : DataExtractorBase(dstPath + "\\images\\")
{
}

void ImageExtractor::extractData(k4a_capture_t captureHandle)
{
	depthImage = k4a_capture_get_depth_image(captureHandle);
	colorImage = k4a_capture_get_color_image(captureHandle);

	if (depthImage == nullptr || colorImage == nullptr) {
		return;
	}

	uint64_t timestamp = k4a_image_get_device_timestamp_usec(colorImage);
	int color_width = k4a_image_get_width_pixels(colorImage);
	int color_height = k4a_image_get_height_pixels(colorImage);
	uint8_t* color_buffer = k4a_image_get_buffer(colorImage);
	Mat color_mat(color_height, color_width, CV_8UC4, (void*)color_buffer);
	imwrite(_dstPath + to_string(timestamp) + ".png", color_mat);

	k4a_image_release(depthImage);
	k4a_image_release(colorImage);
	_nrFrames += 1;
}
