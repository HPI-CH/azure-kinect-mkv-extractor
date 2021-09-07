#pragma once
#include "data_extractor_base.h"

#include <string>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <vector>

using namespace std;


struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

class PointcloudExtractor : public DataExtractorBase {

public:

	PointcloudExtractor(k4a_calibration_t calibration, string output_path);
	void extractData(k4a_capture_t captureHandle);
    static void write_point_cloud(vector<color_point_t> points, string file_name);

private:
	k4a_transformation_t mTransformationHandle = nullptr;
	k4a_image_t pointcloudImage = nullptr;
    k4a_image_t convColor = nullptr;
    int depthWidth;
    int depthHeight;
    
    k4a_image_t depthImage;
    k4a_image_t colorImage;
};
