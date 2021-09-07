#pragma once
#include "data_extractor_base.h"

#include <fstream>
#include <string>
#include <vector>

#include <k4a/k4a.h>
#include <k4abt.h>

using namespace std;

class JointExtractor : public DataExtractorBase {

public:

	JointExtractor(string dstPath, k4a_calibration_t calibration, bool useGPU);
	void extractData(k4a_capture_t captureHandle);

	void printNrValidFrames() {
		cout << "Joint Extractor: " << _nrFrames << endl;
	}

private:
	void addErrorLine(uint64_t timestamp);
	string createHeader(vector<char> axes, char d);

	string _joint_file_3d;
	string _joint_file_2d;
	string _orientation_file;
	ofstream _file_stream;

	char d = ';';

	k4a_calibration_t _calibration;
	k4abt_tracker_t _tracker = nullptr;
	k4a_image_t depthImage = nullptr;
	k4a_image_t colorImage = nullptr;
	k4abt_frame_t body_frame = nullptr;
};
