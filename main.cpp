#include "data_extractor_base.h"
#include "joint_extractor.h"
#include "image_extractor.h"
#include "pointcloud_extractor.h"

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <filesystem>
#include <map>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>

using namespace std;

vector<DataExtractorBase*> extractors;
map<string, bool> arguments{
	{"--skeleton", false},
	{"--pointcloud", false},
	{"--rgb", false}
};

bool use_gpu = false;

bool processMkvOffline(string input_path, string output_path)
{
	k4a_playback_t playback_handle;
	k4a_calibration_t calibration;
	VERIFY(k4a_playback_open(input_path.c_str(), &playback_handle), "Cannot open recording.");
	VERIFY(k4a_playback_set_color_conversion(playback_handle, K4A_IMAGE_FORMAT_COLOR_BGRA32), "Cant set BGRA as color format mode.");
	VERIFY(k4a_playback_get_calibration(playback_handle, &calibration), "Failed to get calibration.");

	map<string, bool>::iterator it;
	for (it = arguments.begin(); it != arguments.end(); it++)
	{
		if ((*it).second == false) { continue; }

		if ((*it).first == "--skeleton") 
		{
			extractors.push_back(new JointExtractor(output_path, calibration, use_gpu));
		} 
		else if ((*it).first == "--rgb")
		{
			extractors.push_back(new ImageExtractor(output_path));
		}
		else
		{
			extractors.push_back(new PointcloudExtractor(calibration, output_path));
		}
	}

	int frame_count = 0;
	k4a_capture_t capture_handle = nullptr;

	while (k4a_playback_get_next_capture(playback_handle, &capture_handle) != K4A_STREAM_RESULT_EOF)
	{
		cout << "Rendering frame " << frame_count << '\r';
		for (DataExtractorBase* value : extractors) {
			value->extractData(capture_handle);
		}
		k4a_capture_release(capture_handle);
		frame_count += 1;
	}

	cout << "Total read " << frame_count << " frames" << endl;
	cout << "Results saved in " << output_path;

	k4a_playback_close(playback_handle);
	return true;
}

static void showUsage()
{
	std::cerr << "Usage: offline_processor.exe <file_name.mkv> <option(s)>\n"
		<< "Options:\n"
		<< "\t--gpu\t\t using GPU for skeleton extraction\n"
		<< "\t--skeleton\t\t extract skeletons from video file\n"
		<< "\t--rgb\t\t extract RGB images from video file\n"
		<< "\t--pointcloud\t\t extract RGB+D images (pointclouds) from video file\n";
	string input;
	getline(cin, input);
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		showUsage();
		exit(0);
	}

	string output_path(argv[1]);
	output_path = output_path.substr(0, output_path.size() - 4);  // Remove the file ending
	filesystem::remove_all(output_path); // Remove existing directory with same name
	filesystem::create_directory(output_path);

	for (int i = 2; i < argc; i += 1) {
		auto itr = arguments.find(string(argv[i]));
		if (itr != arguments.end()) {
			(*itr).second = true;
		}
		else if (string(argv[i]) == "--gpu")
		{
			use_gpu = true;
		}
		else 
		{
			cerr << "Unknown argument: " << argv[i] << endl;
			showUsage();
			exit(0);
		}
	}

	return processMkvOffline(argv[1], output_path) ? 0 : -1;
}
