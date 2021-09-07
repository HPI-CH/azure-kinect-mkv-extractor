#include "joint_extractor.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <filesystem>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>

using namespace std;


JointExtractor::JointExtractor(string dstPath, k4a_calibration_t calibration, bool useGPU) : DataExtractorBase(dstPath + "\\skeleton\\") {
	_calibration = calibration;

	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	if (useGPU) {
		tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
	}
	else {
		tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
	}

	VERIFY(k4abt_tracker_create(&_calibration, tracker_config, &_tracker), "Body tracker initialization failed!");

	// write 3D joint position file
	_joint_file_3d = _dstPath + "positions_3d.csv";
	_file_stream.open(_joint_file_3d);
	_file_stream << createHeader(vector<char>({ 'c', 'x', 'y', 'z' }), d);
	_file_stream.close();

	// write 3D joint orientation file
	_orientation_file = _dstPath + "orientations_3d.csv";
	_file_stream.open(_orientation_file);
	_file_stream << createHeader(vector<char>({ 'w', 'x', 'y', 'z' }), d);
	_file_stream.close();

	// write 2D joint position file
	_joint_file_2d = _dstPath + "positions_2d.csv";
	_file_stream.open(_joint_file_2d);
	_file_stream << createHeader(vector<char>({ 'c', 'x', 'y' }), d);
	_file_stream.close();
}

std::string JointExtractor::createHeader(vector<char> axes, char d)
{
	string header = "timestamp;body_idx;";
	string joint_name;
	int joint_count = (int)K4ABT_JOINT_COUNT;
	for (int i = 0; i < joint_count; i += 1)
	{
		joint_name = g_jointNames.find((k4abt_joint_id_t)i)->second;
		for (int axis = 0; axis < axes.size(); axis += 1) {
			char end = (i == (joint_count - 1) && (axis == axes.size() - 1)) ? '\n' : d;
			header += joint_name + " (" + axes[axis] + ')' + end;
		}
	}

	return header;
}

void JointExtractor::addErrorLine(uint64_t timestamp) {
	cout << "Append Error Line" << endl;
	string joint_line_3d(to_string(timestamp) + d + to_string(-1));
	for (uint32_t i = 0; i < (int)K4ABT_JOINT_COUNT * 4; i += 1) {
		joint_line_3d += d + to_string(-1);
	}

	// Write 3D joint positions to file
	_file_stream.open(_joint_file_3d, ios::app);
	_file_stream << joint_line_3d + "\n";
	_file_stream.close();
}

void JointExtractor::extractData(k4a_capture_t captureHandle)
{
	depthImage = k4a_capture_get_depth_image(captureHandle);
	colorImage = k4a_capture_get_color_image(captureHandle);

	if (depthImage == nullptr || colorImage == nullptr) {
		return;
	}

	if (k4abt_tracker_enqueue_capture(_tracker, captureHandle, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED)
	{
		return;
	}

	if (k4abt_tracker_pop_result(_tracker, &body_frame, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED) {
		return;
	}

	uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	uint64_t timestamp = k4abt_frame_get_device_timestamp_usec(body_frame);
	k4a_float2_t position_2d;
	int valid;

	if (num_bodies == 0) {
		addErrorLine(timestamp);
		return;
	}

	k4abt_skeleton_t skeleton;
	int body_id;
	for (uint32_t i = 0; i < num_bodies; i++)
	{
		VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &skeleton), "Get body from body frame failed!");
		body_id = k4abt_frame_get_body_id(body_frame, i);

		string joint_line_3d(to_string(timestamp) + d + to_string(body_id));
		string ori_line(to_string(timestamp) + d + to_string(body_id));
		string joint_line_2d(to_string(timestamp) + d + to_string(body_id));

		for (int j = 0; j < (int)K4ABT_JOINT_COUNT; j++)
		{
			k4abt_joint_t joint = skeleton.joints[j];
			joint_line_3d += d + to_string(joint.confidence_level) + d + to_string(joint.position.xyz.x) + d + to_string(joint.position.xyz.y) + d + to_string(joint.position.xyz.z);
			ori_line += d + to_string(joint.orientation.wxyz.w) + d + to_string(joint.orientation.wxyz.x) + d + to_string(joint.orientation.wxyz.y) + d + to_string(joint.orientation.wxyz.z);
	
			// Convert 3D to 2D coordinates using mapping function
			k4a_calibration_3d_to_2d(&_calibration, &joint.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &position_2d, &valid);
			joint_line_2d += d + to_string(joint.confidence_level) + d + to_string(position_2d.xy.x) + d + to_string(position_2d.xy.y);
		}

		// Write 3D joint positions to file
		_file_stream.open(_joint_file_3d, ios::app);
		_file_stream << joint_line_3d + "\n";
		_file_stream.close();
		
		// Write 3D joint orientations to file
		_file_stream.open(_orientation_file, ios::app);
		_file_stream << ori_line + "\n";
		_file_stream.close();

		// Write 3D joint positions to file
		_file_stream.open(_joint_file_2d, ios::app);
		_file_stream << joint_line_2d + "\n";
		_file_stream.close();
	}

	k4abt_frame_release(body_frame);
	return;
}
