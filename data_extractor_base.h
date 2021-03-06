#pragma once
#include <string>
#include <iostream>

#include <k4a/k4a.h>


using namespace std;

class DataExtractorBase {

public:
	DataExtractorBase(string dstPath);
	virtual void extractData(k4a_capture_t captureHandle) = 0;	
	virtual void printNrValidFrames() = 0;

protected:
	string _dstPath;
	int _nrFrames = 0;
};
