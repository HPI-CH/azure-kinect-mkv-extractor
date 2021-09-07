#include "data_extractor_base.h"

#include <filesystem>

using namespace std;


DataExtractorBase::DataExtractorBase(string dstPath)
{
	_dstPath = dstPath;
	filesystem::create_directories(_dstPath);
}
