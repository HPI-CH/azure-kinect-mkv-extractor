#include "data_extractor_base.h"
#include <string>

#include <k4a/k4a.h>
#include <k4abt.h>

using namespace std;

class ImageExtractor : public DataExtractorBase {

public:
	ImageExtractor(string dstPath);
	void extractData(k4a_capture_t captureHandle);

private:
	k4a_image_t depthImage = nullptr;
	k4a_image_t colorImage = nullptr;
};
