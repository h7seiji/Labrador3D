/** OPENMVG TEST IN LABRADOR
 * 
 * IMAGE AND FEATURE LIBRARIES ONLY
 * 
 * EXAMPLE
 * ./main kermit000.jpg 2
 * 
 */

// Image
#include "openMVG/image/image_io.hpp"

// FEATURE DETECTION AND DESCRIPTION
#include "openMVG/features/akaze/image_describer_akaze_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
//#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"

#include <iostream>
#include <string>
#include <utility>
#include <sys/time.h>

using namespace openMVG::image;
using namespace openMVG::features;
using namespace std;

// ==========================================================
// ENUMERATE to choose options
// ==========================================================
EDESCRIBER_PRESET stringToEnum(const uint8_t i)
{
  EDESCRIBER_PRESET preset;
  if (i==0)
    preset = NORMAL_PRESET;
  else
  if (i==1)
    preset = HIGH_PRESET;
  else
  if (i==2)
    preset = ULTRA_PRESET;
  else
    preset = EDESCRIBER_PRESET(-1);
  return preset;
}



int main(int argc, char **argv)
{
	// =================================================================
	//   ( 0 )   SET PARAMETERS
	// =================================================================
	
	if(argc!=3) {
		cout << "Wrong arguments." << endl
			 << "  [1] image file name" << endl
			 << "  [2] description preset" << endl;
		return EXIT_FAILURE;
	}
	
	string sImageFilename = argv[1];
	int iDescPreset = atoi(argv[2]);
	if(iDescPreset<0 || iDescPreset>2) {
		cout << "Description preset must be 0, 1 or 2." << endl;
		return EXIT_FAILURE;
	}
	string sView_filename = "test.jpg";
	
	// TIMERS
	struct timeval start, end;
	long mtime, seconds, useconds;
	
	// images container
	Image<RGBColor> imageRGB;
	Image<unsigned char> imageGray;
	
	
	// =================================================================
	//   ( 1 )  INITIALIZE IMAGE DESCRIBER
	// =================================================================
	// always AKAZE MSURF
	
	unique_ptr<Image_describer> image_describer;
	image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF));
	image_describer->Set_configuration_preset(stringToEnum(iDescPreset));
    
    
    // =================================================================
	//   ( 2 )  READ AND WRITE IMAGE
	// =================================================================
	
	cout << "Read and Write image: ";
	{
		gettimeofday(&start,NULL);
		
		ReadImage(sImageFilename.c_str(), &imageRGB);
		WriteImage(sView_filename.c_str(), imageRGB);
		
		gettimeofday(&end,NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = (1000000*seconds + useconds)/1000;
		cout << mtime << " ms" << endl;
	}
	
	// =================================================================
	//   ( 3 )  CONVERT TO GRAYSCALE
	// =================================================================
	// you can also read image directly to grayscale format
	
	cout << "Converting to grayscale: ";
	{
		gettimeofday(&start,NULL);
		
		ConvertPixelType(imageRGB, &imageGray);
		
		gettimeofday(&end,NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = (1000000*seconds + useconds)/1000;
		cout << mtime << " ms" << endl;
	}
	
	// =================================================================
	//   ( 4 )  FEATURE DETECTION AND DESCRIPTION, AND EXPORT FILES
	// =================================================================
	
	cout << "Detecting point features" << endl;
	{
		gettimeofday(&start,NULL);
		
		auto regions = image_describer->Describe(imageGray);
		
		gettimeofday(&end,NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = (1000000*seconds + useconds)/1000;
		cout << "Regions: " << regions->RegionCount()
			 << "\tTime: " << mtime << " ms" << endl;
		
		cout << "Exporting Files: ";
		gettimeofday(&start,NULL);
		
		image_describer->Save(regions.get(), "image.feat","image.desc");
		
		gettimeofday(&end,NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = (1000000*seconds + useconds)/1000;
		cout << mtime << " ms" << endl;
	
	}
	
	return EXIT_SUCCESS;
    
}


