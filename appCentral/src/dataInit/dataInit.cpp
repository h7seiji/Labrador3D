/** OpenMVG TEST 16
 * 
 * STRUCTURE FROM MOTION
 * COMPUTE STRUCTURE FROM KNOWN POSES
 * 
 * INITIALIZATION
 * (1) estimate camera poses (with sequential/global sfm)
 * (2) save match pairs info
 * (3) initialize MVE folder structure (delete previous)
 * (4) save poses and img_paths in SFM_DATA.BIN
 * 
 * other features
 * (5) undistort images before saving (barely needed)
 * (6) intermediation with .FEAT and .DESC files
 * 
 * 
 * 
 */

// BASICS
// SFM container, images
// views and intrinsics
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/cameras/cameras.hpp"
#include "openMVG/image/image_io.hpp"

// HANDLE FILES
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

// command line and display progress
#include "third_party/cmdLine/cmdLine.h"
//#include "third_party/progress/progress_display.hpp"


#include <iostream>
#include <iomanip>
#include <string>
#include <utility>
#include <sys/time.h>



using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace std;


// ==========================================================
// ENUMERATE to choose options
// ==========================================================

void limpar_e_criar_pasta( string dir ) {
	if (stlplus::is_folder(dir)) {
		stlplus::folder_delete(dir,true);
		stlplus::folder_create(dir);
	}
	else
		stlplus::folder_create(dir);
}

string viewString_from_id( IndexT id ) {
	ostringstream padding;
	padding << std::setw(4) << std::setfill('0') << id;
	return "view_" + padding.str() + ".mve";
}



int main(int argc, char **argv)
{

	// TIMERS
	struct timeval start, end, global_start, global_end;
	long mtime, seconds, useconds, aux_timer;
	gettimeofday(&global_start,NULL);
	

	// =================================================================
	// =================================================================
	//   ( 0 )   S E T   O P T I O N S
	// =================================================================
	// =================================================================
	
	CmdLine cmd;
	
	int num_port = 0;
	double focal = -1.0;
	int img_width = 640;
	int img_height = 480;
	//double distortion1=0.0, distortion2=0.0, distortion3=0.0;
	
	cmd.add( make_option('f', focal, "focal_pixels") );
	cmd.add( make_option('n', num_port, "num ports") );
	cmd.add( make_option('w', img_width, "image width") );
	cmd.add( make_option('h', img_height, "image height") );
	//cmd.add( make_option('a', distortion1, "distortion1") );
	//cmd.add( make_option('b', distortion2, "distortion2") );
	//cmd.add( make_option('c', distortion3, "distortion3") );
	
	if (argc == 1) 
	{
		cerr << "Invalid parameter." << endl;
		return EXIT_FAILURE;
	}
	cmd.process(argc, argv);
	
	// ==========================================================
	// (0.1) VERIFY PARAMETERS
	// ==========================================================
	
	// Verify focal distance
	if(focal < 0) {
		cerr << endl << "Focal distance not specified." << endl;
		return EXIT_FAILURE;
	}

	if(num_port < 1) {
		cerr << endl << "Number of ports not specified." << endl;
		return EXIT_FAILURE;
	} else
	if (num_port > 16) {
		cerr << endl << "Number of ports not supported." << endl;
		return EXIT_FAILURE;
	}
	
	// ==========================================================
	// (0.3) DELETING & CREATING MVE DIRECTORY STRUCTURE
	// ==========================================================
	
	const string sOutDirectory = "MVE";
	limpar_e_criar_pasta(sOutDirectory);
	
	// Create 'views' subdirectory
	const string sOutViewsDirectory = stlplus::folder_append_separator(sOutDirectory) + "views";
    if (!stlplus::folder_exists(sOutViewsDirectory))
		stlplus::folder_create(sOutViewsDirectory);
	
	// image features and descriptions folder
	const string sDescribeDir = "imageDescribe";
	limpar_e_criar_pasta(sDescribeDir);
	
	// init folder
	const string sOutDir = "init";
	limpar_e_criar_pasta(sOutDir);
	
	
	// =================================================================
	// =================================================================
	//   ( 1 )  SET INTRINSICS AND VIEWS (images and focal input)
	// =================================================================
	// =================================================================

	SfM_Data sfm_data;
	sfm_data.s_root_path = sOutViewsDirectory;
	Views & views = sfm_data.views;
	Intrinsics & intrinsics = sfm_data.intrinsics;
	
	// ==========================================================
	// (1.1) READ VIEWS AND INTRINSICS
	// ==========================================================
	
	cout << endl
		 << "\n(0) INITIALIZING SFM-DATA AND FOLDERS\n\n";
	
	for ( int i=0; i<num_port; i++)
	{

		float width = img_width;
		float height = img_height;
		float ppx = width / 2.0;
		float ppy = height / 2.0;

		// SET INTRINSIC MATRIX FROM PARAMETERS (FOR SFM_DATA)
		shared_ptr<IntrinsicBase> intrinsic;
		intrinsic = std::make_shared<Pinhole_Intrinsic>
			(width, height, focal, ppx, ppy);
		
		// SET MVE VIEWS DIRECTORIES
		IndexT id = views.size();
		
		// Create current view subdirectory 'view_xxxx.mve'
		const string viewXXXX = viewString_from_id(id);
		limpar_e_criar_pasta(stlplus::folder_append_separator(sOutViewsDirectory) + viewXXXX);
		limpar_e_criar_pasta(stlplus::folder_append_separator(sDescribeDir) + viewXXXX);
		
		const string sUndistPath = viewXXXX + "/undistorted.jpg";
		
		// Creates SFM_DATA View
		View v(sUndistPath, views.size(), views.size(), views.size(), width, height);
		intrinsics[v.id_intrinsic] = intrinsic;
		views[v.id_view] = make_shared<View>(v);
		
		// trial prints
		cout << v.id_view << "\t"; // IndexT type
		cout << sUndistPath << "\n";
		
	}
	
	Save(sfm_data,
		stlplus::create_filespec(sOutDir, "initial_sfm", ".bin"),
		ESfM_Data(VIEWS|INTRINSICS));
	
    gettimeofday(&global_end,NULL);
	seconds  = global_end.tv_sec  - global_start.tv_sec;
	useconds = global_end.tv_usec - global_start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	cout << endl << "Initializing Time: " << mtime << " ms" << endl << endl;
	
	return EXIT_SUCCESS;
    
}