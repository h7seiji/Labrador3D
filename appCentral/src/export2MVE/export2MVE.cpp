/** OpenMVG TEST 16
 * 
 * STRUCTURE FROM MOTION
 * COMPUTE STRUCTURE FROM KNOWN POSES
 * 
 * MAIN SFM
 * (1) use match pairs info (accelerate feature matching)
 * (2) use precomputed camera poses info (direct triangulation)
 * (3) export to MVE (fastened by already saving images in folder structure)
 * 
 *  * other features
 * (4) undistort images before saving (barely needed)
 * (5) intermediation with .FEAT and .DESC files
 * 
 * 
 * 
 */

// BASICS
// SFM container, images
// views and intrinsics
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/cameras/cameras.hpp"
#include "openMVG/image/image_io.hpp"

// HANDLE FILES
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

// command line and display progress
#include "third_party/progress/progress_display.hpp"




#include <iostream>
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



bool exportToMVE2Format(
  const SfM_Data & sfm_data,
  const std::string & sOutDirectory // Output MVE2 files directory
  );

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
	
	struct timeval start, end;
	long mtime, seconds, useconds;
	
	gettimeofday(&start,NULL);

	// =================================================================
	// =================================================================
	
	cout << endl
	 << "-------------------------------------------------\n"
     << "--------------- EXPORTING TO MVE ----------------\n"
     << "-------------------------------------------------\n\n";

	// Load input SfM_Data scene
	// read views, intrinsics and extrinsics
	string sSfM_Data_Filename = "output/01_sparse_cloud.bin";
	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL)) ) {
		cerr << endl
		  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << endl;
		return EXIT_FAILURE;
	}

	{
		cout << "DELETING PREVIOUS FILES" << endl;
		
		if(stlplus::file_exists("MVE/synth_0.out"))
			stlplus::file_delete("MVE/synth_0.out");
		
		for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
		{
			Views::const_iterator iterViews = sfm_data.views.begin();
			advance(iterViews, i);
			View * view = iterViews->second.get();
			
			ostringstream padding;
			padding << std::setw(4) << std::setfill('0') << view->id_view;
			const string sImg_path = "view_" + padding.str() + ".mve";
			const string sPath = "MVE/views/" + sImg_path;
			
			if(stlplus::file_exists( stlplus::folder_append_separator(sPath)+"meta.ini" ))
				stlplus::file_delete( stlplus::folder_append_separator(sPath)+"meta.ini" );
			if(stlplus::file_exists( stlplus::folder_append_separator(sPath)+"depth-L2.mvei" ))
				stlplus::file_delete( stlplus::folder_append_separator(sPath)+"depth-L2.mvei" );
			if(stlplus::file_exists( stlplus::folder_append_separator(sPath)+"undist-L2.png" ))
				stlplus::file_delete( stlplus::folder_append_separator(sPath)+"undist-L2.png" );
			
		}
	
	}
	
	exportToMVE2Format(sfm_data, "MVE");
	
	gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	cout << "Exporting Time: " << mtime << " ms" << endl;

	// ==========================================================
	// ==========================================================
	
	return EXIT_SUCCESS;
    
}


// =================================================================
// =================================================================
//   FUNCTIONS DEFINITIONS
// =================================================================
// =================================================================



bool exportToMVE2Format(
  const SfM_Data & sfm_data,
  const std::string & sOutDirectory // Output MVE2 files directory
  )
{
  std::atomic<bool> bOk(true);
  // Create basis directory structure
  if (!stlplus::is_folder(sOutDirectory))
  {
    std::cout << "\033[1;31mCreating directory:  " << sOutDirectory << "\033[0m\n";
    stlplus::folder_create(sOutDirectory);
    bOk = stlplus::is_folder(sOutDirectory);
  }

  if (!bOk)
  {
    std::cerr << "Cannot access one of the desired output directories" << std::endl;
    return false;
  }

  // Export the SfM_Data scene to the MVE2 format
  {
    // Prepare to write bundle file
    // Get cameras and features from OpenMVG
    const Views & views = sfm_data.GetViews();
    const size_t cameraCount = views.size();
    // Tally global set of feature landmarks
    const Landmarks & landmarks = sfm_data.GetLandmarks();
    const size_t featureCount = landmarks.size();
    const std::string filename = "synth_0.out";
    std::cout << "Writing bundle (" << cameraCount << " cameras, "
        << featureCount << " features): to " << filename << "...\n";
    std::ofstream out(stlplus::folder_append_separator(sOutDirectory) + filename);
    out << "drews 1.0\n";  // MVE expects this header
    out << cameraCount << " " << featureCount << "\n";

    for (const auto & views_it : views)
    {
        const View * view = views_it.second.get();
        if (sfm_data.IsPoseAndIntrinsicDefined(view))
        {
            Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
            const IntrinsicBase * cam = iterIntrinsic->second.get();
            const Pose3 & pose = sfm_data.GetPoseOrDie(view);
            const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam);
            const Mat3 & rotation = pose.rotation();
            const Vec3 & translation = pose.translation();
            // Focal length and principal point must be normalized (0..1)
            const float flen = pinhole_cam->focal() / static_cast<double>(std::max(cam->w(), cam->h()));
            out
              << flen << " " << "0" << " " << "0" << "\n"  // Write '0' distortion values for pre-corrected images
              << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << "\n"
              << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << "\n"
              << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << "\n"
              << translation[0] << " " << translation[1] << " " << translation[2] << "\n";
        }
        else
        {
            // export a camera without pose & intrinsic info (export {0})
            // see: https://github.com/simonfuhrmann/mve/blob/952a80b0be48e820b8c72de1d3df06efc3953bd3/libs/mve/bundle_io.cc#L448
            for (int i = 0; i < 5 * 3; ++i)
              out << "0" << (i % 3 == 2 ? "\n" : " ");
        }
    }

    // For each feature, write to bundle: position XYZ[0-3], color RGB[0-2], all ref.view_id & ref.feature_id
    // The following method is adapted from Simon Fuhrmann's MVE project:
    // https://github.com/simonfuhrmann/mve/blob/e3db7bc60ce93fe51702ba77ef480e151f927c23/libs/mve/bundle_io.cc
    for (const auto & landmarks_it : landmarks)
    {
      const Vec3 & exportPoint = landmarks_it.second.X;
      out << exportPoint.x() << " " << exportPoint.y() << " " << exportPoint.z() << "\n";
      out << 250 << " " << 100 << " " << 150 << "\n";  // Write arbitrary RGB color, see above note

      // Write number of observations (features)
      const Observations & obs = landmarks_it.second.obs;
      out << obs.size();

      for (const auto & obs_it : obs)
      {
        const IndexT viewId = obs_it.first;
        const IndexT featId = obs_it.second.id_feat;
        out << " " << viewId << " " << featId << " 0";
      }
      out << "\n";
    }
    out.close();

    // Export (calibrated) views as undistorted images in parallel
    std::cout << "Exporting views..." << std::endl;

    // Create 'views' subdirectory
    const std::string sOutViewsDirectory = stlplus::folder_append_separator(sOutDirectory) + "views";
    if (!stlplus::folder_exists(sOutViewsDirectory))
    {
      std::cout << "\033[1;31mCreating directory:  " << sOutViewsDirectory << "\033[0m\n";
      stlplus::folder_create(sOutViewsDirectory);
    }

    C_Progress_display my_progress_bar(views.size());

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < static_cast<int>(views.size()); ++i)
    {
      if (!bOk) continue;
      auto views_it = views.begin();
      std::advance(views_it, i);
      const View * view = views_it->second.get();

      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
          continue;

      // Create current view subdirectory 'view_xxxx.mve'
      std::ostringstream padding;
      padding << std::setw(4) << std::setfill('0') << view->id_view;
      std::string sOutViewIteratorDirectory;
      sOutViewIteratorDirectory = stlplus::folder_append_separator(sOutViewsDirectory) + "view_" + padding.str() + ".mve";

      // We have a valid view with a corresponding camera & pose
      const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
      //const std::string dstImage =
      //  stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory), "undistorted","png");

      //if (!stlplus::folder_exists(sOutViewIteratorDirectory))
      //  stlplus::folder_create(sOutViewIteratorDirectory);

      //Image<RGBColor> image, image_ud, thumbnail;
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
      const IntrinsicBase * cam = iterIntrinsic->second.get();
      /*
      if (cam->have_disto())
      {
        // Undistort and save the image
        if (!ReadImage(srcImage.c_str(), &image))
        {
          std::cerr
            << "Unable to read the input image as a RGB image:\n"
            << srcImage << std::endl;
          bOk = false;
          continue;
        }
        UndistortImage(image, cam, image_ud, BLACK);
        if (!WriteImage(dstImage.c_str(), image_ud))
        {
          std::cerr
            << "Unable to write the output image as a RGB image:\n"
            << dstImage << std::endl;
          bOk = false;
          continue;
        }
      }
      else // (no distortion)
      {
        // If extensions match, copy the PNG image
        if (stlplus::extension_part(srcImage) == "PNG" ||
            stlplus::extension_part(srcImage) == "png")
        {
          stlplus::file_copy(srcImage, dstImage);
        }
        else
        {
          if (!ReadImage( srcImage.c_str(), &image) ||
              !WriteImage( dstImage.c_str(), image))
          {
            std::cerr << "Unable to read and write the image" << std::endl;
            bOk = false;
            continue;
          }
        }
      }
      */
      
      // Prepare to write an MVE 'meta.ini' file for the current view
      const Pose3 & pose = sfm_data.GetPoseOrDie(view);
      const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam);
      const Mat3 & rotation = pose.rotation();
      const Vec3 & translation = pose.translation();
      // Pixel aspect: assuming square pixels
      const float pixelAspect = 1.f;
      // Focal length and principal point must be normalized (0..1)
      const float flen = pinhole_cam->focal() / static_cast<double>(std::max(cam->w(), cam->h()));
      const float ppX = std::abs(pinhole_cam->principal_point()(0)/cam->w());
      const float ppY = std::abs(pinhole_cam->principal_point()(1)/cam->h());

      // For each camera, write to bundle: focal length, radial distortion[0-1],
      // rotation matrix[0-8], translation vector[0-2]
      std::ostringstream fileOut;
      fileOut
        << "# MVE view meta data is stored in INI-file syntax." << fileOut.widen('\n')
        << "# This file is generated, formatting will get lost." << fileOut.widen('\n')
        << fileOut.widen('\n')
        << "[camera]" << fileOut.widen('\n')
        << "focal_length = " << flen << fileOut.widen('\n')
        << "pixel_aspect = " << pixelAspect << fileOut.widen('\n')
        << "principal_point = " << ppX << " " << ppY << fileOut.widen('\n')
        << "rotation = " << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " "
        << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " "
        << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << fileOut.widen('\n')
        << "translation = " << translation[0] << " " << translation[1] << " "
        << translation[2] << " " << fileOut.widen('\n')
        << fileOut.widen('\n')
        << "[view]" << fileOut.widen('\n')
        << "id = " << view->id_view << fileOut.widen('\n')
        << "name = " << stlplus::filename_part(srcImage.c_str()) << fileOut.widen('\n');

      // To do:  trim any extra separator(s) from openMVG name we receive, e.g.:
      // '/home/insight/openMVG_KevinCain/openMVG_Build/software/SfM/ImageDataset_SceauxCastle/images//100_7100.JPG'
      std::ofstream file(
        stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory),
        "meta","ini").c_str());
      file << fileOut.str();
      file.close();

      // Save a thumbnail image "thumbnail.png", 50x50 pixels
      /*
      Image<RGBColor> thumbnail;
      thumbnail = create_thumbnail(image, 50, 50);
      const std::string dstThumbnailImage =
        stlplus::create_filespec(stlplus::folder_append_separator(sOutViewIteratorDirectory), "thumbnail","png");
      WriteImage(dstThumbnailImage.c_str(), thumbnail);
      */ 

      ++my_progress_bar;
    }
  }
  return bOk;
}

