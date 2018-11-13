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
 * 
 **/



// The <cereal/archives> headers are special and must be included first.
#include <cereal/archives/json.hpp>

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

// FEATURE DETECTION AND DESCRIPTION
#include "openMVG/features/feature.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
//#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"

// FEATURE MATCHING
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
//#include "openMVG/matching/matcher_brute_force.hpp"
// geometric filtering
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
//#include "openMVG/matching_image_collection/E_ACRobust.hpp"

// STRUCTURE FROM MOTION
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
// direct triangulation
#include "openMVG/tracks/tracks.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
// structure from known poses
#include "openMVG/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"
#include "openMVG/geometry/frustum.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data_filters_frustum.hpp"

// command line and display progress
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"

// BUNDLE ADJUSTMENT
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"

// FOR EXPORT
// none


#include <iostream>
#include <string>
#include <utility>
#include <sys/time.h>



using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::cameras;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::matching_image_collection;
using namespace openMVG::sfm;
using namespace std;


// ==========================================================
// ENUMERATE to choose options
// ==========================================================



EDESCRIBER_PRESET stringToEnum(const string & sPreset)
{
  EDESCRIBER_PRESET preset;
  if (sPreset == "0")
    preset = NORMAL_PRESET;
  else
  if (sPreset == "1")
    preset = HIGH_PRESET;
  else
  if (sPreset == "2")
    preset = ULTRA_PRESET;
  else
    preset = EDESCRIBER_PRESET(-1);
  return preset;
}


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
	// =================================================================
	// =================================================================
	//   ( 0 )   S E T   O P T I O N S
	// =================================================================
	// =================================================================
	
	CmdLine cmd;

	string sSfM_Data_Filename = "init/calibrated_cameras.bin";
	string sMatches_Pairs_Filename = "init/match_pairs.txt";
	const string sDescribeDir = "imageDescribe";
	
	string sDescPreset = "NORMAL";
	double dMax_reprojection_error = 4.0; // for both methods
	
	cmd.add( make_option('p', sDescPreset, "describer_preset") );
	cmd.add( make_option('r', dMax_reprojection_error, "residual_threshold"));

	//cmd.add( make_option('n', num_port, "num ports") );
	//cmd.add( make_option('w', img_width, "image width") );
	//cmd.add( make_option('h', img_height, "image height") );
	
	if (argc == 1) 
	{
		cerr << "Invalid parameter." << endl;
		return EXIT_FAILURE;
	}
	cmd.process(argc, argv);
	
	// =================================================================
	// =================================================================
	//   ( 1 )  LOAD SFM_DATA AND VERIFY PARAMETERS
	// =================================================================
	// =================================================================
	
	// Load input SfM_Data scene
	// read views, intrinsics and extrinsics
	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS))) {
		cerr << endl
		  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << endl;
		return EXIT_FAILURE;
	}
	
	// Verify Describer Preset
	if(stringToEnum(sDescPreset) == -1) {
		cerr << endl << "Descriptor preset wrongly provided." << endl;
		return EXIT_FAILURE;
	}

	// DELETE PREVIOUS WORK FILES
	const string sOutDir = "output";
	limpar_e_criar_pasta(sOutDir);
	
	
	// ==========================================================
	// (1.1) PRINT OPTIONS SET
	// ==========================================================
	
	// TIMERS
	struct timeval start, end;
	long mtime, seconds, useconds, aux_timer;
	
	Views & views = sfm_data.views;
	Intrinsics & intrinsics = sfm_data.intrinsics;
	
	
	// =================================================================
	// =================================================================
	//   ( 2 )  FEATURE DETECTION AND DESCRIPTION
	// =================================================================
	// =================================================================
	
	// (2.1) INITIALIZE DESCRIBER
	// (2.2) EXPORT IMAGE EDESCRIBER TO JSON
    {
    	unique_ptr<Image_describer> image_describer;
		image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF));
		image_describer->Set_configuration_preset(stringToEnum(sDescPreset));

		const string sImage_describer = stlplus::create_filespec(sDescribeDir, "image_describer", "json");
		ofstream stream(sImage_describer.c_str());
		if (!stream.is_open())
			return EXIT_FAILURE;

		cereal::JSONOutputArchive archive(stream);
		archive(cereal::make_nvp("image_describer", image_describer));
		auto regionsType = image_describer->Allocate();
		archive(cereal::make_nvp("regions_type", regionsType));
    }

	// =================================================================
	// =================================================================
	//   ( 3 )  PAIRWISE FEATURE MATCHING
	// =================================================================
	// =================================================================
	
	PairWiseMatches matches;
	
	cout << "\n(3) PAIRWISE FEATURE MATCHING\n\n";
	
	
	// ==========================================================
	// (3.1) LOAD REGIONS
	// ==========================================================
	
	// Regions Provider
	shared_ptr<Regions_Provider> regions_provider;
	regions_provider = make_shared<Regions_Provider>();
	
	const string sImage_describer = stlplus::create_filespec(sDescribeDir, "image_describer", "json");
	unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
	
	if (!regions_provider->load(sfm_data, sDescribeDir, regions_type)) {
		cerr << endl << "Invalid regions." << endl;
		return EXIT_FAILURE;
	}
	
	
	// ==========================================================
	// (3.1) Initialize Matcher
	// ==========================================================
	
	// from indMatch.hpp
	PairWiseMatches map_PutativesMatches;
	float fDistRatio = 0.8f; //default
	
	// from Matcher.hpp
	unique_ptr<Matcher> collectionMatcher;
	collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions(fDistRatio));

	// pairwise matching Pair of Indexes
	Pair_Set pairs;
	
	// If there is match pairs file
	if (!loadPairs(sfm_data.GetViews().size(), sMatches_Pairs_Filename, pairs))
	{
		for (int i=0; i<sfm_data.views.size()-1; ++i)
		{
			Views::const_iterator it1 = sfm_data.views.begin();
			advance(it1, i);
			const IndexT v1 = it1->second.get()->id_view;
			for(int j=i+1; j<sfm_data.views.size(); j++)
			{
				Views::const_iterator it2 = sfm_data.views.begin();
				advance(it2, j);
				const IndexT v2 = it2->second.get()->id_view;
				//cout << "(" << v1 << "," << v2 << ")" << endl;
				pairs.insert(Pair(v1,v2));
			}
		}
	}
	else
		pairs = exhaustivePairs(sfm_data.GetViews().size());
	

	aux_timer = 0;
	// ==========================================================
	// (3.2) PAIRWISE MATCHING
	// ==========================================================
	gettimeofday(&start,NULL);
	
	// esta funcao nao usa SfM_Data pra nada
	collectionMatcher->Match(sfm_data, regions_provider, pairs, map_PutativesMatches);
	
	gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	aux_timer += mtime;

	cout << endl << "Matched Pairs:" << endl;
	Pair_Set:: iterator it;
	for( it = pairs.begin(); it!=pairs.end(); ++it)
	{
		Pair par = *it;
		cout << par.first << "," << par.second << "\tNo of matches: " << map_PutativesMatches.at(par).size() << endl;
	}
	cout << "Processing Time: " << mtime << " ms" << endl << endl;
	
	// ==========================================================
	// (3.3) GEOMETRIC FILTERING OF MATCHES
	// ==========================================================
	gettimeofday(&start,NULL);
	
	unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
	  new ImageCollectionGeometricFilter(&sfm_data, regions_provider));
	
	int imax_iteration = 1024;
	bool bGuided_matching = false;
	const double d_distance_ratio = 0.6;
	
	filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
	  map_PutativesMatches, bGuided_matching, d_distance_ratio);
	matches = filter_ptr->Get_geometric_matches();

	
	gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	aux_timer += mtime;

	cout << "Matched Pairs (after Geometric Filtering):" << endl;
	for( PairWiseMatches::const_iterator iter = matches.begin();
		 iter!=matches.end(); ++iter)
	{
		cout << (iter->first).first << "," << (iter->first).second << "\tNo of matches: " << iter->second.size() << endl;
	}
	
	cout << "Processing Time: " << mtime << " ms" << endl << endl;
    
    cout << "Feature Matching Total Time: " << aux_timer << " ms" << endl;

    // =================================================================
	// =================================================================
	//   ( 4 )  STRUCTURE FROM MOTION SOLVER
	// =================================================================
	// =================================================================
    
    cout << endl
		 << "\n(4) DIRECT TRIANGULATION\n\n";
    
    gettimeofday(&start,NULL);
    
	// =================================================================
	// ====================  DIRECT TRIANGULATION  =====================
	// =================================================================
	
	cout
	  << "======================================\n"
	  << "Robust triangulation of the match file\n"
	  << "======================================"  << endl;
	
	// Compute the tracks from the pairwise estimation
	// Compute tracks from matches
	const int min_track_length = 2;
	tracks::STLMAPTracks tracks;
	{
	  // Building tracks
	  tracks::TracksBuilder tracks_builder;
	  tracks_builder.Build(matches);
	  // Filtering Tracks
	  tracks_builder.Filter(min_track_length);
	  // Build tracks with STL compliant type :
	  tracks_builder.ExportToSTL(tracks);

	  // Display some statistics about the computed tracks
	  {
		std::ostringstream track_stream;
		std::set<uint32_t> images_id;
		tracks::TracksUtilsMap::ImageIdInTracks(tracks, images_id);
		
		track_stream
		  << "------------------" << "\n"
		  << "-- Tracks Stats --" << "\n"
		  << " Tracks number: " << tracks_builder.NbTracks() << "\n"
		  << " Images Id: " << "\n";
		std::copy(images_id.begin(), images_id.end(),
		  std::ostream_iterator<uint32_t>(track_stream, ", "));
		track_stream << "\n------------------" << "\n";

		std::map<uint32_t, uint32_t> track_length_histogram;
		tracks::TracksUtilsMap::TracksLength(tracks, track_length_histogram);
		track_stream << "TrackLength, Count" << "\n";
		for (const auto & it : track_length_histogram)  {
		  track_stream << "\t" << it.first << "\t" << it.second << "\n";
		}
		track_stream << "\n";
		std::cout << track_stream.str();
	  }
	}

	cout
	  << "====================================\n"
	  << "Robust triangulation of the tracks\n"
	  << " - tracks computed from a match file\n"
	  << "====================================" << endl;

	// Fill sfm_data with the computed tracks (no 3D yet)
	Landmarks & structure = sfm_data.structure;
	IndexT idx(0);
	for (const auto & tracks_it : tracks)
	{
	  structure[idx] = {};
	  Observations & obs = structure.at(idx).obs;
	  for (const auto & track_it : tracks_it.second)
	  {
		const auto imaIndex = track_it.first;
		const auto featIndex = track_it.second;
		const Vec2 & pt = regions_provider->get(imaIndex)->GetRegionPosition(featIndex);
		obs[imaIndex] = {pt, featIndex};
	  }
	  ++idx;
	}

	// Compute 3D position of the landmark of the structure by robust triangulation of the observations
	{
	  //const double max_reprojection_error = 4.0; // pixels reprojection error
	  bool console_verbose = true;
	  SfM_Data_Structure_Computation_Robust structure_estimator(
		dMax_reprojection_error,
		min_track_length,
		min_track_length,
		console_verbose);
	  structure_estimator.triangulate(sfm_data);
	}
	
	regions_provider.reset(); // Regions are not longer needed.
	RemoveOutliers_AngleError(sfm_data, 2.0);

	cout << "\n#landmark found: " << sfm_data.GetLandmarks().size() << endl;
	
	
	// =================================================================
	// ======================  BUNDLE ADJUSTMENT  ======================
	// =================================================================

	cout << endl
	     << "\n (4.5) BUNDLE ADJUSTMENT\n\n";
	// Check that poses & intrinsic cover some measures (after outlier removal)
	const IndexT minPointPerPose = 12; // 6 min
	const IndexT minTrackLength = 3; // 2 min
	if (eraseUnstablePosesAndObservations(sfm_data, minPointPerPose, minTrackLength))
	{
	  KeepLargestViewCCTracks(sfm_data);
	  eraseUnstablePosesAndObservations(sfm_data, minPointPerPose, minTrackLength);
	  cout << "Point_cloud cleaning:\n"
		   << "\t #3DPoints: " << sfm_data.structure.size() << "\n";
	}

	cout << "Bundle adjustment..." << endl;
	Bundle_Adjustment_Ceres bundle_adjustment_obj;
	bundle_adjustment_obj.Adjust
	  (
		sfm_data,
		Optimize_Options(
		  Intrinsic_Parameter_Type::ADJUST_ALL,
		  Extrinsic_Parameter_Type::ADJUST_ALL,
		  Structure_Parameter_Type::ADJUST_ALL)
	  );
	
	
	gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	
	cout << endl << "Triangulation Processing Time: " << mtime << " ms" << endl;
	
	
	
	// ============================================
	// =============  FINAL EXPORTS  ==============
	// ============================================
	
	cout << endl
		 << "\n===============================\n"
	     << "=========== RESULTS ===========\n"
	     << "===============================\n\n";
    
	cout << "SUCCESSFULLY DONE." << endl
	     << "Computed Tracks: " << sfm_data.structure.size() << endl
	     << endl << "Global Processing Time: " << mtime << " ms" << endl << endl;
	
	Save(sfm_data,
		stlplus::create_filespec(sOutDir, "01_sparse_cloud", ".bin"),
		ESfM_Data(ALL));
	
	return EXIT_SUCCESS;
    
}


// =================================================================
// =================================================================
//   FUNCTIONS DEFINITIONS
// =================================================================
// =================================================================

