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
 * 
 */


// The <cereal/archives> headers are special and must be included first.
#include <cereal/archives/json.hpp>

// BASICS
// SFM container, images
// views and intrinsics
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/cameras/cameras.hpp"
#include "openMVG/image/image_io.hpp"

// HANDLE FILES
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

// FEATURE DETECTION AND DESCRIPTION
#include "openMVG/features/feature.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"

// FEATURE MATCHING
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
// geometric filtering
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"

// STRUCTURE FROM MOTION
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
// incremental
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"

// command line and display progress
#include "third_party/cmdLine/cmdLine.h"
//#include "third_party/progress/progress_display.hpp"


#include <iostream>
#include <fstream>
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


//Pair_Set compute_match_pairs( Poses calibrated, int i_neighbor_count );


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
	struct timeval start, end;
	long mtime, seconds, useconds, aux_timer;

	// =================================================================
	// =================================================================
	//   ( 0 )   S E T   O P T I O N S
	// =================================================================
	// =================================================================
	
	CmdLine cmd;

	const string sInitFolder = "init";
	const string sSfM_Data_Filename = "init/initial_sfm.bin";
	const string sDescribeDir = "imageDescribe";
	string sDescPreset = "NORMAL";
	string sIntrinsic_refinement_options = "ADJUST_ALL";
	string report;
	//double distortion1=0.0, distortion2=0.0, distortion3=0.0;

	
	//cmd.add( make_option('d', sDescriberType, "image_describer") );
	cmd.add( make_option('p', sDescPreset, "describer_preset") );
	cmd.add( make_option('r', sIntrinsic_refinement_options, "refineIntrinsics") );
	cmd.add( make_option('z', report, "report file name") );
	
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
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
		cerr << endl
		  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << endl;
		return EXIT_FAILURE;
	}

	if (!stlplus::folder_exists("MVE")){
		cerr << "MVE folder is missing." << endl;
		return EXIT_FAILURE;
	}

	if (!stlplus::folder_exists("imageDescribe")){
		cerr << "imageDescribe folder is missing." << endl;
		return EXIT_FAILURE;
	}

	const string sOutDir = "output";
	limpar_e_criar_pasta(sOutDir);
	
	// Verify Describer Preset
	if(stringToEnum(sDescPreset) == -1) {
		cerr << endl << "Descriptor preset wrongly provided." << endl;
		return EXIT_FAILURE;
	}
	
	const Intrinsic_Parameter_Type intrinsic_refinement_options =
    StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
	if (intrinsic_refinement_options == static_cast<Intrinsic_Parameter_Type>(0) )
	{
		cerr << "Invalid input for refinement option of bundle adjustment." << endl;
		return EXIT_FAILURE;
	}
	
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
	
	cout << "\n\n(3) PAIRWISE FEATURE MATCHING\n\n";
	
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
	pairs = exhaustivePairs(sfm_data.GetViews().size()); // number of images
	
	
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
    
    int imax_iteration = 2048;
    bool bGuided_matching = false;
    const double d_distance_ratio = 0.6;
    
	shared_ptr<Matches_Provider> matches_provider = make_shared<Matches_Provider>();
	PairWiseMatches & map_GeometricMatches = matches_provider->pairWise_matches_;

	filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
	  map_PutativesMatches, bGuided_matching, d_distance_ratio);
	map_GeometricMatches = filter_ptr->Get_geometric_matches();
    
    gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
    aux_timer += mtime;
    
    cout << "Matched Pairs (after Geometric Filtering):" << endl;
	for( PairWiseMatches::const_iterator iter = map_GeometricMatches.begin();
	     iter!=map_GeometricMatches.end(); ++iter)
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
		 << "\n(4) SEQUENTIAL STRUCTURE FROM MOTION\n\n";
    
    gettimeofday(&start,NULL);
    
    // feature provider
    shared_ptr<Features_Provider> feats_provider = make_shared<Features_Provider>();
    for (IndexT i = 0; i<sfm_data.views.size(); i++)
		feats_provider->feats_per_view[i] = regions_provider->get(i)->GetRegionsPositions();
	
    // matches provider
    // already stored in matching
    
    // Choose SFM Type
    SfM_Data results;

    SequentialSfMReconstructionEngine sfmEngine(
		sfm_data,
		sInitFolder,
		stlplus::create_filespec(sInitFolder, "Reconstruction_Report.html"));
		
	// configure
	int i_User_camera_model = PINHOLE_CAMERA; //_RADIAL3;

	// Configure the features_provider & the matches_provider
	sfmEngine.SetFeaturesProvider(feats_provider.get());
	sfmEngine.SetMatchesProvider(matches_provider.get());

	// Configure reconstruction parameters
	sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
	sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));
	sfmEngine.Set_Use_Motion_Prior(false);
	
	
	// HANDLE INITIAL PAIR PARAMETER
	Pair initialPairIndex;
	int maxmatch = 0;
	for( PairWiseMatches::const_iterator iter = map_GeometricMatches.begin();
		 iter!=map_GeometricMatches.end(); ++iter)
	{
		if(iter->second.size() > maxmatch) {
			maxmatch = iter->second.size();
			initialPairIndex = iter->first;
		}
	}
	sfmEngine.setInitialPair(initialPairIndex);
	cout << "Initial Pair: (" << initialPairIndex.first << "," << initialPairIndex.second << ")\n";
	
	// MAIN PROCESSING
	sfmEngine.Process();
	results = sfmEngine.Get_SfM_Data();
	
	gettimeofday(&end,NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = (1000000*seconds + useconds)/1000;
	
	cout << endl << "SFM Processing Time: " << mtime << " ms" << endl << endl;
	
	
	// =================================================================
	// =================================================================
	//   ( 5 )  RESULTS
	// =================================================================
	// =================================================================

	cout << endl
		 << "\n===============================\n"
	     << "=========== RESULTS ===========\n"
	     << "===============================\n\n";
	
	int nv = results.GetViews().size();
	int np = results.GetPoses().size();
	
	cout
	<< "Estimated POSES: " << np << " of " << nv << " VIEWS\n"
	<< "Estimated TRACKS: " << results.GetLandmarks().size() << endl;
	
	if(np < nv) {
		cout << "\nUncalibrated Cameras:\n";
		for (const auto & view : results.GetViews())
			if(  !results.IsPoseAndIntrinsicDefined(view.second.get())  )
				cout << view.second.get()->s_Img_path << endl;
		cout << endl << "FAILED TO ESTIMATE ALL CAMERA POSES." << endl;
	}
	else {
		cout << endl << "CAMERA POSES SUCCESSFULLY ESTIMATED." << endl;
		//int n_neighbor = 5;
		//Pair_Set export_pairs = compute_match_pairs( results.GetPoses(), n_neighbor );
		Pair_Set export_pairs = getPairs(map_GeometricMatches);
		cout << "Match Pairs: " << export_pairs.size() << " of " << pairs.size() << endl;
		savePairs(stlplus::create_filespec(sInitFolder, "match_pairs", ".txt"), export_pairs);
	}
	
	Save(results,
		stlplus::create_filespec(sInitFolder, "calibrated_cameras", ".bin"),
		ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS));
	Save(results,
		stlplus::create_filespec(sOutDir, "01_sparse_cloud", ".bin"),
		ESfM_Data(ALL));
	
	ofstream file (report);
	if (file.is_open())
	{
		file << "Feature Matching: " << aux_timer << " ms\n";
		file << "Sequential SFM: " << mtime << " ms\n";
		file.close();
	}
	
	return EXIT_SUCCESS;
    
}


// =================================================================
// =================================================================
//   FUNCTIONS DEFINITIONS
// =================================================================
// =================================================================

// ----------------------------------
/*
Pair_Set compute_match_pairs( Poses calibrated, int i_neighbor_count )
{
	// List the poses priors
	vector<Vec3> vec_pose_centers;
	map<IndexT, IndexT> contiguous_to_pose_id;
	set<IndexT> used_pose_ids;
	for (const auto & it : calibrated)
	{
		vec_pose_centers.push_back( it.second.center() );
		contiguous_to_pose_id[contiguous_to_pose_id.size()] = it.first;
		used_pose_ids.insert(it.first);
	}
	
	// Compute i_neighbor_count neighbor(s) for each pose
	Pair_Set pose_pairs;
	
	size_t contiguous_pose_id = 0;
	for (const Vec3 pose_it : vec_pose_centers)
	{
		matching::ArrayMatcherBruteForce<double> matcher;
		if (matcher.Build(vec_pose_centers[0].data(), vec_pose_centers.size(), 3))
		{
		  const double * query = pose_it.data();

		  IndMatches vec_indices;
		  std::vector<double> vec_distance;
		  const int NN = i_neighbor_count + 1; // since itself will be found
		  if (matcher.SearchNeighbours(query, 1, &vec_indices, &vec_distance, NN))
		  {
			for (size_t i = 1; i < vec_indices.size(); ++i)
			{
			  IndexT idxI = contiguous_to_pose_id.at(contiguous_pose_id);
			  IndexT idxJ = contiguous_to_pose_id.at(vec_indices[i].j_);
			  if (idxI > idxJ)
				std::swap(idxI, idxJ);
			  pose_pairs.insert(Pair(idxI, idxJ));
			}
		  }
		}
		++contiguous_pose_id;
		
	}
	
	return pose_pairs;
	
}
*/
