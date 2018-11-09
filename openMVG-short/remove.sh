cd /usr/local
sudo rm -r include/openMVG
sudo rm -r include/openMVG_dependencies
cd lib

sudo rm liblib_clp.a liblib_CoinUtils.a liblib_Osi.a liblib_OsiClpSolver.a

sudo rm libopenMVG_ceres.a libopenMVG_cxsparse.a libopenMVG_easyexif.a libopenMVG_lemon.a libopenMVG_fast.a libopenMVG_jpeg.a libopenMVG_png.a libopenMVG_tiff.a libopenMVG_stlplus.a libopenMVG_zlib.a

sudo rm libopenMVG_features.a libopenMVG_geometry.a libopenMVG_image.a libopenMVG_linearProgramming.a libopenMVG_lInftyComputerVision.a libopenMVG_matching.a libopenMVG_numeric.a libopenMVG_sfm.a libopenMVG_system.a libvlsift.a libopenMVG_kvld.a libopenMVG_matching_image_collection.a libopenMVG_multiview.a

sudo rm openMVG-targets.cmake openMVG-targets-release.cmake
