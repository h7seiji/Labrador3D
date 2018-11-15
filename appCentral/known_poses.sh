#!/bin/bash 

# ================================================
# ============= P A R A M E T R O S ==============
# ================================================
N=$1	# numero de placas
R=2	# resolucao da deteccao (0,1,2)
TESTE=1	# usa imagens de teste ou nao (0,1)
# ================================================

echo -e "===================================================\n========= RECONSTRUCTION FROM KNOWN POSES =========\n===================================================\n"
echo -e "Number of boards: $N\n"

# ================================================
REC=1

if [ ! -e init/calibrated_cameras.bin ]
then
  echo "ERROR: no calibrated cameras!"
  REC=0
fi

if [ ! -e MVE ]
then
  echo "ERROR: no MVE folder!"
  REC=0
fi

if [ ! -e imageDescribe ]
then
  echo "ERROR: no imageDescribe folder!"
  REC=0
fi
# ================================================

if [ $REC -eq 1 ]
then

START=$(date +%s%N)
./bin/takePictures $N $R $TESTE
END=$(date +%s%N)
DIFF=$( (($END - $START)/1000000) )
echo -e "Distributed Feature Detection Time: $DIFF ms\n"

START=$(date +%s%N)

./bin/triangulation -p $R
./bin/export2MVE
#./bin/p05-color -i output/01_sparse_cloud.bin -o output/01_colored.ply

# MVE dmrecon needs at least 5 images
# MVS-Texturing may gives some error for few images
echo -e "\n\nDEPTHMAP RECONSTRUCTION\n"
./bin/dmrecon -s2 MVE
./bin/scene2pset -F2 MVE output/02_dense_cloud.ply
echo -e "\n\nSURFACE RECONSTRUCTION\n"
./bin/fssrecon output/02_dense_cloud.ply output/03_surface.ply
./bin/meshclean -t10 output/03_surface.ply output/04_surface_clean.ply
echo -e "\n\nTEXTURE RECONSTRUCTION\n"
./bin/texrecon MVE::undistorted output/04_surface_clean.ply output/05_textured

END=$(date +%s%N)
DIFF2=$(( $DIFF + (($END-$START)/1000000) ))
echo -e "\nReconstruction Time: $DIFF2 ms"

fi
