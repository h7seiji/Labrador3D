#!/bin/bash 

# ================================================
# ============= P A R A M E T R O S ==============
# ================================================
N=$1	# numero de placas
R=2	# resolucao da deteccao (0,1,2)
F=612	# distancia focal da camera
TESTE=1	# usa imagens de teste ou nao (0,1)
# ================================================

echo -e "===================================================\n=============== FULL RECONSTRUCTION ===============\n===================================================\n"
echo "Number of boards: $N"



START=$(date +%s%N)
./bin/dataInit -f $F -n $N -w 640 -h 480
./bin/takePictures $N $R $TESTE
END=$(date +%s%N)
DIFF1=$((($END - $START)/1000000))
echo -e "Distributed Feature Detection Time: $DIFF1 ms\n"

START=$(date +%s%N)
./bin/sequentialSFM -p $R
./bin/export2MVE
#./bin/p05-color -i output/01_sparse_cloud.bin -o output/01_colored.ply
END=$(date +%s%N)
DIFF2=$((($END - $START)/1000000))

# MVE dmrecon needs at least 5 images
# MVS-Texturing may gives some error for few images
echo -e "\n\nDEPTHMAP RECONSTRUCTION\n"
START=$(date +%s%N)
./bin/dmrecon -s2 MVE
END=$(date +%s%N)
DIFF3=$((($END - $START)/1000000))
echo -e "Depthmap Reconstruction Time: $DIFF3 ms\n"
START=$(date +%s%N)
./bin/scene2pset -F2 MVE output/02_dense_cloud.ply
END=$(date +%s%N)
DIFF4=$((($END - $START)/1000000))
echo -e "Exporting to pointcloud: $DIFF4 ms\n"

echo -e "\n\nSURFACE RECONSTRUCTION\n"
START=$(date +%s%N)
./bin/fssrecon output/02_dense_cloud.ply output/03_surface.ply
END=$(date +%s%N)
DIFF5=$((($END - $START)/1000000))
echo -e "Surface Reconstruction Time: $DIFF5 ms\n"
START=$(date +%s%N)
./bin/meshclean -t10 output/03_surface.ply output/04_surface_clean.ply
END=$(date +%s%N)
DIFF6=$((($END - $START)/1000000))
echo -e "Surface Cleaning Time: $DIFF6 ms\n"

echo -e "\n\nTEXTURE RECONSTRUCTION\n"
START=$(date +%s%N)
./bin/texrecon MVE::undistorted output/04_surface_clean.ply output/05_textured
END=$(date +%s%N)
DIFF7=$((($END - $START)/1000000))
echo -e "Texturing Time: $DIFF7 ms\n"

TOTAL=$(( $DIFF1 + $DIFF2 + $DIFF3 + $DIFF4 + $DIFF5 + $DIFF6 + $DIFF7 ))
echo -e "\nFull Reconstruction Total Time: $TOTAL ms\n"


