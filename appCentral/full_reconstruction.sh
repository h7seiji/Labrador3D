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



START=$(date +%s)
./bin/dataInit -f $F -n $N -w 640 -h 480
./bin/takePictures $N $R $TESTE
END=$(date +%s)
DIFF=$(( $END - $START ))
echo -e "Distributed Feature Detection Time: $DIFF seconds\n"

START=$(date +%s)

./bin/sequentialSFM -p $R
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



END=$(date +%s)
DIFF2=$(( $DIFF + $END - $START ))


echo -e "\nFull Reconstruction Total Time: $DIFF2 seconds\n"



