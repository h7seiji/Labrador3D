#!/bin/bash 

# ======================================================
# ================ P A R A M E T R O S =================
# ======================================================
N=$1	# numero de placas
R=2	# resolucao da deteccao (0,1,2)
TESTE=1	# usa imagens de teste ou nao (0,1)

# ======================================================
# ==================  O U T R O S  =====================
# ======================================================
NET=enp3s0  # nome da conexao (ver com ifconfig)
PASS=xxx    # senha de usuario para usar sudo
# ======================================================

echo -e "===================================================\n========= RECONSTRUCTION FROM KNOWN POSES =========\n===================================================\n"
echo -e "Number of boards: $N\n"

# ======================================================
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
# ======================================================

if [ $REC -eq 1 ]
then

# ======================================================
# COMUNICATION
START=$(date +%s%N)
./bin/takePictures $N $R $TESTE
END=$(date +%s%N)
DIFF1=$((($END - $START)/1000000))
echo -e "Distributed Feature Detection Time: $DIFF1 ms\n"

# ======================================================
# Desativa conexao ethernet
echo $PASS | sudo -S ip link set down $NAME

# Cria arquivo TXT para registrar tempos
NUM=1
ONE=1
WRITE="1"
while [ $WRITE != "0" ]; do
  if [ ! -e "result$NUM.txt" ]
  then
    FILE="result$NUM.txt"
    touch $FILE
    WRITE="0"
  else
    NUM=$(($NUM + $ONE))
  fi
done
# ======================================================

# ======================================================
 # TRIANGULATION
START=$(date +%s%N)
./bin/triangulation -p $R -z $FILE
./bin/export2MVE
./bin/p05-color -i output/01_sparse_cloud.bin -o output/01_colored.ply
END=$(date +%s%N)
DIFF2=$((($END - $START)/1000000))

# ======================================================
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

echo -e "\nKNOWN POSES RECONSTRUCTION TEST: NETWORK\n" >> $FILE
echo -e "Feature Detection: $DIFF1 ms" >> $FILE
echo -e "Depthmap Recon: $DIFF3 ms" >> $FILE
echo -e "Exporting to pointcloud: $DIFF4 ms" >> $FILE
echo -e "Surface Recon: $DIFF5 ms" >> $FILE
echo -e "Surface Cleaning: $DIFF6 ms" >> $FILE
echo -e "Texturing: $DIFF7 ms" >> $FILE
echo -e "Total Time: $TOTAL ms" >> $FILE

sudo ip link set up $NAME

fi
