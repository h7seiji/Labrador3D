#!/bin/bash 

# ================================================
# ============= P A R A M E T R O S ==============
# ================================================
N=$1	# numero de placas
R=2	# resolucao da deteccao (0,1,2)
F=612	# distancia focal da camera
TESTE=1	# usa imagens de teste ou nao (0,1)
# ================================================

echo -e "=================================================\n============== CAMERAS CALIBRATION ==============\n=================================================\n"
echo "Number of boards: $N"



START=$(date +%s%N)
./bin/dataInit -f $F -n $N -w 640 -h 480
./bin/takePictures $N $R $TESTE
END=$(date +%s%N)
DIFF=$((($END - $START)/1000000))
echo -e "Distributed Feature Detection Time: $DIFF ms\n"

START=$(date +%s%N)
./bin/sequentialSFM -p $R
END=$(date +%s%N)
DIFF2=$(( $DIFF + (($END - $START)/1000000) ))
echo -e "\nCalibration Total Time: $DIFF2 ms\n"


