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
 **/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <sys/time.h>


// IMAGE CONTAINER
#include "openMVG/image/image_io.hpp"

// FEATURE DETECTION AND DESCRIPTION
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
// para imprimir arquivo SVG com features
//#include "openMVG/features/svg_features.hpp"

// Utilidades para lidar com arquivos
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace std;

/// FUNCAO AUXILIAR PARA DEFINIR DETALHAMENTO NA DETECCAO
EDESCRIBER_PRESET stringToEnum(string sPreset)
{
  features::EDESCRIBER_PRESET preset;
  if (sPreset == "0")
    preset = features::NORMAL_PRESET;
  else
  if (sPreset == "1")
    preset = features::HIGH_PRESET;
  else
  if (sPreset == "2")
    preset = features::ULTRA_PRESET;
  else
    preset = features::EDESCRIBER_PRESET(-1);
  return preset;
}


int main(int argc, char **argv)
{
	cout << endl << "(3) PROCESSANDO IMAGEM" << endl;
	if (argc < 3) {
        fprintf(stderr,"usage %s hostname port\n", argv[0]);
        exit(0);
    }

    string a = argv[1];
	string sImageFilename = a + ".jpg";
	string sFeat = a + ".feat";
	string sDesc = a + ".desc";

	string preset = argv[2];
	/*
	ifstream myfile ("com1.txt");
	if (myfile.is_open())
	{	
		getline(myfile, preset);
		myfile.close();
	}*/
	cout << "Read reset: " << preset << endl;
	
	// ==========================================================
	// (1) INICIALIZACAO E CONFIGURACOES
	// ==========================================================
	
	// Tipo do Descritor
	// SIFT: deu problema na biblioteca pra mim
	// AKAZE-MLDB: eh um descritor binario, melhor nao usar
	// AKAZE-MSURF: detecta 2x mais features que o SIFT, mas eh mais lento
	// SIFT_ANATOMY: bom
	
	// Pega as dimensoes da imagem
	// talvez nem precise disso, pq saberemos a priori o tamanho da imagem
	// mas se quisermos configurar automatizadamente, pode ser util

	//ImageHeader imgHeader;
	//ReadImageHeader(sImageFilename.c_str(), &imgHeader);
	//float width = imgHeader.width;
	//float height = imgHeader.height;
	
	
	// ==========================================================
	// (2) INICIALIZACAO DO IMAGE-DESCRIBER
	// ==========================================================
	
	// Classe virtual que contem metodos de descricao
	unique_ptr<Image_describer> image_describer;
	image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF));
    
	image_describer->Set_configuration_preset(stringToEnum(preset));
	
	
	// ==========================================================
	// (3) DESCRICAO (deteccao de point features)
	// ==========================================================
	
	Image<unsigned char> imageGray;

	// Le imagem e descreve
	ReadImage(sImageFilename.c_str(), &imageGray);
	unique_ptr<Regions> region = image_describer->Describe(imageGray);
	image_describer->Save(region.get(), sFeat, sDesc);
	
	cout << "Number of Regions: " << region->RegionCount() << endl;

	return EXIT_SUCCESS;
    
}
