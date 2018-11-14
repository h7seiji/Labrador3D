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
 **/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <sys/time.h>

using namespace std;
//using namespace cv;

// ==========================================================
// Functions communication
// ==========================================================

/// FUNCOES DE RECEBE/ENVIA INT E BYTE
void sendInt(int sockfd, int m) {
    if (write(sockfd,&m,sizeof(m)) < 0)perror("ERROR sendInt");
}

int receiveInt(int sockfd){
    int num = 0;
    if(read(sockfd,&num,sizeof(num)) < 0)perror("ERROR receiveInt");
    return num;
}

void receiveBytes(int sockfd, unsigned char *buf, int nBytesToReceive){
    int bytes_acumulados = 0;
    int bytes_recebidos = 0;
    while(bytes_acumulados < nBytesToReceive){
        bytes_recebidos = read(sockfd,buf+bytes_acumulados,nBytesToReceive-bytes_acumulados);
        if(bytes_recebidos == -1) perror("recv");
        bytes_acumulados += bytes_recebidos;
    }
}

void sendBytes(int sockfd, unsigned char *buf, int nBytesToSend){
    while(nBytesToSend > 0){
        int bytes_enviados = write(sockfd,buf,nBytesToSend);
        if(bytes_enviados == -1) perror("send");
        nBytesToSend -= bytes_enviados;
        buf += bytes_enviados;
    }
}

/// LE ARQUIVO .DESC, TRANSFORMA EM BYTES E ENVIA
void descToBinaryAndSend(int sockfd, std::string arquivo_desc){
	 std::ifstream is (arquivo_desc.c_str(), std::ifstream::binary);
     if (is) {
        // get length of file:
        is.seekg (0, is.end);
        int length = is.tellg();
        is.seekg (0, is.beg);

        char * buffer = new char [length];

        std::cout << ".DESC Reading " << length << " characters... " << endl;
        // read data as a block:
        is.read (buffer,length);
		
        if (is)
			std::cout << ".DESC all characters read successfully." << endl;
        else{
			std::cout << ".DESC error: only " << is.gcount() << " could be read" << endl;
			exit(0);
        }
        is.close();
	
		sendInt(sockfd,length);//envia um inteiro que represeta o numero de bytes a setem enviados
		sendBytes(sockfd,(unsigned char*) buffer,length);//envia os bytes
       
        delete[] buffer;
    }
}

/// LE ARQUIVO .FEAT, TRANSFORMA EM BYTES E ENVIA
void featToBinaryAndSend(int sockfd, std::string arquivo_feat){
	 std::ifstream is (arquivo_feat.c_str(), std::ifstream::binary);
     if (is) {
		 
        // get length of file:
        is.seekg (0, is.end);
        int length = is.tellg();
        is.seekg (0, is.beg);

        char * buffer = new char [length];

        std::cout << ".FEAT Reading " << length << " characters... " << endl;
        // read data as a block:
        is.read (buffer,length);
		if (is)
			std::cout << ".FEAT all characters read successfully." << endl;
        else{
			std::cout << ".FEAT error: only " << is.gcount() << " could be read" << endl;
			exit(0);
        }
        is.close();
	
		sendInt(sockfd,length);//envia um inteiro que represeta o numero de bytes a setem enviados
		sendBytes(sockfd,(unsigned char*) buffer,length);//envia os bytes
       
        delete[] buffer;
    }
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void conexao(int *sockfd, int portno, struct hostent *server){
	
    struct sockaddr_in serv_addr;
    
	*sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    cout << " . . . conectando" << endl;
    while(connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){}    
    return;
}


int main(int argc, char **argv)
{
	cout << endl << "(4) ENVIAR DADOS" << endl;
	if (argc < 4) {
        fprintf(stderr,"usage %s hostname port\n", argv[0]);
        exit(0);
    }

	// FILENAME
	//string sImageFilename = "00000002.jpg";
	//string sFeat = "00000002.feat";
	//string sDesc = "00000002.desc";
	string a = argv[3];
    string sImageFilename = a + ".jpg";
    string sFeat = a + ".feat";
    string sDesc = a + ".desc";


    // REMOVE OPENCV HERE !!

	cv::Mat image;
    vector<unsigned char> vb;
    vector<int> param;	// parametros compacta imagem;
    param.push_back(1); //CV_IMWRITE_JPEG_QUALITY
    param.push_back(80);
	
	int sockfd;
    struct hostent *server;
    
    // IP e PORTA
	int portno = atoi(argv[2]);
    server =  gethostbyname(argv[1]);// ip server
    
    conexao(&sockfd, portno, server);
	
	// ==========================================================
	// SEND IMAGEM TO SERVER
	// ==========================================================

	image = imread(sImageFilename, 1);// carrega imagem (CV_LOAD_IMAGE_COLOR)
    bool y=cv::imencode(".jpg",image,vb,param);// compacta a imagem
    sendInt(sockfd,vb.size());//envia um número que represeta o tamanho da imagem compactada em bytes
    sendBytes(sockfd,vb.data(),vb.size());//envia a imagem compactada
    
	// ==========================================================
	// SEND .FEAT .DESC TO SERVER
	// ==========================================================
    
    int conf_img = receiveInt(sockfd);
	featToBinaryAndSend(sockfd, sFeat);
    int conf_feat = receiveInt(sockfd);
	descToBinaryAndSend(sockfd, sDesc);
	
    close(sockfd);
	
	cout << "Sent!" << endl
		 << "-----------------------" << endl << endl;
	return EXIT_SUCCESS;
    
}
