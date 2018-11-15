#include <iostream>
#include <iomanip>
#include <string>
#include <utility>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <math.h>

#include <cstdlib>
#include <pthread.h>

#include <vector>
#include <cstring>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;
//using namespace cv;

// ==========================================================
// Functions communication
// ==========================================================

string viewString_from_id( int id ) {
	ostringstream padding;
	padding << std::setw(4) << std::setfill('0') << id;
	return "view_" + padding.str() + ".mve";
}

void sendInt(int newsockfd, int m) {
    if (write(newsockfd,&m,sizeof(m)) < 0)perror("ERROR sendInt");
}

int receiveInt(int newsockfd){
    int num = 0;
    if(read(newsockfd,&num,sizeof(num)) < 0)perror("ERROR receiveInt");
    return num;
}

void receiveBytes(int newsockfd, unsigned char *buf, int nBytesToReceive){
    int bytes_acumulados = 0;
    int bytes_recebidos = 0;
    while(bytes_acumulados < nBytesToReceive){
        bytes_recebidos = read(newsockfd,buf+bytes_acumulados,nBytesToReceive-bytes_acumulados);
        if(bytes_recebidos == -1) perror("recv");
        bytes_acumulados += bytes_recebidos;
    }
}

void sendBytes(int newsockfd, unsigned char *buf, int nBytesToSend){
    while(nBytesToSend > 0){
        int bytes_enviados = write(newsockfd,buf,nBytesToSend);
        if(bytes_enviados == -1) perror("send");
        nBytesToSend -= bytes_enviados;
        buf += bytes_enviados;
    }
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void *connect_send_commands(void *parametros) {
	
	int *arg1;						// PEGA O NUMERO DA PORTA DESSA THREAD
	arg1 = (int *)parametros;		// SO FUNCIONOU DESSE JEITO ESTRANHO
	int portno;						//
	portno = *arg1;					//

	int tipo_detector = (int) (portno%100)/10;
	int resolucao_deteccao = (int) portno%10;
	
	portno -= (tipo_detector*10 + resolucao_deteccao);
	
    int sockfd, newsockfd;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    
	// INICIA CONEXAO
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    cout << " ... aguardando " << portno << endl;
    newsockfd = accept(sockfd,
                       (struct sockaddr *) &cli_addr,
                       &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");    
    // CONECTOU
	cout << "conectou " << portno << endl;
	
	// ENVIA COMANDOS PARA A PLACA
	sendInt(newsockfd,tipo_detector);
	sendInt(newsockfd, resolucao_deteccao);
	int confirmacao_placa = receiveInt(newsockfd);
	
	close(sockfd);
	cout << "confimacao de " << portno << " : " << confirmacao_placa << endl;
	
	pthread_exit(NULL);
}

void *connect_send_command_take_picture(void *parametros) {
	
	int *arg1;						// PEGA O NUMERO DA PORTA DESSA THREAD
	arg1 = (int *)parametros;		// SO FUNCIONOU DESSE JEITO ESTRANHO
	int portno;						//
	portno = *arg1;					//

	//cout << portno << endl;
	
    int sockfd, newsockfd;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    
	// INICIA CONEXAO
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd,
                       (struct sockaddr *) &cli_addr,
                       &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");    
    // CONECTOU
	cout << "conectou " << portno << endl;
	
	// ENVIA COMANDOS PARA A PLACA
	sendInt(newsockfd,1);
	int confirmacao_placa = receiveInt(newsockfd);
	
	close(sockfd);
	cout << "confimacao de " << portno << " : " << confirmacao_placa << endl;
	
	pthread_exit(NULL);
}

void *connect_receive_image_feat_desc(void *port_no) {
	int *port;			// PEGA O NUMERO DA PORTA DESSA THREAD
	port = (int *)port_no;		// SO FUNCIONOU DESSE JEITO ESTRANHO
	int portno;			//
	portno = *port;			//
	
	string sImage = "MVE/views/";
	string sFeat_Desc = "imageDescribe/";

	string viewXXXX = viewString_from_id( (portno-2000)/200 );
	string filename = viewXXXX + "/undistorted.jpg";
	//cout << portno << " - " << viewXXXX << endl;

	int nBytes;
	vector<unsigned char> vb;
	vector<unsigned char> vb_desc;
	vector<unsigned char> vb_feat;
	
	//cv::Mat image;
	//vector<int> param;
	//param.push_back(1); //CV_IMWRITE_JPEG_QUALITY
	//param.push_back(80);
    
	int sockfd, newsockfd;
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		error("ERROR opening socket");
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		error("ERROR on binding");
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd,
		       (struct sockaddr *) &cli_addr,
		       &clilen);
	if (newsockfd < 0)
		error("ERROR on accept");    
         
	// RECEBE IMAGEM COLORIDA
	nBytes = receiveInt(newsockfd);//recebe o tamanho da imagem compactada
	cout << "mBytes" << nBytes << endl;
	vb.resize(nBytes);
	receiveBytes(newsockfd,vb.data(),nBytes);//recebe a imagem compactada e coloca no vetor vb
	//image = cv::imdecode(vb,1);//descompacta a imagem
	//cv::imwrite(sImage + filename,image,param);
	ofstream outfileImg ((sImage + filename).c_str(),ofstream::binary);
	outfileImg.write ((char*)vb.data(),nBytes);
	outfileImg.close();
	cout << "recebeu imagem de " << portno << endl;

	sendInt(newsockfd,1); // confirmacao
	
	// RECEBE ARQUIVO .FEAT E SALVA COMO TEST_no.feat
	nBytes = receiveInt(newsockfd);
	vb_feat.resize(nBytes);
	receiveBytes(newsockfd,vb_feat.data(),nBytes);   
	ofstream outfilefeat ((sFeat_Desc + filename + ".feat").c_str(),ofstream::binary);
	//ofstream outfilefeat ((sFeat_Desc + "filefeat"+num_port+".feat").c_str(),ofstream::binary);
	outfilefeat.write ((char*)vb_feat.data(),nBytes);
	outfilefeat.close();
	cout << "recebeu .feat de " << portno << endl;

	sendInt(newsockfd,1); // confirmacao
	
	// RECEBE ARQUIVO .DESC E SALVA COMO TEST_no.desc
	nBytes = receiveInt(newsockfd);
	vb_desc.resize(nBytes);
	receiveBytes(newsockfd,vb_desc.data(),nBytes);    
	ofstream outfiledesc ((sFeat_Desc + filename+".desc").c_str(),ofstream::binary);
	outfiledesc.write ((char*)vb_desc.data(),nBytes);
	outfiledesc.close();
	cout << "recebeu .desc de " << portno << endl;
	
	close(sockfd);
	
	pthread_exit(NULL);
}

// ==================================================================

int main(int argc, char **argv)
{
	// ATENCAO!!!!!!
	//
	// RECEBER PARAMETROS WIDTH (int img_width) E HEIGHT (img_height) DA IMAGEM
	cout << "\n(1) DETECCAO DISTRIBUIDA DE FEATURES\n" << endl;

	if(argc!=4) {
		fprintf(stderr,"usage %s hostname\n", argv[0]);
        exit(0);
	}

	// PORTS
	int ports1[16] = {2000,2200,2400,2600,2800,3000,3200,3400,3600,3800,4000,4200,4400,4600,4800,5000};
	int ports2[16] = {2010,2210,2410,2610,2810,3010,3210,3410,3610,3810,4010,4210,4410,4610,4810,5010};
	int ports3[16] = {2020,2220,2420,2620,2820,3020,3220,3420,3620,3820,4020,4220,4420,4620,4820,5020}; // THIS TO RECEIVE IMAGE AND FEATURES

	int noPorts = atoi(argv[1]);

	// Resolution detection
	// 0 - NORMAL
	// 1 - HIGH
	// 2 - ULTRA
	int resolucao_deteccao = atoi(argv[2]);
	if(resolucao_deteccao<0 || resolucao_deteccao>2){
		cerr << "Resolucao fora do intervalo" << endl;
    	EXIT_FAILURE;
	}
	int teste = atoi(argv[3]);
	if(!(teste==0||teste==1)) {
		cerr << "Erro na opcao teste" << endl;
    	EXIT_FAILURE;
	}
	// ========================================================
	
	pthread_t threads_send_commands[noPorts];
	pthread_t threads_send_take_picture[noPorts];
	pthread_t threads_receive_image_feat_desc[noPorts];
	int rc1, rc2, rc3;
	
	// Initialize all connections to send commands
	for(int i = 0; i < noPorts; i++){
		ports1[i] += resolucao_deteccao;
		ports1[i] += teste*10;
		rc1 = pthread_create(&threads_send_commands[i], NULL, connect_send_commands, (void *)&ports1[i]);
	}    
	// Wait all connections close
	for (int j = 0; j < noPorts; j++){
		pthread_join(threads_send_commands[j], NULL);
	}
	cout << "teminou parte 1 (comando recebido)" << endl
		 << "----------------------------------" << endl << endl;
	
	
	// Initialize all connections to send command take picture
	for(int i = 0; i < noPorts; i++){
		rc2 = pthread_create(&threads_send_take_picture[i], NULL, connect_send_command_take_picture, (void *)&ports2[i]);
	}
	// Wait all connections close
	for (int j = 0; j < noPorts; j++){
		pthread_join(threads_send_take_picture[j], NULL);
	}
	cout << "teminou parte 2 (tirou foto)" << endl
		 << "----------------------------------" << endl << endl;

	// Initialize all connections to receive image and .feat .desc
	for(int i = 0; i < noPorts; i++){
		rc3 = pthread_create(&threads_receive_image_feat_desc[i], NULL, connect_receive_image_feat_desc, (void *)&ports3[i]);
	}    
	// Wait all connections close
	for (int j = 0; j < noPorts; j++){
		pthread_join(threads_receive_image_feat_desc[j], NULL);
	}
	cout << "terminou parte 3 (recebeu arquivos)" << endl
		 << "----------------------------------" << endl << endl;
	
	return EXIT_SUCCESS;
    
}
