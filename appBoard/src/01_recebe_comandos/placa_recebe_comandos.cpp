#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

//using namespace cv;
using namespace std;

#include <iostream>
#include <string>
#include <utility>
#include <sys/time.h>

/// FUNCOES DE RECEBE/ENVIA ALGO INT E BYTE
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

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void conect(int *sockfd, int portno, struct hostent *server){
	
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

    cout << " . . . aguardando" << endl;
    while(connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){}    
    return;
}


int main(int argc, char *argv[]) {

	cout << "\n(1) RECEBER COMANDOS\n";
	int tipo_detector;
	int resolucao_deteccao;
	
	int sockfd;
    struct hostent *server;
    
    if (argc < 3) {
        fprintf(stderr,"usage %s hostname\n", argv[0]);
        exit(0);
    }
    
    int portno = atoi(argv[2]); // numero da porta (DEFINIR PARA CADA PLACA)
    server =  gethostbyname(argv[1]);// ip server
	// ==========================================================
	// RECEBE COMANDOS DO SERVER
	// ==========================================================
    conect(&sockfd, portno, server);
    tipo_detector = receiveInt(sockfd);
    resolucao_deteccao = receiveInt(sockfd);
	sendInt(sockfd,1); // confirmar o recebimento do comandos
	close(sockfd);
	cout << " teste? " << tipo_detector << endl
		 << " resolucao deteccao: " << resolucao_deteccao << endl; 
	
    // SALVA COMANDO EM FILE
	ofstream myfile ("com1.txt");
	if (myfile.is_open())
	{
		myfile << resolucao_deteccao;
		myfile.close();
	}
	
	// TESTE OU NAO
	if(tipo_detector==1) {
		ofstream myfile ("com2.txt");
		if (myfile.is_open())
		{
			myfile << 6;
			myfile.close();
		}
	}

    return EXIT_SUCCESS;
}

