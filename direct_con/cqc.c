#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>

// #include <stdlib.h>

#include <netinet/in.h>
#include <netdb.h>

#include <string.h>

#include "cqc.h"

#include <fcntl.h>
#include <sys/stat.h>


/*
 * cqc_init
 *
 * Initialize the CQC Backend.
 *
 * Arguments:
 * app_id	ID to use for this application
 */

cqc_lib *
cqc_init(int app_id)
{
	cqc_lib *cqc;

	/* Initialize CQC Data structure */
	cqc = (cqc_lib *)malloc(sizeof(cqc_lib));
	bzero((char *) cqc, sizeof(cqc_lib));
	cqc->app_id = app_id;

	return(cqc);
}

/* 
 * cqc_error
 *
 * Print the appropriate error message for the error code received.
 */
void
cqc_error(uint8_t type)
{
	switch(type) {
		case CQC_ERR_GENERAL:
			fprintf(stderr,"CQC ERROR: General error.\n");
			break;
		case CQC_ERR_NOQUBIT:
			fprintf(stderr,"CQC ERROR: No qubit with the specified ID for this application.\n");
			break;
		case CQC_ERR_UNSUPP:
			fprintf(stderr,"CQC ERROR: Command not supported by implementation.\n");
			break;
		case CQC_ERR_TIMEOUT:
			fprintf(stderr,"CQC ERROR: Timeout.\n");
			break;
		case CQC_ERR_INUSE:
			fprintf(stderr,"CQC ERROR: Qubit ID already in use.\n");
			break;
		default:
			fprintf(stderr,"CQC ERROR: Unknown error type.\n");
	}
	return;
}

/* 
 * cqc_connect
 *
 * Connect to CQC Backend (if necessary).
 *
 * Arguments:
 * hostname 	hostname to connect to
 * portno	port number to connect to
 */

int
cqc_connect(cqc_lib *cqc, char *hostname, int portno)
{
	int sock;
	struct sockaddr_in serv_addr;
	struct hostent *server;

	/* Set up connection details */	
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
                perror("ERROR opening socket");
		return(-1);
        }
        
        server = gethostbyname(hostname);
        if (server == NULL) {
                fprintf(stderr,"ERROR, no such host\n");
		return(-1);
        }

	bzero((char *) &serv_addr, sizeof(serv_addr));
   	serv_addr.sin_family = AF_INET;
   	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
   	serv_addr.sin_port = htons(portno);

   	/* Now connect to the server */
   	if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
      		perror("ERROR - Cannot connect");
		return(-1);
   	}
	cqc->sockfd = sock;
	return(0);
}

/* 
 * cqc_cleanup
 *
 * Tear down the connection to the CQC Backend (if any) and free the memory of the cqc structure.
 */

int 
cqc_cleanup(cqc_lib *cqc)
{	

	close(cqc->sockfd);
	free(cqc);

	return(0);
}

/*
 * cqc_simple_cmd
 *
 * Executes a simple CQC command (not requiring any additional details)
 *
 * Arguments:
 * command	command identifier to be sent
 * qubit_id	identifier of qubit on which to perform this command
 * notify	whether to request a DONE upon completion (0 = no, 1 = yes)
 */
	
int 
cqc_simple_cmd(cqc_lib *cqc, uint8_t command, uint16_t qubit_id, uint8_t notify)
{
	int n;
	cqcHeader cqcH;
	cmdHeader cmdH;

	/* Prepare CQC message indicating a command */
	cqcH.version = CQC_VERSION;
	cqcH.app_id = cqc->app_id;
	cqcH.type = CQC_TP_COMMAND;
	cqcH.length = CQC_CMD_HDR_LENGTH;
   
   	/* Send message to the server */
   	n = write(cqc->sockfd, &cqcH, CQC_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
		return(-1);
   	}

	/* Prepare message for the specific command */
	bzero(&cmdH, sizeof(cmdH));
	cmdH.qubit_id = qubit_id;
	cmdH.instr = command;
	if(notify == 1) {
		cmdH.options = CQC_OPT_NOTIFY | CQC_OPT_BLOCK;
	} 

   	/* Send message to the server */
   	n = write(cqc->sockfd, &cmdH, CQC_CMD_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
      		return(-1);
   	}

	return(n);
}

/* 
 * cqc_full_cmd
 *
 * Executes a full CQC command incl extra header
 *
 * Arguments:
 * command	command identifier
 * qubit_id	qubit to execute on
 * notify	indication whether notification is requested (0/1)
 * action	indication whether further actions should be taken following this command (0/1, requires further commands to be sent)
 * block	indication whether further execution should be suspended while executing this command (0/1)
 * xtra_id	id of additional qubit
 * steps	number of steps to rotate or repetitions to perform
 * r_app_id	remote application id
 * r_node	remote node (IP)
 * r_port	remote node port (for classical control info)
 * cmdLength	length of extra commands to be sent
 */

int
cqc_full_cmd(cqc_lib *cqc, uint8_t command, uint16_t qubit_id, char notify, char action, char block, uint16_t xtra_id, uint8_t steps, uint16_t r_app_id, uint32_t r_node, uint16_t r_port, uint32_t cmdLength)
{
	int n;
	cqcHeader cqcH;
	cmdHeader cmdH;
	xtraCmdHeader xtra;

	/* Prepare CQC message indicating a command */
	cqcH.version = CQC_VERSION;
	cqcH.app_id = cqc->app_id;
	cqcH.type = CQC_TP_COMMAND;
	cqcH.length = CQC_CMD_HDR_LENGTH + CQC_CMD_XTRA_LENGTH;
   
   	/* Send message to the server */
   	n = write(cqc->sockfd, &cqcH, CQC_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
		return(-1);
   	}

	/* Prepare message for the specific command */
	bzero(&cmdH, sizeof(cmdH));
	cmdH.qubit_id = qubit_id;
	cmdH.instr = command;

	cmdH.options = 0;
	if(notify == 1) {
		cmdH.options = cmdH.options | CQC_OPT_NOTIFY;
	}
	if(action == 1) {
		cmdH.options = cmdH.options | CQC_OPT_ACTION;
	}
	if(block == 1) {
		cmdH.options = cmdH.options | CQC_OPT_BLOCK;
	}
   	/* Send message to the server */
   	n = write(cqc->sockfd, &cmdH, CQC_CMD_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
      		return(-1);
   	}

	/* Prepare extra header */
	bzero(&xtra, sizeof(xtra));
	xtra.xtra_qubit_id = xtra_id;
	xtra.steps = steps;
	xtra.remote_app_id = r_app_id;
	xtra.remote_node = r_node;
	xtra.remote_port = r_port;
	xtra.cmdLength = cmdLength;
   	/* Send message to the server */
   	n = write(cqc->sockfd, &xtra, CQC_CMD_XTRA_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
      		return(-1);
   	}

	return(n);
}

/*
 * cqc_hello
 *
 * Sends a HELLO message to the CQC Backend.
 */

int
cqc_hello(cqc_lib *cqc)
{
	int n;
	cqcHeader cqcH;

	/* Prepare CQC message indicating a command */
	cqcH.version = CQC_VERSION;
	cqcH.app_id = cqc->app_id;
	cqcH.type = CQC_TP_HELLO;
	cqcH.length = CQC_CMD_HDR_LENGTH;
   
   	/* Send message to the server */
   	n = write(cqc->sockfd, &cqcH, CQC_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR writing to socket");
		return(-1);
   	}
	return(n);
}

/*
 * cqc_send
 *
 * Request the qubit to send to remote node.
 *
 * Arguments:
 * qubit_id		qubit to send
 * remote_app_id  	app id on the remote node to send to
 * remote_node		address of remote node (IPv6)
 * remote_port		port for classical control info
 */

int
cqc_send(cqc_lib *cqc, uint16_t qubit_id, uint16_t remote_app_id, uint32_t remote_node, uint16_t remote_port)
{
	return(cqc_full_cmd(cqc, CQC_CMD_SEND, qubit_id, 1, 0, 1, 0, 0, remote_app_id, remote_node, remote_port, 0));
}

/* 
 * cqc_recv
 *
 * Request to receive a qubit. 
 *
 * Arguments:
 * qubit_id		id to assign to this qubit once it is received
 */
int
cqc_recv(cqc_lib *cqc, uint16_t qubit_id)
{
	int n;
	cqcHeader reply;
	notifyHeader note;

	/* Send out request to receive a qubit */
	n = cqc_simple_cmd(cqc, CQC_CMD_RECV, qubit_id, 0);
	if (n < 0) {
		perror("ERROR - Cannot send receive request");
		return(-1);
	}

  	/* Now read CQC Header from server response */
   	bzero(&reply, sizeof(reply));
   	n = read(cqc->sockfd, &reply, CQC_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR - cannot get reply header");
      		return(-1);
   	}	
	if(reply.type != CQC_TP_RECV) {
		fprintf(stderr,"ERROR: Expected RECV");
		return(-1);
	}

	/* Then read qubit id from notify header */
   	bzero(&note, sizeof(note));
   	n = read(cqc->sockfd, &note, CQC_NOTIFY_LENGTH);
   	if (n < 0) {
      		perror("ERROR - cannot get measurement outcome");
      		return(-1);
   	}	

	return(note.qubit_id);
}
	
/* 
 * cqc_epr
 *
 * Request to generate EPR pair with remote node.
 *
 * Arguments:
 * remote_app_id	app id on the remote node to send to
 * remote_node		address of remote node to receive from (IPv6)
 * remote_port		port for classical control info
 */
int
cqc_epr(cqc_lib *cqc, uint16_t remote_app_id, uint32_t remote_node, uint16_t remote_port)
{
	return(cqc_full_cmd(cqc, CQC_CMD_RECV, 1, 0, 0, 1, 0, 0, remote_app_id, remote_node, remote_port, 0));
}	

/*
 * cqc_measure
 *
 * Request to measure a specific qubit. This will block until the reply is received. 
 * (Non blocking measure requests can be performed using cqc_simple_cmd)
 *
 * Arguments:
 * qubit_id		qubit to measure
 */

int
cqc_measure(cqc_lib *cqc, uint16_t qubit_id)
{
	int n;
	cqcHeader reply;
	notifyHeader note;

	n = cqc_simple_cmd(cqc, CQC_CMD_MEASURE, qubit_id, 1);
	if (n < 0) {
		perror("ERROR - measurement failed");
		return(-1);
	}

  	/* Now read CQC Header from server response */
   	bzero(&reply, sizeof(reply));
   	n = read(cqc->sockfd, &reply, CQC_HDR_LENGTH);
   	if (n < 0) {
      		perror("ERROR - cannot get reply header");
      		return(-1);
   	}	
	if(reply.type != CQC_TP_MEASOUT) {
		fprintf(stderr,"ERROR: Expected MEASOUT");
		return(-1);
	}

	/* Then read measurement outcome from notify header */
   	bzero(&note, sizeof(note));
   	n = read(cqc->sockfd, &note, CQC_NOTIFY_LENGTH);
   	if (n < 0) {
      		perror("ERROR - cannot get measurement outcome");
      		return(-1);
   	}	

	return(note.outcome);
}

/* 
 * cqc_wait_until_done
 *
 * Read a certain number of DONE commands before proceeding.
 *
 * Arguments:
 * reps	number of replies to wait for
 */
int
cqc_wait_until_done(cqc_lib *cqc, unsigned int reps)
{
	int i, n;
	cqcHeader reply;

	for(i = 0; i < reps; i++) {
  		/* Now read CQC Header from server response */
   		bzero(&reply, sizeof(reply));
   		n = read(cqc->sockfd, &reply, CQC_HDR_LENGTH);
   		if (n < 0) {
      			perror("ERROR - cannot get reply header");
      			return(-1);
   		}	

		/* Check whether an error occured */
		if(reply.type >= CQC_ERR_GENERAL) {
			cqc_error(reply.type);
			return(-1);
		}

		/* Otherwise check whether it's done */
		if(reply.type != CQC_TP_DONE) {
			fprintf(stderr,"Unexpected reply of type %d\n",reply.type);
			return(-1);
		}
	}
	return(0);
}

/*
 *  cqc_twoqubit
 * 
 *  Execute local two qubit gate.
 *
 *  Arguments:
 *  command     command id to execute
 *  qubit1      number of the first qubit
 *  qubit2	number of the second qubit
 */

int
cqc_twoqubit(cqc_lib *cqc, uint8_t command, uint16_t qubit1, uint16_t qubit2)
{
	return(cqc_full_cmd(cqc, command, qubit1, 1, 0, 1, qubit2, 0, 0, 0, 0, 0));
}

/*Classical communication*/

int rec_classical(char *myfifo,int *outp,int MAX_BUF)
{    
	/*int MAX_BUF=1024;*/
	int fd;
	/*char * myfifo = "/tmp/myfifo";*/
	char buf[MAX_BUF];

	    /* open, read, and display the message from the FIFO */
	fd = open(myfifo, O_RDONLY);
	while(outp[MAX_BUF]!=1 && outp[MAX_BUF]!=0)
	{
	read(fd, outp, MAX_BUF);
	printf("%d %d %d \n",outp[0],outp[10],outp[19]);
	sleep(1);
	}
	close(fd);
	return 0;
}
int send_classical(char *myfifo ,int *ms,int length)
{
	int fd;
	printf("ms:%d",ms[2]);
    	/*char * myfifo = "/tmp/myfifo";*/

    	/* create the FIFO (named pipe) */
    	mkfifo(myfifo, 0666);

    	/* write "Hi" to the FIFO */
    	fd = open(myfifo, O_WRONLY);
    	write(fd, ms,length);
    	close(fd);

    	/* remove the FIFO */
    	unlink(myfifo);
	return 0;
}


/*NEXT ATTEMPT */
#include<arpa/inet.h>
#include<string.h>
int send_cl(char* message,int length,char * server_reply,int portn)
{
    int sock;
    struct sockaddr_in server;
    //char server_reply[2000];
    //printf("message: %s,first letter %c\n",message,message[0]);
    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
     
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( portn );
 
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }
     
    puts("Connected\n");
     
    //keep communicating with server
	int x=1;
    while(x==1)
    {
        x=0; 
        //Send some data
        if( send(sock , message , strlen(message) , 0) < 0)
        {
            puts("Send failed");
            return 1;
        }
         
        //Receive a reply from the server
        if( recv(sock , server_reply , 2000 , 0) < 0)
        {
            puts("recv failed");
            break;
        }
         
        puts("Server reply :");
        puts(server_reply);
	
    }
     
    close(sock);
    return 0;
}


int rec_cl(char* message,int length,char * outp,int portn)
{
    int socket_desc , client_sock , c , read_size;
    struct sockaddr_in server , client;
    char client_message[2000];
     
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
     
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(portn);
     
    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");
     
    //Listen
    listen(socket_desc , 3);
     
    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
     
    //accept connection from an incoming client
    client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
    if (client_sock < 0)
    {
        perror("accept failed");
        return 1;
    }
    puts("Connection accepted");
     
    //Receive a message from client
    while( (read_size = recv(client_sock , outp , length , 0)) > 0 )
    {
        //Send the message back to client
        write(client_sock , message , length);
	break;
    }
     
    if(read_size == 0)
    {
        puts("Client disconnected");
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }
     
    return 0;
}

