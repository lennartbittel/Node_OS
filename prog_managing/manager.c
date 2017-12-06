#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
#include<time.h>
#define BUFFER_LENGTH 256               ///< The buffer length (crude but fine)
static char receive[BUFFER_LENGTH];     ///< The receive buffer from the LKM
typedef struct {
	int task;
	int first;
	int second;
	int third;
}command;
union Get_prog{
	struct{
		char name[100];
		int prio;
		int size;
		command cmds[100];
		}__attribute__((__packed__));
	char str[1708];
	};

union Get_prog *read_in(char *file_name,int prio)
{
	union Get_prog *prog=malloc(sizeof(union Get_prog));
	FILE *file;
	file = fopen(file_name, "r");
	char c[100];
	char * cur;
	if (file) 
		{
		int i=0;
		printf("file opend: %s\n",file_name);
		strcpy(prog->name,file_name);
		prog->prio=prio;	
		while(fgets(c,100,file)!=NULL)
			{
			  
			  if(c[0]=='%') continue;
			  printf("round: %d  ",i);
 			  ++i;				
			  cur = strtok (c," ");
			  prog->cmds[i-1].task=atoi(cur);
		 	  printf("task: %d\n",prog->cmds[i-1].task);
			  cur = strtok (NULL, " ,.-");if(cur==0) continue;
			  
			  prog->cmds[i-1].first=atoi(cur);cur = strtok (NULL, " ,.-");if(cur==0) continue;
			  prog->cmds[i-1].second=atoi(cur);cur = strtok (NULL, " ,.-");if(cur==0) continue;
			  prog->cmds[i-1].third=atoi(cur);cur = strtok (NULL, " ,.-");if(cur==0) continue;
			  
			}
		prog->size=i;
		fclose(file);
		}
	printf("done reading in\n");
	return prog;
}

int main(int argc, char *argv[]){
   struct timeval time;
   union Get_prog *prog=read_in(argv[1],10);
   printf("%d,%d",prog->prio,prog->str[100]);
   printf("size: %zu\n",sizeof(union Get_prog));
   int ret, fd;
   char stringToSend[BUFFER_LENGTH];
   printf("Starting device test code example...\n");
   fd = open("/dev/nodeos", O_RDWR);             // Open the device with read/write access
   if (fd < 0){
      perror("Failed to open the device...");
      return errno;
   }
   /*printf("Type in a short string to send to the kernel module:\n");
   scanf("%[^\n]%*c", stringToSend);                // Read in a string (with spaces)
   printf("Writing message to the device [%s].\n", stringToSend);*/
   gettimeofday( &time, 0 );
   long start_t = 1000000 * time.tv_sec + time.tv_usec;
   ret = write(fd, prog->str, sizeof(prog->str)); // Send the string to the LKM
   if (ret < 0){
      perror("Failed to write the message to the device.");
      return errno;
   }
   gettimeofday( &time, 0 );
   long inter_t = 1000000 * time.tv_sec + time.tv_usec;
   
   ret = read(fd, receive, BUFFER_LENGTH);  
   gettimeofday( &time, 0 );
   long end_t = 1000000 * time.tv_sec + time.tv_usec;
   printf("Reading from the device...\n");      // Read the response from the LKM
   printf("time it takes in us: %d= %d+ %d\n",end_t-start_t,inter_t-start_t,end_t-inter_t);
   if (ret < 0){
      perror("Failed to read the message from the device.");
      return errno;
   }
   printf("The received message is: [%s]\n", receive);
   printf("End of the program\n");
   return 0;
}
