/**
 * @file   testebbchar.c
 * @author Derek Molloy
 * @date   7 April 2015
 * @version 0.1
 * @brief  A Linux user space program that communicates with the ebbchar.c LKM. It passes a
 * string to the LKM and reads the response from the LKM. For this example to work the device
 * must be called /dev/ebbchar.
 * @see http://www.derekmolloy.ie/ for a full description and follow-up descriptions.
*/
#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
#include<time.h>
#define BUFFER_LENGTH 256               ///< The buffer length (crude but fine)
static char receive[BUFFER_LENGTH];     ///< The receive buffer from the LKM

int main(){
   struct timeval time;
   
   

   int ret, fd;
   char stringToSend[BUFFER_LENGTH];
   printf("Starting device test code example...\n");
   fd = open("/dev/ebbchar", O_RDWR);             // Open the device with read/write access
   if (fd < 0){
      perror("Failed to open the device...");
      return errno;
   }
   printf("Type in a short string to send to the kernel module:\n");
   scanf("%[^\n]%*c", stringToSend);                // Read in a string (with spaces)
   printf("Writing message to the device [%s].\n", stringToSend);
   gettimeofday( &time, 0 );
   long start_t = 1000000 * time.tv_sec + time.tv_usec;
   ret = write(fd, stringToSend, strlen(stringToSend)); // Send the string to the LKM
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
