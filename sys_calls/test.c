#include <stdio.h>
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sys/time.h>

int main(int argc, char *argv[]){
   struct timeval time;
   
   gettimeofday( &time, 0 );
   long start_t = 1000000 * time.tv_sec + time.tv_usec;
   long int amma;
   for(int i=0;i<1000;++i)
	 amma=3;//syscall(354);
   gettimeofday( &time, 0 );
   printf("System call sys_hello returned %ld\n", amma);
   long end_t = 1000000 * time.tv_sec + time.tv_usec;
     // Read the response from the LKM
   printf("time it takes in us: %ld\n",end_t-start_t);
   
   printf("End of the program\n");
   return 0;
}
