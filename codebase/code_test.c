#include <stdio.h>
#include<time.h>
#include <unistd.h>

#include <stdlib.h>
#define plus 1
#define ifc 2 //first smaller second
#define gotoc 3
#define rep 500000



int main()
{
	int list[100];
	int code[100];
	int x=3;
	int y=4;
	list[0]=4;
	list[1]=3;
	list[2]=0;//i
	list[3]=rep;
	list[4]=1;
	struct timeval time;
	gettimeofday( &time, 0 );
	long start_t = 1000000 * time.tv_sec + time.tv_usec;
	for (int i=0; i<rep;++i)
		y+=x;
	gettimeofday( &time, 0 );
  	long end_t = 1000000 * time.tv_sec + time.tv_usec;
	printf("time: %ld,res:%d\n",end_t-start_t,y);
	/*for(int i=0;i<rep;++i)
	{
		code[3*i]=plus;
		code[3*i+1]=1;
		code[3*i+2]=0;
	}*/
	code[0]=plus;
	code[1]=4;
	code[2]=2;
	code[3]=plus;
	code[4]=1;
	code[5]=0;
	code[6]=ifc;
	code[7]=2;
	code[8]=3;
	code[9]=gotoc;
	code[10]=0;
	code[11]=-1;
	
	int *lauf=code+3-1;
	gettimeofday( &time, 0 );
	start_t = 1000000 * time.tv_sec + time.tv_usec;
	while(1)
	{
		++lauf;
		//printf("%d\n",lauf-code);
		switch (*lauf)
		{
		case plus: list[*(lauf+2)]+=list[*(lauf+1)];lauf+=2; break;
		case ifc: if(list[*(lauf+1)]>=list[*(lauf+2)]) {lauf+=4;break;};lauf+=2;break;
		case gotoc: lauf=code-1+ *(lauf+1); break;
		case -1:  goto end;
		}
	}
end:
	gettimeofday( &time, 0 );
  	end_t = 1000000 * time.tv_sec + time.tv_usec;
	printf("\ntime: %ld,res:%d,%d\n",end_t-start_t,list[0],list[2]);
}
