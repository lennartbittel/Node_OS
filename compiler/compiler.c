#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>
#include<time.h>
#define BUFFER_LENGTH 100
#define Integer 0
#define Qubit 1
#define Person 2
struct var
	{
	int type; 
	char *name;
	}
#define var_len 1000 
struct var var_lis[var_len];
int find_num(*char name)
{
	for(int i=0; i<var_len;++i)
		if(strcmp(var_lis[i],input)
}
int current_line=0;

int main(int argc, char* argv[])
{
	char rec[BUFFER_LENGTH];
	FILE *code = fopen(argv[1], "r");
    	if (code == NULL) {
        	printf("can open file %s",argv[1]);
        	return 0;
	FILE *res_file;
	freopen(argv[2], "wb", fptr);
    	size_t len = 255;
    	char *line = malloc(sizeof(char) * len);

    	}
    	while(fgets(line, len, code) != NULL) {

        	printf("line %d: %s\n",current_line, line);
		do_task
		++current_line;
    	}
    	free(line);
	
}

