#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <time.h>
#define  not_started 0
#define  running 1
#define  finished 2
//#include <linux/rbtree.h>
struct Progs{
	char name[100];
	int state;
	int priority;
	int code[1000][10];
	int codevar[10];
	int current; //which line to evaluate next
	int size;
	int breaks;
	int starting_time;
	};

char c[100];
int i=0;
FILE *file;
DIR *d;
struct dirent *dir;
char file_name[100];
char * cur;
#define folder "progs/"
struct Progs prog_lis[100];
int exec_code(int *code)
{
	//printf("code:%d,%d\n",*(code),*(code+1));
	return 0; 
}
int read_in(){
d = opendir(folder);
    if (d)
    {
	dir = readdir(d);dir = readdir(d);
	int num=0;
        while ((dir = readdir(d)) != NULL)
        {	
	    	sprintf(file_name,"%s%s", folder,dir->d_name);
		printf("%s\n", file_name);
		file = fopen(file_name, "r");
		if (file) {
		int i=0;


		strcpy(prog_lis[num].name,dir->d_name);
		prog_lis[num].state=not_started;
		prog_lis[num].current=0;
		prog_lis[num].breaks=0;
		prog_lis[num].starting_time=-1;
		prog_lis[num].priority=4;			


		while(fgets(c,100,file)!=NULL)
			{
			  
			  if(c[0]=='%') continue;
			  cur = strtok (c," ");
			  int pos=0;
			  while (cur != NULL)
			  {
				    prog_lis[num].code[i][pos++]=atoi(cur);
					printf("%d,",prog_lis[num].code[i][pos-1]);
				    cur = strtok (NULL, " ,.-");
			  }
			++i;
   	 		printf("%d:%s\n",i,c);
			}
		fclose(file);
		++num;
        }
	printf("fully done,%d!!\n",prog_lis[0].code[0][0]);
        
    }
    closedir(d);
	return 0;
}

}
#define max_prog 100
#define amount_qubits 10
typedef enum { false, true } bool;
struct OS{
	struct Progs **Running;
	bool used_r[max_prog];
	struct Progs **Sleeping;
	bool used_s[max_prog];
	struct Progs **Not_started;
	bool used_n[max_prog];
	int qubits[amount_qubits];
	int q_live[amount_qubits];
	int g_prio[max_prog]; 
	struct Progs prog_lis[max_prog];
};
int OS_init(struct OS *self){
	self->Running=malloc(max_prog * sizeof(struct Progs *));
	self->Sleeping=malloc(max_prog * sizeof(struct Progs *));
	self->Not_started=malloc(max_prog * sizeof(struct Progs *));
	for(int i=0; i<amount_qubits;++i)
	{
		self->qubits[i]=0;
		self->q_live[i]=-1;
	}
	for(i=0; i<max_prog;++i)
	{
		self->used_r[i]=false;
		self->used_s[i]=false;
		self->used_n[i]=false;
		self->g_prio[i]=-1;
	}
	return 0;
}


int OS_make_prio(struct OS *self)
{
	for(int i=0;i<max_prog;++i)
	{
		if(self->used_r[i])
		{
			self->g_prio[i]=(10-self->Running[i]->priority)*10;
		}
	}
}
int OS_next_p(struct OS *self)
{
	int i=0;
	for(int k=1;k<max_prog;++k)
	{
		if(self->used_r[k])
		{
			if(self->g_prio[k]<self->g_prio[i])//get minimum
				i=k;
		}
	}
	return i;
}
int OS_insert_p(struct OS *self,struct Progs *prog)
{
	for(int i=0; i< max_prog;++i)
	{
		if(!self->used_n[i])
		{
		self->Not_started[i]=prog;
		self->used_n[i]=true;
		return i;
		}
	}
	return -1;
}
int OS_start_p(struct OS *self,int prog_num)
{
	for(int i=0; i< max_prog;++i)
	{
		if(!self->used_r[i])
		{
			self->Running[i]=self->Not_started[prog_num];
			self->used_r[prog_num]=true;
			self->Not_started[prog_num]=NULL;
			self->used_n[prog_num]=false;
			return i;
		}
	}
}
int OS_sleep_p(struct OS *self,int prog_num)
{
	for(int i=0; i< max_prog;++i)
	{
		if(!self->used_s[i])
		{
			self->Sleeping[i]=self->Running[prog_num];
			self->used_s[prog_num]=true;
			self->Running[prog_num]=NULL;
			self->used_r[prog_num]=false;
			return i;
		}
	}
}
int OS_run_p(struct OS *self)
{
	int i=OS_next_p(self);
	struct Progs *prog=self->Running[i];
	int res=exec_code(prog->code[prog->current]);
	++(prog->current);
	++(self->g_prio[i]);
	
}

int main()
{
	struct timeval time;
	read_in();
	struct OS my_os;
	OS_init(&my_os);
	printf("insert program\n");
	
	printf("done\n");
	printf("my_os:%d\n",my_os.Not_started[0]->state);
	OS_insert_p(&my_os,&prog_lis[0]);
	OS_insert_p(&my_os,&prog_lis[1]);
	OS_start_p(&my_os,0);
	OS_start_p(&my_os,1);
	OS_make_prio(&my_os);
	gettimeofday( &time, 0 );
   	long start_t = 1000000 * time.tv_sec + time.tv_usec;
	for(int i=0;i<400;++i)
		OS_run_p(&my_os);
	gettimeofday( &time, 0 );
   	long end_t = 1000000 * time.tv_sec + time.tv_usec;
	printf("400 tasks take %d us\n",end_t-start_t);
	printf("next_prog: %d\n",OS_next_p(&my_os));
	printf("%x,%x\n",&prog_lis[0],my_os.Not_started[0]);
	for(int i=0;i<max_prog;++i)
	{
		printf("%d,",my_os.used_n[i]);
	}
	return 0;
}
