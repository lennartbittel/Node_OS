#include <stdio.h>
#include <stdlib.h>
#include<unistd.h>
#include<errno.h>
#include<fcntl.h>
#include<time.h>
#include <math.h>
#define cmd_identity		0	/* Identity (do nothing, wait one step) */
#define	cmd_new_qubit		1	/* Ask for a new qubit */
#define cmd_meas_rm		2	/* Measure qubit */
#define cmd_meas		3	/* Measure qubit inplace */
#define cmd_reset		4	/* Reset qubit to |0> */
#define cmd_sendq		5	/* Send qubit to another node */
#define cmd_recvq		6	/* Ask to receive qubit */
#define cmd_eprq		7	/* Create EPR pair with the specified node */

#define cmd_xgate		10	/* Pauli X */
#define cmd_zgate		11	/* Pauli Z */
#define cmd_ygate		12	/* Pauli Y */
#define cmd_tgate		13	/* T Gate */
#define cmd_xrot		14	/* Rotation over angle around X in pi/256 increments */
//This is the wrong way. But also in CQC!!
#define cmd_zrot		15	/* Rotation over angle around Y in pi/256 increments */
#define cmd_yrot		16	/* Rotation over angle around Z in pi/256 increments */
//end wrong
#define cmd_hgate		17	/* Hadamard Gate */
#define cmd_kgate		18	/* K Gate - taking computational to Y eigenbasis */

#define cmd_cnot		20	/* CNOT Gate with this as control */
#define cmd_cphase		21	/* CPHASE Gate with this as control */

#define cmd_if			100
#define cmd_goto		101

#define cmd_sendc		201 	//person_name, var
#define cmd_recvc		202	//person_name, var 

#define cmd_done		1000

void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}
#define keep 0
#define remove 1
long long binom(int n,int k)
{
    long long ans=1;
    k=k>n-k?n-k:k;
    
    for(int j=1;j<=k;j++,n--)
    {
        if(n%j==0)
        {
            ans*=n/j;
        }else
        if(ans%j==0)
        {
            ans=ans/j*n;
        }else
        {
            ans=(ans*n)/j;
        }
    }
    return ans;
}
int *get_num(int m,int k,int num)
{
	int *nums;
	if(k==0)
		{
		nums=malloc(10*sizeof(int));
		return nums;
		}
	for(int i=m;i>=k;--i)
		{
		if(binom(i-1,k-1)>num)
			{
			nums=get_num(i-1,k-1,num);
			nums[k-1]=i;
			return nums;
			}
		num-=binom(i-1,k-1);
		}
	return NULL;
}
//printBits(sizeof(val),&val);
int put_q(unsigned long *val)
{	
	for(int i=0; i<64;++i)
		if(!(*val & (1<<i)))
		{
			*val+=(1<<i);
			return i;
		}
	return -1;
}
int remove_q(unsigned long *val,int i)
{
	if(!(*val & (1<<i)))
		printf("qubit does not exist\n");
	*val-=(1<<i);
	return 0;
}

typedef struct {
char ip[4];
int name;
}person;
typedef struct{
int task;
int first;
int second;
int third;
}ops;
typedef struct {
	union {
		struct{
			char name[100];
			int np;
			int prio;
			int cur;
			int icr;
			int max_q;
			int prot_id;
			person peop[10];
			ops operations[400];
		}__attribute__((__packed__));
		char str[100+4*4+8*10+400*16];
		};
	int **c_res;
	unsigned long val;
} qsys;
int find_item(void **lis,void *item,int len)
{
	for(int k=0; k<len;++k)
		if(lis[k]=item)
			return k;
	return -1;
}

int init_sys(qsys *self,int np,int prot_id)
{
	self->c_res=malloc(sizeof(int *)*100);
	//self->q_var=malloc(sizeof(int *)*10);
	//self->peop=malloc(sizeof(person *)*10);
	self->prot_id=prot_id;
	self->np=np;
	self->cur=0;
	self->icr=0;
	self->val=0;
	self->max_q=0;
	//self->operations=malloc(sizeof(ops)*60);
	return 0;
}
int create_q(qsys *self)
{
	int i=put_q(&(self->val));
	if((i+1)>self->max_q)
		self->max_q=i+1;
	self->operations[self->cur].task=cmd_new_qubit;
	self->operations[self->cur].first=i;
	(self->cur)++;
	return i;
}
int apply_gate(qsys *self,int task,int qubit)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=task;
	self->operations[self->cur].first=qubit;
	return self->operations[(self->cur)++].first;
}
int apply_rgate(qsys *self,int task,int qubit,int angle)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=task;
	self->operations[self->cur].first=qubit;
	self->operations[self->cur].second=angle;
	return self->operations[(self->cur)++].first;
}
int apply_2gate(qsys *self,int task,int qubit1,int qubit2)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=task;
	self->operations[self->cur].first=qubit1;
	self->operations[self->cur].second=qubit2;
	return self->operations[(self->cur)++].first;
}
int apply_cpgate(qsys *self,int qubit1,int qubit2,int angle)
{
	apply_rgate(self,cmd_zrot,qubit1,angle/2);
	apply_2gate(self,cmd_cphase,qubit1,qubit2);
	apply_rgate(self,cmd_zrot,qubit1,256-angle/2);
	//self->operations[self->cur]=malloc(sizeof(ops));
	return 0;
}
int send_q(qsys *self,int qubit,int person)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=cmd_sendq;
	self->operations[self->cur].first=qubit;
	self->operations[self->cur].second=person;
	self->operations[self->cur].third=304;
	remove_q(&(self->val),qubit);
	return self->operations[(self->cur)++].first;
}
int recv_q(qsys *self,int person)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	int i=put_q(&(self->val));
	if((i+1)>self->max_q)
		self->max_q=i+1;
	self->operations[self->cur].task=cmd_recvq;
	self->operations[self->cur].first=i;
	self->operations[self->cur].second=person;
	(self->cur)++;
	return i;
}
int meas_q(qsys *self,int qubit,int *resval,int rem)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=3-rem;
	self->operations[self->cur].first=qubit;
	self->operations[self->cur].second=self->icr;
	self->c_res[(self->icr)++]=resval;
	if(rem)
		remove_q(&(self->val),qubit);
	return self->operations[(self->cur)++].first;
}

union return_msg {
struct	{
	int type;
	int results[200];
	};
char str[200*4+4];
};
int perform_sys(qsys *self,int prio,int get_return)
{
	struct timeval time;
	if(self->val)
		printf("ERR: there are qubits which are still in use\n");
	printf("amount of steps:%d,amount of final vals:%d,max_qubits:%d\n",self->cur,self->icr,self->max_q);
	for(int i=0;i< self->cur; ++i)
		{
		printf("task %d: %d,first:%d,second: %d\n",i,self->operations[i].task,self->operations[i].first,self->operations[i].second);
	
		}
	self->prio=prio;
	self->operations[(self->cur)++].task=cmd_done;
	
	int fd,ret;
	fd = open("/dev/nodeos", O_RDWR);
	printf("fd: %d\n",fd);
	if (fd < 0){
      		perror("Failed to open the device...");
      		return errno;
		}
	gettimeofday( &time, 0 );
	ret = write(fd, self->str, sizeof(self->str));
	if (ret < 0){
      		perror("Failed to write the message to the device.");
      		return errno;
		}
	long start_t = 1000000 * time.tv_sec + time.tv_usec;
	while(get_return)
	{
		sleep(1);
		union return_msg ms;
	
		ret = read(fd, ms.str, sizeof(ms.str));  
		if(ret<0)
			{
			printf("ms was not rec");
			return -1;
			}
		if(ms.type==0)
			printf("still running: %d/%d \n",ms.results[0],ms.results[1]);
		if (ms.type==1)
			{
			gettimeofday( &time, 0 );
  	 		long end_t = 1000000 * time.tv_sec + time.tv_usec;
			printf("program was executed. Runtime: %lds ,%ld ms\n",(end_t-start_t)/1000000,((end_t-start_t)/1000)%1000);
			for(int i=0;i< self->icr; ++i)
				*(self->c_res[i])=ms.results[i];
			return 0;
			}
	}

}

int qft(qsys *self,int *q,int n)
{
	
	

	for(int i=0;i<n;++i)
		{
		apply_gate(self,cmd_hgate,q[i]);	
		for(int j=i+1;j<n;++j)
			apply_cpgate(self,q[i],q[j],256/(1<<(j-i+1)));
		}


}
int shor(int number,int n)
{
	printf("start\n");
	for(int m=0;m<5;++m)
	{
		//int n=4;
		int q[n];
		int res[n];
		qsys sys;
		init_sys(&sys,1,313);
		int qf=create_q(&sys);
		apply_gate(&sys,cmd_xgate,qf);
		for(int i=0; i<n;++i)
			{
			q[i]=create_q(&sys);
			apply_gate(&sys,cmd_hgate,q[i]);	
			apply_cpgate(&sys,qf,q[i],((256/number)<<i)%256);
			}
		int a=rand()%(number);
		printf("chosen a:%d\n",a);
		qft(&sys,q,n);
		for(int i=0; i<n;++i)
			{
			meas_q(&sys,q[i],&res[i],remove);
			}
		int result;
		int qfr;
		meas_q(&sys,qf,&qfr,remove);
		result=perform_sys(&sys,10,0);
		for(int k=0;k<n;++k)
			printf("%d, ",res[k]);
		printf("done,%d\n",qfr);
	}
}
/*int test_cphase(int angle)
{
	int rep=15;
	int res[1000];
	qsys sys;
	init_sys(&sys);
	for(int k=0; k<rep;++k)
	{
		int q1= create_q(&sys);
		int q2= create_q(&sys);
		apply_gate(&sys,cmd_xgate,q1);	
		apply_gate(&sys,cmd_xgate,q2);
		apply_rgate(&sys,cmd_yrot,q1,128);
		//apply_cpgate(&sys,q1,q2,angle);
		//printf("qubit id: %d\n",q1);
		//printBits(8,&(sys.val));
		//apply_gate(&sys,cmd_hgate,q2);
		meas_q(&sys,q1,&res[2*k],remove);
		meas_q(&sys,q2,&res[2*k+1],remove);
		
	}
	int result;
	result=perform_sys(&sys,10);
	for(int k=0;k<rep;++k)
		printf("res: %d,%d",res[2*k],res[2*k+1]);
	printf("done\n");
	return 0;
}*/
int main()
{
	srand(time(NULL));
	unsigned int a=rand()%10000;
	a=393;
	
	qsys sys;
	init_sys(&sys,1,a);
	int q=create_q(&sys);
	//send_q(&sys,q,1);
	//int qr=recv_q(&sys,1);
	int res;
	send_q(&sys,q,0);
	int result=perform_sys(&sys,10,0);

	sleep(2);
 	if(0)
{
	init_sys(&sys,0,a);
	int qa=recv_q(&sys,1);
	//send_q(&sys,q,1);
	//int qr=recv_q(&sys,1);
	apply_gate(&sys,cmd_hgate,qa);
	result=perform_sys(&sys,10,1);
	printf("result:%d\n",res);
}
	//shor(15,9);
/*
	printf("start\n");
	int n=4;
	int q[n];
	int res[n];
	qsys sys;
	init_sys(&sys);
	for(int i=0; i<n;++i)
		{
		q[i]=create_q(&sys);
		}
	qft(&sys,q,n);
	for(int i=0; i<n;++i)
		{
		meas_q(&sys,q[i],&res[i],remove);
		}
	int result;
	result=perform_sys(&sys,10);
	for(int k=0;k<n;++k)
		printf("%d, ",res[k]);
	printf("done\n");*/
/*
	
	printf("start:%ld\n",sizeof(unsigned long long));


	int rep=100;
	int i=0;
	int res[rep];
	int basis[rep];
	for(int k=0;k<rep;++k)
	{
		basis[k]=rand()&1;
	}
	qsys sys;
	init_sys(&sys);
	person bob;
	for(int k=0; k<27;++k)
	{
		int q1= create_q(&sys);
		//printf("qubit id: %d\n",q1);
		//printBits(8,&(sys.val));
		apply_gate(&sys,cmd_hgate,q1);
		meas_q(&sys,q1,&res[k],remove);
		
	}

	int result;
	result=perform_sys(&sys,10);
	printf("res:%d\n",result);
	int num=0;
	for(int k=0;k<27;++k)
	{
		printf("%d, ",res[k]);
		num=num*2+res[k];
	}
	
	printf("\nthere are %lld combinations. this gives the numbers:\n",binom(49,6));
	int *nums=get_num(49,6,(num/9)%binom(49,6));
	for(int k=0; k<6;++k)
		printf("%d, ",nums[k]);
	printf("special: %d\n",num%9+1);*/
	return 0;
}
