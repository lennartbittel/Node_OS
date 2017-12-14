#include <stdio.h>
#include <stdlib.h>
#include<unistd.h>
#include<errno.h>
#include<fcntl.h>
#include<time.h>
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
#define cmd_yrot		15	/* Rotation over angle around Y in pi/256 increments */
#define cmd_zrot		16	/* Rotation over angle around Z in pi/256 increments */
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
			int prio;
			int cur;
			int icr;
			int max_q;
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

int init_sys(qsys *self)
{
	self->c_res=malloc(sizeof(int *)*100);
	//self->q_var=malloc(sizeof(int *)*10);
	//self->peop=malloc(sizeof(person *)*10);
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
int apply_2gate(qsys *self,int task,int qubit1,int qubit2)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=task;
	self->operations[self->cur].first=qubit1;
	self->operations[self->cur].second=qubit2;
	return self->operations[(self->cur)++].first;
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
int perform_sys(qsys *self,int prio)
{
	if(self->val)
		printf("ERR: there are qubits which are still in use\n");
	for(int i=0;i< self->cur; ++i)
		{
		printf("task %d: %d,first:%d,second: %d\n",i,self->operations[i].task,self->operations[i].first,self->operations[i].second);
	printf("amount of steps:%d,amount of final vals:%d,max_qubits:%d\n",self->cur,self->icr,self->max_q);
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
	ret = write(fd, self->str, sizeof(self->str));
	if (ret < 0){
      		perror("Failed to write the message to the device.");
      		return errno;
		}
	while(1)
	{
		sleep(1);
		printf("sleeping over\n");
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
			for(int i=0;i< self->icr; ++i)
				*(self->c_res[i])=ms.results[i];
			return 0;
			}
	}

}
int main()
{
	srand(time(NULL));
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
		printf("qubit id: %d\n",q1);
		printBits(8,&(sys.val));
		apply_gate(&sys,cmd_hgate,q1);
		meas_q(&sys,q1,&res[k],remove);
		
	}
/*	for(int lauf=0;lauf<rep;++lauf)
	{
		int * q1= create_q(sys);
		int * q2= create_q(sys);
		apply_gate(sys,H,q1);
		apply_2gate(sys,CNOT,q1,q2);
		send_q(sys,q2,bob);
		if(sys,basis[lauf])
			apply_gate(sys,q1,H);
		meas_q(sys,q1,&res[rep],remove);		
	}
*/
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
	printf("special: %d\n",num%9+1);
	return 0;
}
