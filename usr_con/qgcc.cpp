#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdexcept>

#include <iostream>
#include <fstream>
using namespace std;


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
#define cmd_swap		22

#define cmd_if			100	
#define cmd_goto		101
#define cmd_add			102   //third:0 two variables, third:1 add int to variable,third:2 set variable to int
#define cmd_multi		104
#define cmd_setval		105

#define cmd_sendc		201 	//var,person_name
#define cmd_recvc		202	//var,person_name

#define cmd_done		1000

//types
#define one_qubit_op
#define two_qubit_op


/*
class operations{
	public:
	int type;//qm, classical network
	char qasm_name[];
	int low_level[4];
	int time;
	intc *q0;
	intc *q1;
	int angle;
};
*/

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
		//nums=malloc(10*sizeof(int));
		nums=(int*) malloc(10);
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
			//person peop[10];
			ops operations[400];
		}__attribute__((__packed__));
		char str[100+4*4+400*16];
		};
	int **c_res;
	unsigned long val;
	unsigned long valc;
} qsys;
int find_item(int **lis,int *item,int len)
{
	for(int k=0; k<len;++k)
		if(lis[k]==item)
			return k;
	return -1;
}

int init_sys(qsys *self,int np,int prot_id)
{
	//self->c_res=malloc(sizeof(int *)*100);
	self->c_res=(int **) malloc(100);
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
int add_inst(qsys *self,int task,int first,int second,int third)
{
	self->operations[self->cur].task=task;
	self->operations[self->cur].first=first;
	self->operations[self->cur].second=second;
	self->operations[self->cur].third=third;
	return self->operations[(self->cur)++].first;
}
int create_q(qsys *self,int with_cmd=1)
{
	int i=put_q(&(self->val));
	if((i+1)>self->max_q)
		self->max_q=i+1;
	if(with_cmd)
	{
		self->operations[self->cur].task=cmd_new_qubit;
		self->operations[self->cur].first=i;
		(self->cur)++;
	}
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
int recv_q(qsys *self,int q_id,int person)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=cmd_recvq;
	self->operations[self->cur].first=q_id;
	self->operations[self->cur].second=person;
	(self->cur)++;
	return q_id;
}
int epr_q(qsys *self,int q_id,int person)
{

	add_inst(self,cmd_eprq,q_id,person,0);
	return self->cur;
}
int meas_q(qsys *self,int qubit,int c_id,int rem)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=3-rem;
	self->operations[self->cur].first=qubit;		
	self->operations[self->cur].second=c_id;
	if(rem)
		remove_q(&(self->val),qubit);
	return self->operations[(self->cur)++].first;
}
int send_c(qsys *self,int c_id,int person)
{
	//self->operations[self->cur]=malloc(sizeof(ops));
	self->operations[self->cur].task=cmd_sendc;
	self->operations[self->cur].first=c_id;
	self->operations[self->cur].second=person;
	return (self->cur)++;
}
int recv_c(qsys *self,int q_id,int person)
{

	self->operations[self->cur].task=cmd_recvc;
	self->operations[self->cur].first=q_id;
	self->operations[self->cur].second=person;
	//self->c_res[(self->icr)++]=resval;
	return (self->cur)++;
}

int add_bits(qsys *self,int id_0,int id_1)
{
	add_inst(self,cmd_add,id_0,id_1,0);
}
int add_val(qsys *self,int id_0,int val)
{
	add_inst(self,cmd_add,id_0,val,1);
}
int set_val(qsys *self,int id_0,int val)
{
	add_inst(self,cmd_add,id_0,val,2);
}
union return_msg {
struct	{
	int type;
	int results[200];
	};
char str[200*4+4];
};
int show_code(qsys *self);
int perform_sys(qsys *self,int prio,int get_return)
{
	struct timeval time;
	if(self->val)
		printf("ERR: there are qubits which are still in use\n");
	printf("amount of steps:%d,amount of final vals:%d,max_qubits:%d\n",self->cur,self->icr,self->max_q);
	/*for(int i=0;i< self->cur; ++i)
		{
		printf("task %d: %d,first:%d,second: %d,third:%d\n",i,self->operations[i].task,self->operations[i].first,self->operations[i].second,self->operations[i].third);
	
		}*/
	
	self->prio=prio;
	self->operations[(self->cur)++].task=cmd_done;
	show_code(self);
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
			printf("ms was not rec\n");
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
int load_prog(qsys *self,int prio)
{
	if(self->val)
		printf("ERR: there are qubits which are still in use\n");
	self->prio=prio;
	self->operations[(self->cur)++].task=cmd_done;
	
	int fd,ret;
	fd = open("/dev/nodeos", O_RDWR);
	ioctl(fd, self->prot_id*10+1, (int32_t*) &self->str); 
	if (ret < 0){
      		perror("Failed to write the message to the device.");
      		return errno;
		}
	return 0;
}
int get_status(int app_id,qsys *self=NULL)
{	
		union return_msg ms;
		int fd,ret;
		fd = open("/dev/nodeos", O_RDWR);
		ret = ioctl(fd, app_id*10+2, (int32_t*) &ms.str);  
		if(ret<0)
			{
			printf("%d:ms was not rec",app_id);
			return -1;
			}
		if(ms.type==0)
			{
			printf("%d:still running: %d/%d \n",app_id,ms.results[0],ms.results[1]);
			return 1;
			}
		if (ms.type==1)
			{
			printf("%d:program was executed. \n",app_id);
			for(int i=0;i< self->icr; ++i)
			{
				printf(" %d ",ms.results[i]);
				if (self!=NULL)
					*(self->c_res[i])=ms.results[i];
			}		
			printf("\n");	
			return 0;
			}
}

int show_code(qsys *self)
{
	printf("System code for person %d:\n",self->np);
	printf("Classical variables: %d\n",self->icr);
	printf("Quantum variables: %d\n",self->max_q);
	for(int i=0;i<self->cur; ++i)
	{
		ops curop=self->operations[i];
		switch(curop.task)
		{
		case cmd_new_qubit:
			printf("%d: qubit q%d\n",i,curop.first);
			break;
		case cmd_meas_rm:
			printf("%d: measure q%d into c%d\n",i,curop.first,curop.second);
			break;
		case cmd_xgate:
			printf("%d: Apply X to q%d\n",i,curop.first);
			break;
		case cmd_ygate:
			printf("%d: Apply Y to q%d\n",i,curop.first);
			break;
		case cmd_zgate:
			printf("%d: Apply Z to q%d\n",i,curop.first);
			break;
		case cmd_hgate:
			printf("%d: Apply H to q%d\n",i,curop.first);
			break;
		case cmd_tgate:
			printf("%d: Apply T to q%d\n",i,curop.first);
			break;
		case cmd_kgate:
			printf("%d: Apply K to q%d\n",i,curop.first);
			break;
		case cmd_xrot:
			printf("%d: Apply X rotation to q%d of %d/256\n",i,curop.first,curop.second);
			break;
		case cmd_yrot:
			printf("%d: Apply Y rotation to q%d of %d/256\n",i,curop.first,curop.second);
			break;
		case cmd_zrot:
			printf("%d: Apply Z rotation to q%d of %d/256\n",i,curop.first,curop.second);
			break;
		case cmd_cnot:
			printf("%d: CNOT on q%d with q%d as control\n",i,curop.first,curop.second);
			break;
		case cmd_cphase:
			printf("%d: CPhase on q%d with q%d\n",i,curop.first,curop.second);
			break;
		case cmd_swap:
			printf("%d: SWAP on q%d with q%d\n",i,curop.first,curop.second);
			break;
		case cmd_if:
			printf("%d: if c%d",i,curop.first);
			if(curop.third<0)
				printf("<");
			if(curop.third>0)
				printf(">");
			if (curop.third<2 && curop.third>-2)
				printf("=");
			printf("c%d perform next line\n",curop.second);
			break;
		
		case cmd_goto:
			printf("%d: GOTO line %d\n",i,curop.first);
			break;
		case cmd_add:
			if(curop.third==0)
				printf("%d: c%d =c%d+c%d\n",i,curop.first,curop.first,curop.second);
			if(curop.third==1)
				printf("%d: c%d =c%d+%d\n",i,curop.first,curop.first,curop.second);
			if(curop.third==2)
				printf("%d: c%d =%d\n",i,curop.first,curop.second);
			break;

		case cmd_sendc:
			printf("%d: Send c%d to person: p%d\n",i,curop.first,curop.second);
			break;
		case cmd_recvc:
			printf("%d: Recieve c%d from person: p%d\n",i,curop.first,curop.second);
			break;
		case cmd_eprq:
			printf("%d: Use EPR with p%d as q%d\n",i,curop.second,curop.first);
			break;
		case cmd_done:
			printf("%d: Computation over, return values\n",i);
			break;
		}
	}
}
int qasm(qsys *self)
{
	ofstream myfile;
  	myfile.open ("outp.qasm",ios::trunc);
  	myfile << "#Auto generated QASM\n";
	int qubits[100];
	for(int i=0;i<100;++i)
		qubits[i]=0;
	for(int i=0; i< self->cur;++i)
	{
		ops curop=self->operations[i];
		switch(curop.task)
		{
		case cmd_new_qubit:
			if(qubits[curop.first]==0)
				myfile << "\t qubit\t q"<<curop.first<<",0\n";
			else
				myfile << "\t zero\t q"<<curop.first<<"\n";
			qubits[curop.first]=1;
			break;
		case cmd_meas_rm:
			myfile << "\t measure\t q"<<curop.first<<"\n";
			break;
		case cmd_hgate:
			myfile << "\t H\t q"<<curop.first<<"\n";
			break;
		case cmd_tgate:
			myfile << "\t T\t q"<<curop.first<<"\n";
			break;
		case cmd_xgate:
			myfile << "\t X\t q"<<curop.first<<"\n";
			break;
		case cmd_ygate:
			myfile << "\t Y\t q"<<curop.first<<"\n";
			break;
		case cmd_zgate:
			myfile << "\t Z\t q"<<curop.first<<"\n";
			break;
		case cmd_cnot:
			myfile << "\t cnot \t q"<<curop.first<<",q"<<curop.second<<"\n";
			break;
		case cmd_cphase:
			myfile << "\t c-z \t q"<<curop.first<<",q"<<curop.second<<"\n";
			break;
		case cmd_swap:
			myfile << "\t swap \t q"<<curop.first<<",q"<<curop.second<<"\n";
			break;
		}
	}
  	myfile.close();
}
qsys new_sys;
qsys *default_sys;
int change_sys(qsys *sys)
{
	default_sys=sys;
}
qsys *nv_sys(qsys *self)
{
	init_sys(&new_sys,0,self->prot_id);
	int qlis[self->max_q];
	for(int i=0; i< self->max_q;++i)
		qlis[i]=i;
	for(int i=0; i< self->cur;++i)
	{
		ops curop=self->operations[i];
		if(curop.task>19 && curop.task<23) //Two qubit gates
		{
			if(qlis[curop.first]!=0 && qlis[curop.second]!=0)
			{
			
				apply_2gate(&new_sys,cmd_swap,qlis[curop.first],0);
				int inter=qlis[0];
				qlis[0]=qlis[curop.first];
				qlis[curop.first]=inter;
			}	
		}
		add_inst(&new_sys,curop.task,qlis[curop.first],qlis[curop.second],curop.third);
	}
	return &new_sys;
}	
//gate definition

//*********************************************************QUBITS*****************************************
struct gate2_op;
class intq;
struct gate2_op{
	int gate;
	intq *qubit;
};
struct gate2_op *g(int gate,intq *qubit)
{
	struct gate2_op *x;
	x=(struct gate2_op *) malloc(1);
	x->gate=gate;
	x->qubit=qubit;
	return x;
};
class intq {
	public:
	qsys *sys;
	int id;
	int type;
	int additional;
	intq(int person=-1,qsys* s=default_sys,int typ='g',char basis='z')
	{
		sys=s;
		type=typ;
		if(person==-1)
			id=create_q(sys);
		else
		{
			id=put_q(&(s->val));
			if((id+1)>s->max_q)
				s->max_q=id+1;
			epr_q(s,id,person);
		}
	}
	intq operator *=(int gate)
	{
		apply_gate(this->sys,gate,this->id);
		return *this;
	}
	intq operator *=(struct gate2_op *x)
	{
		apply_2gate(this->sys,x->gate,this->id,x->qubit->id);
		free(x);
		return *this;
	}
};


//int con_qs(qsys *sys,qubit *q);


//********************************************************variables***************************************
class intc{
	public:
	qsys *sys;
	int original;
	int id;
	int val;
	
	intc(int person=-1, qsys* s=default_sys,int o=0)
	{
		s->c_res[s->icr]=&val;
		sys=s;
		original=o;
		id=(s->icr)++;
		if(person==-1)
		{
			val=0;
			if (o!=0) add_inst(s,cmd_add,id,o,-1);
		}
		else
			recv_c(s,id,person);
	}
	intc operator+(const intc& other) const
	{
		if(this->sys!=other.sys) 
			throw std::invalid_argument("variables from different systems\n");
		intc res; 
		//printf("%d%d\n!",this->original,other.original);
		
		add_inst(other.sys,cmd_add,res.id,this->id,other.id);
		return res;
	}
	intc& operator=(intc other)
	{
		this->id=other.id;
		return *this;
	}
	intc& operator=(intq qubit)
	{
		if(this->sys!=qubit.sys) 
			throw std::invalid_argument("variables from different systems\n");
		meas_q(this->sys,qubit.id,this->id,1);//It now allways removes. Change this
		return *this;
	}
	intc& operator+=(intq qubit)
	{
		if(this->sys!=qubit.sys) 
			throw std::invalid_argument("variables from different systems\n");
		intc m;
		
		meas_q(this->sys,qubit.id,m.id,1);//It now allways removes. Change this
		*this+=m;
		return *this;
	}
	intc& operator+=(intc val)
	{
		if(this->sys!=val.sys) 
			throw std::invalid_argument("variables from different systems\n");
		add_bits(this->sys,this->id,val.id);
		return *this;
	}
	intc& operator=(int other)
	{
		set_val(this->sys,this->id,other);
		//add_inst(this->sys,cmd_add,this->id,other,-1); //-1: set value, -2 add value
		return *this;
	}
};

int send_data(intc *c,int person,qsys *s=default_sys)
{
	send_c(s,c->id,person);
}
int current_nested=0;
struct forloop_help
{
	int line_start;
	int change;
	intc *variable;
	
};
struct forloop_help nesting[10];

int ifq(intc *a,intc *b, int sym=0)
{
	add_inst(default_sys,cmd_if,a->id,b->id,sym);
}
int ifq(intc *a,int b, int sym=0)
{
	intc b_c;
	b_c=b;
	add_inst(default_sys,cmd_if,a->id,b_c.id,sym);
}
int forq(intc *start,int cond,int change)
{
	nesting[current_nested].line_start=default_sys->cur+1;
	ifq(start,cond,2);
	//add_inst(default_sys,cmd_if,start->id,cond_c.id,1); //if start<cond continue
	add_inst(default_sys,cmd_goto,0,0,-1);
	
	nesting[current_nested].change=change;
	nesting[current_nested].variable=start;
	++current_nested;
	return current_nested;
}
int endq()
{
	--(current_nested);
	//*(nesting[current_nested].variable)+=nesting[current_nested].change;
	add_val(default_sys,nesting[current_nested].variable->id,nesting[current_nested].change);
	add_inst(default_sys,cmd_goto, nesting[current_nested].line_start,0,-10);
	default_sys->operations[nesting[current_nested].line_start+1].first=default_sys->cur;
	return current_nested;
}

int qft(qsys *self,intc *q,int n)
{
	for(int i=0;i<n;++i)
		{
		apply_gate(self,cmd_hgate,q[i].id);	
		for(int j=i+1;j<n;++j)
			apply_cpgate(self,q[i].id,q[j].id,256/(1<<(j-i+1)));
		}

}
int teleportation()
{
	int alice=0;
	int bob=1;
	qsys sysA;init_sys(&sysA,alice,0);
	change_sys(&sysA);
	intc res;
	res=0;	
	intc a;
	forq(&(a=0),100,1);
	{
		
		intq q0(bob);
		intc m0(bob);
		printf("m0.id:%d\n",m0.id);
		ifq(&m0,1);
			q0*=cmd_xgate;
		intc m1(bob);
		ifq(&m1,1);
			q0*=cmd_zgate;
		res+=q0;
		
	}endq();
	
	
	//bob_part
	qsys sysB;init_sys(&sysB,bob,0);
	change_sys(&sysB);
	intc b;
	forq(&(b=0),100,1);
	{
	intq qb0(alice);
	intq qb1;
	qb0*=g(cmd_cnot,&qb1);
	qb0*=cmd_hgate;
	intc cb0;
	cb0=qb0;
	send_data(&cb0,alice);
	intc cb1;
	cb1=qb1;
	send_data(&cb1,alice);
	}endq();
	
	/*intc qs[5];
	for(int i=0; i<5;++i)
		qs[i]=i;
	intc q0(&sysA);
	intc q1(&sysA);
	intc q2(&sysA);
	intc q3(&sysA);
	intc q4(&sysA);
	printf("curr:%d",sysA.cur);
	qft(&sysA,qs,5);*/
	/*intq q0;
	intq q1;
	intq q2;
	q0*=cmd_hgate;
	q0*=g(cmd_cnot,&q1);
	intc i;
	i=q0;
	intq q4;
	q4*=cmd_hgate;
	q1*=g(cmd_cnot,&q2);
	intc m;
	intc j;
	j=q1;
	qasm(nv_sys(&sysA));
	//qasm(&sysA);

	*/
	perform_sys(&sysA,0,0);
	perform_sys(&sysB,0,0);
	
	return 0;
}
int sum_to_100()
{
	srand(time(NULL));
	int prot_id=rand()%10000;
	int alice=0;
	int bob=1;
	qsys sysA;init_sys(&sysA,alice,prot_id);
	change_sys(&sysA);
	intc val;
	val=0;
	intc i;
	forq(&(i=0),100,1);
		val+=i;
	endq();
	
	show_code(&sysA);
	load_prog(&sysA,100);
	for( int i=0; i<10;++i)
		get_status(prot_id,&sysA);
	//perform_sys(&sysA,0,1);
	//get_status
}
int parity_check()
{
	//sum_to_100();
	
	int prot_id=rand()%10000;
	int alice=0;
	int bob=1;
	qsys sysA;init_sys(&sysA,alice,prot_id);
	change_sys(&sysA);
	intc val;
	val=0;
	intc i;
	forq(&(i=0),10,1);
		intq q0;
		q0*=cmd_hgate;
		val+=q0;
	endq();
	
	show_code(&sysA);
	load_prog(&sysA,100);
	return prot_id;

	//perform_sys(&sysA,0,1);
	//get_status
}
int start_progs(int *ids)
{
	for(int i=0; i<5;++i)
		ids[i]=parity_check();
	return 0;
}
int main()
{	
	srand(time(NULL));
	int ids[100];
	for(int i=0;i<100;++i)
		ids[i]=0;
	start_progs(ids);
	for(int j=0;j<10;++j)
	{
		printf("round: %d\n",j+1);
		for( int i=0; i<10;++i)
		{
			usleep(500);
			if(ids[i]!=0)
				get_status(ids[i]);
		}
		usleep(1000000);
	}

}
