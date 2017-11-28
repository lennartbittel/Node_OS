/**
 * @file   ebbchar.c
 * @author Derek Molloy
 * @date   7 April 2015
 * @version 0.1
 * @brief   An introductory character driver to support the second article of my series on
 * Linux loadable kernel module (LKM) development. This module maps to /dev/ebbchar and
 * comes with a helper C program that can be run in Linux user space to communicate with
 * this the LKM.
 * @see http://www.derekmolloy.ie/ for a full description and follow-up descriptions.
 */

#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#define  DEVICE_NAME "ebbchar"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "ebb"        ///< The device class -- this is a character device driver

#include <linux/net.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/in.h>
#include <linux/socket.h>
#include <linux/slab.h>


//#include "cqc.h"

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Lennart Bittel");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Node OS");  ///< The description -- see modinfo
MODULE_VERSION("0.0.1");            ///< A version number to inform users

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[256] = {0};           ///< Memory for the string that is passed from userspace
static short  size_of_message;              ///< Used to remember the size of the string stored
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open1(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open1,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
};


#define	CQC_HDR_LENGTH      8
typedef union
{
	struct
	{
		uint8_t version; 
		uint8_t type;		/* Packet control type */
		uint16_t app_id; 	/* Application ID */
		uint32_t length;	/* Total length of command instructions to send */
		void *payload; 		/* Additional details for this command */
	} __attribute__((__packed__));
	char str[CQC_HDR_LENGTH];		/* Additional details for this command */
} __attribute__((__packed__)) cqcHeader;

#define	CQC_CMD_HDR_LENGTH      4
typedef union {
	struct
	{
		uint16_t qubit_id;	/* Qubit to perform the operation on */
		uint8_t instr;		/* Instruction to execute */
		uint8_t options;	/* Options when executing the command */
		void *extraCmd;		/* Additional details for this command */
	} __attribute__((__packed__));
	char str[CQC_CMD_HDR_LENGTH];
} __attribute__((__packed__)) cmdHeader;
	


#define CQC_CMD_XTRA_LENGTH	16
typedef union {
	struct
	{
		uint16_t xtra_qubit_id;	/* ID of the additional qubit */
		uint16_t remote_app_id;	/* Remote application ID */
		uint32_t remote_node;	/* IP of the remote node */
		uint32_t cmdLength;	/* Length of the cmds to exectute upon completion */
		uint16_t remote_port;	/* Port of the remote node for control info */
		uint8_t steps;		/* Angle step of rotation (ROT) OR number of repetitions (FACTORY) */
		uint8_t unused;		/* Need 4 byte segments */
		void *cmdPayload;	/* Details to execute when done with this command */
	} __attribute__((__packed__));
	char str[CQC_CMD_XTRA_LENGTH];
} __attribute__((__packed__)) xtraCmdHeader;


#define CQC_NOTIFY_LENGTH	20
typedef union {
	struct
	{
		uint16_t qubit_id;	/* ID of the received qubit, if any */
		uint16_t remote_app_id;	/* Remote application ID */
		uint32_t remote_node;	/* IP of the remote node */
		uint64_t datetime;	/* Time of qubit */
		uint16_t remote_port;	/* Port of the remote node for control info */
		uint8_t outcome;	/* Measurement outcome */
		uint8_t unused;	/* Need 4 byte segments */
	} __attribute__((__packed__));
	char str[CQC_NOTIFY_LENGTH];
} __attribute__((__packed__)) notifyHeader;
//shit from client
#define PORT 8822

struct socket *conn_socket = NULL;

u32 create_address(u8 *ip)
{
        u32 addr = 0;
        int i;

        for(i=0; i<4; i++)
        {
                addr += ip[i];
                if(i==3)
                        break;
                addr <<= 8;
        }
        return addr;
}

int tcp_client_send(struct socket *sock, const char *buf, const size_t length,unsigned long flags)
{
        struct msghdr msg;
        //struct iovec iov;
        struct kvec vec;
        int len, written = 0, left = length;
        mm_segment_t oldmm;
        msg.msg_name    = 0;
        msg.msg_namelen = 0;
        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        msg.msg_flags   = flags;

        oldmm = get_fs(); set_fs(KERNEL_DS);
repeat_send:
        vec.iov_len = left;
        vec.iov_base = (char *)buf + written;

        //len = sock_sendmsg(sock, &msg, left);
        len = kernel_sendmsg(sock, &msg, &vec, left, left);
        if((len == -ERESTARTSYS) || (!(flags & MSG_DONTWAIT) &&\
                                (len == -EAGAIN)))
                goto repeat_send;
        if(len > 0)
        {
                written += len;
                left -= len;
                if(left)
                        goto repeat_send;
        }
        set_fs(oldmm);
        return written ? written:len;
}

int tcp_client_receive(struct socket *sock, char *str,unsigned long flags)
{
        //mm_segment_t oldmm;
        struct msghdr msg;
        //struct iovec iov;
        struct kvec vec;
        int len;
        int max_size = 50;

        msg.msg_name    = 0;
        msg.msg_namelen = 0;

        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        msg.msg_flags   = flags;

        vec.iov_len = max_size;
        vec.iov_base = str;

        //oldmm = get_fs(); set_fs(KERNEL_DS);
//read_again:
        len = kernel_recvmsg(sock, &msg, &vec, max_size, max_size, flags);

        if(len == -EAGAIN || len == -ERESTARTSYS)
        {
                pr_info(" *** mtp | error while reading: %d | "
                        "tcp_client_receive *** \n", len);

                //goto read_again;
        }


        pr_info(" *** mtp | the server says: %s ,size: %d| tcp_client_receive *** \n", str,(int)(sizeof(str)));
        //set_fs(oldmm);
	
	//ping back again
	//pr_info("CL: send back res:%d,%s",tcp_client_send(sock, str, strlen(str), MSG_DONTWAIT),str);
        return len;
}

int tcp_client_connect(void)
{
        struct sockaddr_in saddr;
        /*
        struct sockaddr_in daddr;
        struct socket *data_socket = NULL;
        */
        unsigned char destip[5] = {131,180,82,194 ,'\0'};
        /*
        char *response = kmalloc(4096, GFP_KERNEL);
        char *reply = kmalloc(4096, GFP_KERNEL);
        */
        int len = 49;
        char response[len+1];
        char reply[len+1];
        int ret = -1;

        //DECLARE_WAITQUEUE(recv_wait, current);
        DECLARE_WAIT_QUEUE_HEAD(recv_wait);
        
        ret = sock_create(AF_INET, SOCK_STREAM, IPPROTO_TCP, &conn_socket);
        if(ret < 0)
        {
                pr_info(" *** mtp | Error: %d while creating first socket. | "
                        "setup_connection *** \n", ret);
                goto err;
        }

        memset(&saddr, 0, sizeof(saddr));
        saddr.sin_family = AF_INET;
        saddr.sin_port = htons(PORT);
        saddr.sin_addr.s_addr = htonl(create_address(destip));

        ret = conn_socket->ops->connect(conn_socket, (struct sockaddr *)&saddr , sizeof(saddr), O_RDWR);
        if(ret && (ret != -EINPROGRESS))
        {
                pr_info(" *** mtp | Error: %d while connecting using conn "
                        "socket. | setup_connection *** \n", ret);
                goto err;
        }

        memset(&reply, 0, len+1);
        strcat(reply, "dick"); 
	/*
	cqcHeader cqcH;

	cqcH.version = 0;
	cqcH.app_id = 13;
	cqcH.type = 0;
	cqcH.length = 0;
        //tcp_client_send(conn_socket, reply, strlen(reply), MSG_DONTWAIT);
	tcp_client_send(conn_socket, cqcH.str, CQC_HDR_LENGTH, MSG_DONTWAIT);
	*/
	/*memset(&reply, 0, len+1);
	strcat(reply, "HOLAdickface"); 
	tcp_client_send(conn_socket, reply, strlen(reply), MSG_DONTWAIT);*/
        wait_event_timeout(recv_wait,!skb_queue_empty(&conn_socket->sk->sk_receive_queue),\
                                                                        5*HZ);
        
        /*add_wait_queue(&conn_socket->sk->sk_wq->wait, &recv_wait);
        while(1)
        {
                //__set_current_status(TASK_INTERRUPTIBLE);
                schedule_timeout(HZ);*/
        
                if(!skb_queue_empty(&conn_socket->sk->sk_receive_queue))
                {
                        /*
                        __set_current_status(TASK_RUNNING);
                        remove_wait_queue(&conn_socket->sk->sk_wq->wait,\
                                                              &recv_wait);
                        */
                        memset(&response, 0, len+1);
                        tcp_client_receive(conn_socket, response, MSG_DONTWAIT);
                        //break;
                }

        /*
        }
        */

err:
        return -1;
}


/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */



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


#define cmd_done		1000



#define usr_identity   11
typedef struct {
	int task;
	int first;
	int second;
	int third;
}command;
typedef struct {
	char name[100];
	int state;
	int priority;
	command **code;
	int codevar[10];
	int qubits[10];
	int cur; //which line to evaluate next
	int size;
	int breaks;
	int starting_time;
	}Progs;

cqcHeader cqcH;
cmdHeader cmdH;
xtraCmdHeader xtra;
command cur;
#define CQC_OPT_NOTIFY 		0x01
#define CQC_OPT_BLOCK           0x04
int perform_cmd(Progs *prog)
{
	printk(KERN_INFO "perform cmd. Hope this will not break...\n");
	cur=*(prog->code[prog->cur]);
	printk(KERN_INFO "Taks started:%d, %d\n",prog->cur,cur.task);
	//cqc_task
	
	cqcH.version = 0;
	cqcH.app_id = usr_identity;
	cqcH.type = 1; //command
	
	
	
	cmdH.qubit_id=cur.first;
	cmdH.instr=cur.task;
	cmdH.options=CQC_OPT_NOTIFY+CQC_OPT_BLOCK;// determines notify, block, action Add as additional!!
	

	
	xtra.cmdLength = 0;
	if(cur.task==-1)
		{
			cqcH.type = 0;
			cqcH.length = 0;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH, MSG_DONTWAIT);
			
		}
	if(cur.task==cmd_identity) 
		{
			cqcH.length = CQC_CMD_HDR_LENGTH;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH, MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
		
		}
	else if(cur.task==cmd_new_qubit||cur.task==cmd_reset) //new qubit
		{
			cmdH.qubit_id=cur.first;
			cqcH.length = CQC_CMD_HDR_LENGTH;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
		}
	else if(cur.task==cmd_meas)  //measurement of qubit
		{
			
			cqcH.length = CQC_CMD_HDR_LENGTH;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);

			//notifyHeader noty;
			//tcp_client_receive(conn_socket, noty.str,MSG_DONTWAIT);
			//prog->codevar[cur.second]=noty.outcome;
			//printk(KERN_INFO "Mes result: %d\n",noty.outcome);
			
		}
	else if((cur.task<=13 && cur.task>=10) ||cur.task==cmd_hgate || cur.task==cmd_kgate) //single qubit gates
		{
			cqcH.length = CQC_CMD_HDR_LENGTH;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
		}

	else if(cur.task==cmd_cnot || cur.task==cmd_cphase ) //single qubit gates
		{
			cqcH.length = CQC_CMD_HDR_LENGTH+CQC_CMD_XTRA_LENGTH;
			xtra.xtra_qubit_id=cur.second;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, xtra.str,CQC_CMD_XTRA_LENGTH , MSG_DONTWAIT);
		}
	else if(cur.task==cmd_xrot || cur.task==cmd_yrot || cur.task==cmd_zrot ) //single qubit gates
		{
			cqcH.length = CQC_CMD_HDR_LENGTH+CQC_CMD_XTRA_LENGTH;
			xtra.steps=cur.second;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, xtra.str,CQC_CMD_XTRA_LENGTH , MSG_DONTWAIT);
		}
	else if(cur.task==cmd_sendq || cur.task==cmd_recvq || cur.task==cmd_eprq) //connection with bell pairs
		{
			cqcH.length = CQC_CMD_HDR_LENGTH+CQC_CMD_XTRA_LENGTH;
			xtra.xtra_qubit_id=cur.second;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, xtra.str,CQC_CMD_XTRA_LENGTH , MSG_DONTWAIT);
		}
	
	else if(cur.task==cmd_if) //if statement
		{
			if(prog->codevar[cur.first])
				++(prog->cur); // If true the second line is executed
			
		}
	else if (cur.task==cmd_done)
		{
			
		}
	++(prog->cur);
	return 0;	 
}













#define max_prog 100
#define amount_qubits 10
//typedef enum { false, true } bool;
typedef struct
{
	char name [20];
	unsigned char ip[5];
	unsigned int port;
}people;
typedef struct
{
	
	Progs **Running;
	int used_r[max_prog];
	Progs **Sleeping;
	int used_s[max_prog];
	Progs **Not_started;
	int used_n[max_prog];
	int qubits[amount_qubits];
	int q_live[amount_qubits];
	int g_prio[max_prog]; 
	Progs prog_lis[max_prog];
}OS;
int OS_init(OS *self){
	int i;
	self->Running=kmalloc(max_prog * sizeof(struct Progs *),GFP_KERNEL);
	self->Sleeping=kmalloc(max_prog * sizeof(struct Progs *),GFP_KERNEL);
	self->Not_started=kmalloc(max_prog * sizeof(struct Progs *),GFP_KERNEL);
	
	for(i=0; i<amount_qubits;++i)
	{
		self->qubits[i]=0;
		self->q_live[i]=-1;
	}
	for(i=0; i<max_prog;++i)
	{
		self->used_r[i]=0;
		self->used_s[i]=0;
		self->used_n[i]=0;
		self->g_prio[i]=-1;
	}
	return 0;
}


int OS_make_prio(OS *self)
{	
	int i;
	for(i=0;i<max_prog;++i)
	{
		if(self->used_r[i])
		{
			self->g_prio[i]=(10-self->Running[i]->priority)*10;
		}
	}
	return 0;
}
int OS_next_p(OS *self)
{
	int i=0;
	int k;
	for(k=1;k<max_prog;++k)
	{
		if(self->used_r[k])
		{
			if(self->g_prio[k]<self->g_prio[i])//get minimum
				i=k;
		}
	}
	return i;
}
int OS_insert_p(OS *self,Progs *prog)
{
	int i;
	for(i=0; i< max_prog;++i)
	{
		if(!self->used_n[i])
		{
		self->Not_started[i]=prog;
		self->used_n[i]=1;
		return i;
		}
	}
	return -1;
}
int OS_start_p(OS *self,int prog_num)
{
	int i;
	for(i=0; i< max_prog;++i)
	{
		if(!self->used_r[i])
		{
			self->Running[i]=self->Not_started[prog_num];
			self->used_r[prog_num]=1;
			self->Not_started[prog_num]=NULL;
			self->used_n[prog_num]=0;
			return i;
		}
	}
	return -1;
}
int OS_sleep_p(OS *self,int prog_num)
{
	//int i=kmalloc(sizeof(int),GFP_KERNEL);
	int i=0;
	for(i=0; i< max_prog;++i)
	{
		if(!self->used_s[i])
		{
			self->Sleeping[i]=self->Running[prog_num];
			self->used_s[prog_num]=1;
			self->Running[prog_num]=NULL;
			self->used_r[prog_num]=0;
			return i;
		}
	}
	return -1;
}
int OS_run_p(OS *self)
{
	int i=OS_next_p(self);
	Progs *prog=self->Running[i];
	int res=perform_cmd(prog);
	if(res<0)
		printk(KERN_INFO "Running Task has failed");
	++(self->g_prio[i]);
	return -1;
	
}
int OS_term_p(OS *self, int prog_n)
	{
		return 0;
	}






//loading docs into the OS. I SHOULD NOT DO THIS

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

struct file *file_open(const char *path, int flags, int rights) 
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

void file_close(struct file *file) 
{
    filp_close(file, NULL);
}

int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}   

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int file_sync(struct file *file) 
{
    vfs_fsync(file, 0);
    return 0;
}


command my_c0;
command my_c1;
command my_c2;
command my_c3;
Progs p;
OS my_os;
int prog_id;
//char cqc_back_cmd[100];
cqcHeader cqc_back_cmd;
static int __init ebbchar_init(void){
   cqcHeader cqcH;
   cqcH.version=2;
   printk(KERN_INFO "EBBChar: Initializing the EBBChar LKM %d \n",cqcH.version);

   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "EBBChar failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "EBBChar: registered correctly with major number %d\n", majorNumber);

   // Register the device class
   ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(ebbcharClass)){                // Check for error and clean up if there is
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(ebbcharClass);          // Correct way to return an error on a pointer
   }
   printk(KERN_INFO "EBBChar: device class registered correctly\n");

   // Register the device driver
   ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
      class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(ebbcharDevice);
   }
   printk(KERN_INFO "EBBChar: device class created correctly\n"); // Made it! device was initialized
	//network stuff
   printk(KERN_INFO "CL:establish TCP connection");
   tcp_client_connect();
   printk(KERN_INFO "start tests\n");
   p.cur=0;
   p.code=kmalloc(sizeof(command *)*10,GFP_KERNEL);
   my_c0.task=-1; my_c0.first=0;p.code[0]=&my_c0;
   my_c1.task=cmd_new_qubit; my_c1.first=0;p.code[1]=&my_c1;
   my_c2.task=cmd_xrot; my_c2.first=0; my_c2.second=10;p.code[2]=&my_c2;
   my_c3.task=cmd_meas; my_c3.first=0;p.code[3]=&my_c3;
   printk(KERN_INFO "Prog is initialized,%d\n",(int)(sizeof(command *)));
   OS_init(&my_os);
   printk(KERN_INFO "OS_init");
   prog_id=OS_insert_p(&my_os,&p);
   printk(KERN_INFO "Added prog");
   OS_start_p(&my_os,prog_id);
   printk(KERN_INFO "Prog ready, This will start the code");
   	OS_run_p(&my_os);
   printk(KERN_INFO "Try to recv:");
   mdelay(1000);
   prog_id=tcp_client_receive(conn_socket, cqc_back_cmd.str,MSG_DONTWAIT);
   printk(KERN_INFO "res: %d, data: ver:%d,type:%d,app_id:%d, length: %d ",prog_id,cqc_back_cmd.version,cqc_back_cmd.type,cqc_back_cmd.app_id,cqc_back_cmd.length);


   	OS_run_p(&my_os);mdelay(1000);
   mdelay(1000);
   prog_id=tcp_client_receive(conn_socket, cqc_back_cmd.str,MSG_DONTWAIT);
   printk(KERN_INFO "res: %d, data: ver:%d,type:%d,app_id:%d, length: %d",prog_id,cqc_back_cmd.version,cqc_back_cmd.type,cqc_back_cmd.app_id,cqc_back_cmd.length);


   	OS_run_p(&my_os);mdelay(1000);
   prog_id=tcp_client_receive(conn_socket, cqc_back_cmd.str,MSG_DONTWAIT);
   printk(KERN_INFO "res: %d, data: ver:%d,type:%d,app_id:%d, length: %d",prog_id,cqc_back_cmd.version,cqc_back_cmd.type,cqc_back_cmd.app_id,cqc_back_cmd.length);


   	OS_run_p(&my_os);mdelay(3000);
   prog_id=tcp_client_receive(conn_socket, cqc_back_cmd.str,MSG_DONTWAIT);
   printk(KERN_INFO "res: %d, data: ver:%d,type:%d,app_id:%d, length: %d\n",prog_id,cqc_back_cmd.version,cqc_back_cmd.type,cqc_back_cmd.app_id,cqc_back_cmd.length);
   return 0;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
int len;
char *response;
char *reply;
static void __exit ebbchar_exit(void){
   DECLARE_WAIT_QUEUE_HEAD(exit_wait);
   device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(ebbcharClass);                          // unregister the device class
   class_destroy(ebbcharClass);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
   printk(KERN_INFO "EBBChar: Goodbye from the LKM!\n");
   //NETWORK END
	/* This is now commented to prevent crash..
	len=49;
	response=kmalloc( (len+1)*sizeof(char),GFP_KERNEL);
	reply=kmalloc( (len+1)*sizeof(char),GFP_KERNEL);
	
        //DECLARE_WAITQUEUE(exit_wait, current);
        

        memset(&reply, 0, len+1);
        strcat(reply, "ADIOS"); 
        //tcp_client_send(conn_socket, reply);
        tcp_client_send(conn_socket, reply, strlen(reply), MSG_DONTWAIT);
        //while(1)
        //{
                
                tcp_client_receive(conn_socket, response);
                add_wait_queue(&conn_socket->sk->sk_wq->wait, &exit_wait)
                
         wait_event_timeout(exit_wait,\
                         !skb_queue_empty(&conn_socket->sk->sk_receive_queue),\
                                                                        5*HZ);
        if(!skb_queue_empty(&conn_socket->sk->sk_receive_queue))
        {
                memset(&response, 0, len+1);
                tcp_client_receive(conn_socket, response, MSG_DONTWAIT);
                //remove_wait_queue(&conn_socket->sk->sk_wq->wait, &exit_wait);
        }

        //}
	*/
        if(conn_socket != NULL)
        {
                sock_release(conn_socket);
        }
	kfree(response);
	kfree(reply);
	kfree(p.code);
	kfree(my_os.Running);
	kfree(my_os.Sleeping);
	kfree(my_os.Not_started);
	printk(KERN_INFO "CL: Connection stopped\n");
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open1(struct inode *inodep, struct file *filep){
   numberOpens++;
   printk(KERN_INFO "EBBChar: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
   int error_count = 0;
   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
   
   
   //tcp_client_receive(conn_socket, message,MSG_DONTWAIT);
   error_count = copy_to_user(buffer, message, sizeof(message));

   if (error_count==0){            // if true then have success
      printk(KERN_INFO "EBBChar: Sent %d characters to the user\n", size_of_message);
      return (size_of_message=0);  // clear the position to the start and return 0
   }
   else {
      printk(KERN_INFO "EBBChar: Failed to send %d characters to the user\n", error_count);
      return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
   
   //sprintf(message, "%s(%zu letters)", buffer, len);   // appending received string with its length
   //size_of_message = strlen(message);                 // store the length of the stored message
   printk(KERN_INFO "EBBClient : sent %zu characters to server\n", len);
   return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "EBBChar: Device successfully closed\n");
   return 0;
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(ebbchar_init);
module_exit(ebbchar_exit);
