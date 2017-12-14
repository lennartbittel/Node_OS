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
#define  DEVICE_NAME "nodeos"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "ebb"        ///< The device class -- this is a character device driver

#include <linux/net.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/in.h>
#include <linux/socket.h>
#include <linux/slab.h>


//#include "cqc.h"
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create(), kthread_run()
#define dev_int_name 1
#define PORT 8820+dev_int_name

//#include "udp_net/mod.h"

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


















struct con_infos;

#define max_con 20
struct con_infos **regist;


#define DEFAULT_PORT 8850
struct socket *sock;
struct sockaddr_in addr;

int socket_open(void);
int my_kill_proc(pid_t pid, int sig);
int ksocket_receive(struct socket *sock, struct sockaddr_in *addr, unsigned char *buf, int len);
int ksocket_send(struct socket *sock, struct sockaddr_in *addr, unsigned char *buf, int len);
struct con_infos *connect_to_ip(unsigned long address,int name);
int get_id_of_ip(unsigned long address);
int get_empty_id(void);




int my_kill_proc(pid_t pid, int sig) {
    int error = -ESRCH;           /* default return value */
    struct task_struct* p;
    struct task_struct* t = NULL; 
    struct pid* pspid;
    rcu_read_lock();
    p = &init_task;               /* start at init */
    do {
        if (p->pid == pid) {      /* does the pid (not tgid) match? */
            t = p;    
            break;
        }
        p = next_task(p);         /* "this isn't the task you're looking for" */
    } while (p != &init_task);    /* stop when we get back to init */
    if (t != NULL) {
        pspid = t->pids[PIDTYPE_PID].pid;
        if (pspid != NULL) error = kill_pid(pspid,sig,1);
    }
    rcu_read_unlock();
    return error;
}


/* function prototypes */


int socket_open(void)
{

	int  err;

        current->flags |= PF_NOFREEZE;
	allow_signal(SIGKILL);
	if ( (err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock)) < 0)
	{
		printk(KERN_INFO ": Could not create a datagram socket, error = %d\n", -ENXIO);
                goto out;
	}
	memset(&addr, 0, sizeof(struct sockaddr));
	addr.sin_family      = AF_INET;
	addr.sin_addr.s_addr      = htonl(INADDR_ANY);
	addr.sin_port      = htons(DEFAULT_PORT);
	if ( (err = sock->ops->bind(sock, (struct sockaddr *) &addr, sizeof(struct sockaddr) ) ) < 0)
        {
                printk(KERN_INFO ": Could not bind or connect to socket, error = %d\n", -err);
                goto close_and_out;
        }
	printk(KERN_INFO": listening on port %d\n", DEFAULT_PORT);
	/*while(1)
	{
		memset(&buf, 0, bufsize+1);
                size = ksocket_receive(sock, &addr, buf, bufsize);
		if (signal_pending(current))
                        break;
		printk("Inaddr: %d/%d,current s_addr:%d",INADDR_ANY,htonl(INADDR_ANY), addr.sin_addr.s_addr);
		printk(KERN_INFO MODULE_NAME": received %d bytes\n", size);
                printk("data: %s\n", buf);
	}*/
	return 0;
close_and_out:
        sock_release(sock);
	sock = NULL;
	return -1;

out:
	return -2;
}
struct con_infos{
	char name[10];
	int name_num;
	unsigned long ip;
	struct socket *sock;
	struct sockaddr_in *addr;
};


struct con_infos *connect_to_ip(unsigned long address,int name)
{
	int err;
	struct socket *sock_send;
        struct sockaddr_in addr_send;
	struct con_infos *infos=kmalloc(sizeof(struct con_infos), GFP_KERNEL);
	if((err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock_send)) < 0 )
	{
                printk(KERN_INFO": could not create socket for new person %d\n", -ENXIO);
                return NULL;
        }
	memset(&addr_send, 0, sizeof(struct sockaddr));
	addr_send.sin_family = AF_INET;
	addr_send.sin_addr.s_addr = address;
	addr_send.sin_port = htons(DEFAULT_PORT);	
	if(( err = sock_send->ops->connect(sock_send, (struct sockaddr *)&addr_send, sizeof(struct sockaddr), 0) )< 0 )
	{
	        printk(KERN_INFO": Could not connect to IP,err:%d\n", -err);
                goto close_and_out;
	}
	infos->sock=sock_send;
	infos->addr=&addr_send;
	infos->ip=address;
	infos->name_num=name;
	printk("added person,%d\n",infos->ip);
	return infos;
close_and_out:
        sock_release(sock_send);
        sock_send = NULL;
	return NULL;
}
int get_id_of_ip(unsigned long address)
{
	int i;
	for(i=0;i<max_con;++i)
		if(regist[i]!=NULL)
			{
			printk("%d,%d\n",regist[i]->addr->sin_addr.s_addr,address);
			if(regist[i]->ip==address)
				return i;
			}
	return -1;
}
int get_id_of_dev_id(int dev_id)
{
	int i;
	for(i=0;i<max_con;++i)
		if(regist[i]!=NULL)
			{
			printk("%d,%d\n",regist[i]->name_num,dev_id);
			if(regist[i]->name_num==dev_id)
				return i;
			}
	return -1;
}
int print_con(void)
{
	int i;
	printk("printing all connections");
	for(i=0;i<max_con;++i)
		{
		if(regist[i]!=NULL)
			{
			printk("id: %d name:%d,ip:%d\n",i,regist[i]->name_num,regist[i]->ip);
			}
		}
	printk("done\n");
	return 0;
}
int get_empty_id(void)
{
	int i;
	for(i=0;i<max_con;++i)
		if(regist[i]==NULL)
			return i;
	return max_con;
}


int ksocket_send(struct socket *sock, struct sockaddr_in *addr, unsigned char *buf, int len)
{
        struct msghdr msg;
        struct kvec vec;
        mm_segment_t oldfs;
        int size = 0;

        if (sock->sk==NULL)
           return 0;

        vec.iov_base = buf;
        vec.iov_len = len;

        msg.msg_flags = MSG_DONTWAIT;
        msg.msg_name = addr;
        msg.msg_namelen  = sizeof(struct sockaddr_in);
        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        //msg.msg_iov = &iov;
        //msg.msg_iovlen = 1;
        msg.msg_control = NULL;

        oldfs = get_fs();
        set_fs(KERNEL_DS);
	size=kernel_sendmsg(sock, &msg, &vec, len, len);
        //size = sock_sendmsg(sock,&msg);
	//size=___sys_sendmsg(sock,&msg,&msg_sys,MSG_DONTWAIT,NULL,0);
	printk("res:%d\n",size);
        set_fs(oldfs);

        return size;
}

int ksocket_receive(struct socket* sock, struct sockaddr_in* addr, unsigned char* buf, int len)
{
        struct msghdr msg;
        struct kvec vec;
        mm_segment_t oldfs;
        int size = 0;

        if (sock->sk==NULL) return 0;

        vec.iov_base = buf;
        vec.iov_len = len;

        msg.msg_flags = 0;
        msg.msg_name = addr;
        msg.msg_namelen  = sizeof(struct sockaddr_in);
        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        //msg.msg_iov = &iov;
        //msg.msg_iovlen = 1;
        msg.msg_control = NULL;

        oldfs = get_fs();
        set_fs(KERNEL_DS);
	size = kernel_recvmsg(sock, &msg, &vec,len,len, MSG_DONTWAIT);
        //size = sock_recvmsg(sock,&msg,len,msg.msg_flags);
        set_fs(oldfs);

        return size;
}































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

int tcp_client_receive(struct socket *sock, char *str,unsigned long flags,int max_size)
{
        struct msghdr msg;
        struct kvec vec;
        int len;
        msg.msg_name    = 0;
        msg.msg_namelen = 0;

        msg.msg_control = NULL;
        msg.msg_controllen = 0;
        msg.msg_flags   = flags;

        vec.iov_len = max_size;
        vec.iov_base = str;
        len = kernel_recvmsg(sock, &msg, &vec, max_size, max_size, flags);

        if(len == -EAGAIN || len == -ERESTARTSYS)
        {
                //pr_info(" *** mtp | error while reading: %d | tcp_client_receive *** \n", len);

        }

        return len;
}
unsigned char destip[5] = {192,168,1,1 ,'\0'};
struct sockaddr_in saddr;
int tcp_client_connect(void)
{
        
        /*
        struct sockaddr_in daddr;
        struct socket *data_socket = NULL;
        */
        
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
        strcat(reply, "hello"); 
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
                        tcp_client_receive(conn_socket, response, MSG_DONTWAIT,50);
                        //break;
                }

        /*
        }
        */

err:
        return -1;
}

#define MAX_CONNS 16
/*
int listen_for_new_people(void)
{
	int server_err;
	struct socket *listen_socket;
        struct sockaddr_in server;
	allow_signal(SIGKILL|SIGTERM);
        DECLARE_WAIT_QUEUE_HEAD(wq);
	server_err = sock_create(AF_INET, SOCK_STREAM, IPPROTO_TCP,  &listen_socket);
	if(server_err<0)
		{
		printk(KERN_INFO "Could not create listening socket");
		return -1;
		}
	listen_socket->sk->sk_reuse = 1;
	//server.sin_addr.s_addr = htonl(INADDR_ANY);
	server.sin_addr.s_addr = INADDR_ANY;
        server.sin_family = AF_INET;
        server.sin_port = htons(DEFAULT_PORT);
	server_err=listen_socket->ops->bind(listen_socket, (struct sockaddr*)&server,sizeof(server));
	if(server_err<0)
		{
		printk(KERN_INFO "Could not bind listening socket");
		return -1;
		}
	server_err = listen_socket->ops->listen(listen_socket, 16);
		if(server_err<0)
		{
		printk(KERN_INFO "Could not listen to listening socket");
		return -1;
		}
	//end of listen, start of accept
	int accept_err = 0;
	struct socket *accept_socket = NULL;
	struct inet_connection_sock *isock; 
	int id = 0;
	DECLARE_WAITQUEUE(accept_wait, current);
	while(1)
		{
		struct tcp_conn_handler_data *data = NULL;
                struct sockaddr_in *client = NULL;
		char *tmp;
                int addr_len;
		accept_err = sock_create(listen_socket->sk->sk_family, listen_socket->type,listen_socket->sk->sk_protocol, &accept_socket);
		if(accept_err < 0 || !accept_socket)
			{
				printk(KERN_INFO "accepting socket not created");
				return -1;
			}
		accept_socket->listen_type = listen_socket->type;
                accept_socket->listen_ops  = listen_socket->ops;
		isock = inet_csk(listen_socket->sk);
		add_wait_queue(&listen_socket->sk->sk_wq->wait, &accept_wait);
		while(reqsk_queue_empty(&isock->icsk_accept_queue))
			{
			__set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ);
			if(kthread_should_stop())
		               	{
		                printk(KERN_INFO "Stopping to listen for new people");
		                __set_current_state(TASK_RUNNING);
		                remove_wait_queue(&listen_socket->sk->sk_wq->wait,&accept_wait);
		                sock_release(accept_socket);
				sock_release(listen_socket);//from listen
		                return 0;
		               }
			}
		__set_current_state(TASK_RUNNING);
		remove_wait_queue(&socket->sk->sk_wq->wait, &accept_wait);
		accept_err = listen_socket->ops->accept(listen_socket, accept_socket, O_NONBLOCK);
		if(accept_err < 0 )
			{
				printk(KERN_INFO "Could not accept the request");
				return -1;
			}
		client = kmalloc(sizeof(struct sockaddr_in), GFP_KERNEL);   
                memset(client, 0, sizeof(struct sockaddr_in));

                addr_len = sizeof(struct sockaddr_in);

                accept_err = accept_socket->ops->getname(accept_socket,(struct sockaddr *)client, &addr_len, 2);


		}

	sock_release(listen_socket);
}
*/
//communication between people


typedef struct
{
	char name_str [20];
	u_int32_t ip;
	unsigned int port;
	uint32_t name;
	uint32_t id;
	struct socket *sock;
}people;
people **net_people;








/*
struct socket *connect_person(u32 adress,int port)
{
	struct socket *person_socket = NULL;
	struct sockaddr_in saddr;
	int ret;
	DECLARE_WAIT_QUEUE_HEAD(recv_wait);
	ret = sock_create(AF_INET, SOCK_STREAM, IPPROTO_TCP, &person_socket);
	if(ret<0) 
		{
		printk(KERN_INFO "Could not connect to socket\n");
		return NULL;
		}
	memset(&saddr, 0, sizeof(saddr));
        saddr.sin_family = AF_INET;
        saddr.sin_port = htons(port);
        saddr.sin_addr.s_addr = htonl(adress);

        ret = person_socket->ops->connect(person_socket, (struct sockaddr *)&saddr , sizeof(saddr), O_RDWR);
	if(ret && (ret != -EINPROGRESS))
		{
		printk(KERN_INFO "Could not connect to person\n");
		return NULL;
		}
	return person_socket;
}

int person_send(people *contact,int cmd, int prot_id,char *ad_mes)
{
	int res;
	comHeader comH;
	comH.version=0;
	comH.my_name=PORT;
	comH.my_id=PORT;
	comH.other_name=contact->name;
	comH.other_id=contact->id;
	comH.type=cmd;
	comH.identifier=prot_id;
	if(cmd==com_message)
		comH.extra_length=sizeof(ad_mes);
	else
		comH.extra_length=0;
	
	//res=tcp_client_send(contact->sock, comH.str,COM_LENGTH, MSG_DONTWAIT);
	if(sizeof(ad_mes)>0||1)
		{
		res=tcp_client_send(contact->sock, ad_mes,sizeof(ad_mes), MSG_DONTWAIT);
		}
	printk("message was sent %d\n",res);
	return 0;
}
int person_recv(people *contact)
{
	int reply_int;
	comHeader comH;
	reply_int =tcp_client_receive(conn_socket, comH.str,MSG_DONTWAIT,COM_LENGTH);
	if(reply_int<0)
		{
		printk(KERN_INFO "No message recv %d\n",reply_int);
		return -1;
		}
	printk("Message recieved from: %d,with type:%d\n",comH.my_id,comH.type);
	return 0;
}
*/
//end communication between people


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



#define usr_identity   11
#define noti_f 1
typedef struct {
	int task;
	int first;
	int second;
	int third;
}command;
typedef struct {
	char name[100];
	uint16_t app_id;
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
#define max_prog 50
#define amount_qubits 10
//typedef enum { false, true } bool;
#define	COM_LENGTH      34
typedef union
{
	struct
	{
		uint8_t version;
		uint32_t send_name;
		uint16_t send_id;//prog_id
		uint32_t recv_name;
		uint16_t recv_id; //prog_id
		uint8_t type;	
		uint64_t identifier; //the input of the ms
		uint64_t ms;
		uint32_t extra_length;
	} __attribute__((__packed__));
	char str[COM_LENGTH];		/* Additional details for this command */
} __attribute__((__packed__)) comHeader;
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
cqcHeader cqcH;
cmdHeader cmdH;
xtraCmdHeader xtra;
command cur;
#define CQC_OPT_NOTIFY 		0x01
#define CQC_OPT_BLOCK           0x04

int OS_next_p(OS *self);
int OS_sleep_p(OS *self,int prog_num);

int perform_cmd(OS *self)
{	
	Progs *prog;
	int index=OS_next_p(self);
	if(index==-1)
		{
		//printk(KERN_INFO "No task found in this round");
		return -1;
		}
	prog=self->Running[index];
	printk(KERN_INFO "Performing Task by %s,app_id:%d,line:%d with commands: %d,%d,%d",prog->name,prog->app_id,prog->cur,prog->code[prog->cur]->task,prog->code[prog->cur]->first,prog->code[prog->cur]->second);
	if (prog->cur >= prog->size)
		return -3;
	cur=*(prog->code[prog->cur]);
	//printk(KERN_INFO "Taks started:%d, %d\n",prog->cur,cur.task);
	//cqc_task
	
	if(cur.task==cmd_if) //if statement
		{
			if(prog->codevar[cur.first])
				(prog->cur)+=2; // If true the second line is executed
			return 0;
			
		}
	else if (cur.task==cmd_goto)
		prog->cur=cur.first;
	else if (cur.task==cmd_done)
		{
			printk(KERN_INFO "Task %s is finished, the results are: %d,%d",prog->name, prog->codevar[0],prog->codevar[1]);
			OS_sleep_p(self,index);
			return 0;
		}
	if(cur.task==cmd_recvc)
		{
			OS_sleep_p(self,index);
			++(prog->cur);
			return 0;
		}
	if(cur.task==cmd_sendc)
		{
			int id;
			comHeader comH;
			printk("sending classical infos\n");
			comH.version=5;
			comH.send_name=dev_int_name;
			comH.send_id=prog->app_id;
			
			id=get_id_of_dev_id(cur.first);
			if(id<0)
				printk("could not find the person\n");
			comH.recv_name=regist[id]->name_num;
			comH.recv_id=cur.third;
			comH.ms=prog->codevar[cur.second];
			ksocket_send(regist[id]->sock, regist[id]->addr, comH.str, sizeof(comH.str));
		}
	cqcH.version = 0;
	cqcH.app_id = prog->app_id;
	cqcH.type = 1; //command
	
	++(prog->priority);
	
	cmdH.qubit_id=cur.first;
	cmdH.instr=cur.task;
	cmdH.options=noti_f*CQC_OPT_NOTIFY+CQC_OPT_BLOCK;// determines notify, block, action Add as additional!!
	

	
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
	else if(cur.task==cmd_meas ||cur.task==cmd_meas_rm)  //measurement of qubit
		{
			
			cqcH.length = CQC_CMD_HDR_LENGTH;
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
			
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
			//xtra.remote_node=htonl(create_address(destip));
			xtra.remote_node=create_address(destip);
			xtra.remote_node=saddr.sin_addr.s_addr;
			xtra.remote_node=2130706433;
			//xtra.remote_port=htons(8820+cur.second);
			xtra.remote_port=8820+cur.second;
			xtra.remote_app_id=cur.third;
			printk("trying to send cmd:%d,ip:%d,port:%d\n",cur.task,xtra.remote_node,xtra.remote_port);
			tcp_client_send(conn_socket, cqcH.str,CQC_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, cmdH.str,CQC_CMD_HDR_LENGTH , MSG_DONTWAIT);
			tcp_client_send(conn_socket, xtra.str,CQC_CMD_XTRA_LENGTH , MSG_DONTWAIT);
		}
	++(prog->cur);
	if(noti_f)
		{
			OS_sleep_p(self,index);
		}
	else
		{
		}

	
	return 0;	 
}














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
	int i=-1;
	int k;
	for(k=0;k<max_prog;++k)
	{
		if(self->used_r[k])
		{
			if(i==-1) i=k;
			else if(self->g_prio[k]<self->g_prio[i])//get minimum
				i=k;
		}
	}
	if(i==-1)
		return -1;
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
	int i;
	for(i=0; i< max_prog;++i)
	{
		if(!self->used_s[i])
		{
			self->Sleeping[i]=self->Running[prog_num];
			self->used_s[i]=1;
			self->Running[prog_num]=NULL;
			self->used_r[prog_num]=0;
			return i;
		}
	}
	return -1;
}
int OS_find_appid(OS *self,uint16_t app_id)
{
	//int i=kmalloc(sizeof(int),GFP_KERNEL);
	int i;
	//printk("try to find app_id %d",app_id);
	for(i=0; i< max_prog;++i)
	{
		if(self->used_s[i])
		{
			if (self->Sleeping[i]->app_id==app_id)
				return i;
		}
	}
	return -1;
}
int OS_wakeup_p(OS *self,int prog_num)
{
	//int i=kmalloc(sizeof(int),GFP_KERNEL);
	int i=0;
	for(i=0; i< max_prog;++i)
	{
		if(!self->used_r[i])
		{
			self->Running[i]=self->Sleeping[prog_num];
			self->used_r[i]=1;
			self->Sleeping[prog_num]=NULL;
			self->used_s[prog_num]=0;
			self->g_prio[i]=10;
			return i;
		}
	}
	return -1;
}
int OS_run_p(OS *self)
{
	int res=perform_cmd(self);
	
	if(res<0)
	{
		//printk(KERN_INFO "Running Task has failed");
		return -1;
	}
	return 1;
	
}
int OS_givenext_p(OS *self)
{
	int i=-1;
	int k;
	for(k=0;k<max_prog;++k)
	{
		if(self->used_n[k])
		{
			if(i==-1)
				i=k;
			else if(self->Not_started[k]->priority > self->Not_started[i]->priority)//get max
				i=k;
		}
	}
	if(i==-1) return -1;
	return i;
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



cqcHeader cqcH_ret;
notifyHeader cqcN_ret;
#define CQC_TP_HELLO            0       /* Alive check */
#define CQC_TP_COMMAND          1       /* Execute a command list */
#define CQC_TP_FACTORY          2       /* Start executing command list repeatedly */
#define CQC_TP_EXPIRE           3       /* Qubit has expired */
#define CQC_TP_DONE             4       /* Command execution done */
#define CQC_TP_RECV             5       /* Recevied qubit */
#define CQC_TP_EPR_OK           6       /* Created EPR pair */
#define CQC_TP_MEASOUT          7       /* Measurement outcome */
#define CQC_TP_GET_TIME         8       /* Get creation time of qubit */
#define CQC_TP_INF_TIME         9       /* Inform about time */

#define CQC_ERR_GENERAL         20      /* General purpose error (no details */
#define CQC_ERR_NOQUBIT         21      /* No more qubits available */
#define CQC_ERR_UNSUPP          22      /* Command sequence not supported */
#define CQC_ERR_TIMEOUT         23      /* Timeout */
#define CQC_ERR_INUSE           24      /* Qubit ID in use when requesting new ID */

int cqc_response(OS *self)
{
	int res;	
	res =tcp_client_receive(conn_socket, cqcH_ret.str,MSG_DONTWAIT,sizeof(cqcH_ret.str));
	if(res<0)
		{
		//printk(KERN_INFO "No message recv\n");
		return -1;
		}
	//printk(KERN_INFO "res: %d, data: ver:%d,type:%d,app_id:%d, length: %d\n",res,cqcH_ret.version,cqcH_ret.type,cqcH_ret.app_id,cqcH_ret.length);
	if(cqcH_ret.length>0)
	{
		//msleep(200);
		//printk("second header");
		res =tcp_client_receive(conn_socket, cqcN_ret.str,MSG_DONTWAIT,sizeof(cqcN_ret.str));
		//printk(KERN_INFO "Notify res: %d, data: id:%d,outcome:%d,datetime:%d, remodteappid: %d \n",res,cqcN_ret.qubit_id,cqcN_ret.outcome,(int)(cqcN_ret.datetime),(int)(cqcN_ret.remote_app_id));
	}
	//printk(KERN_INFO "TRY TO FIND THE ID\n");
	res=OS_find_appid(self,cqcH_ret.app_id);
	//printk(KERN_INFO " Found prog %d for id %d",res,cqcH_ret.app_id);

	if(cqcH_ret.type==CQC_TP_HELLO) //just a simple ping 
		{
		if(noti_f)
			OS_wakeup_p(self,res);
		return 0;
		}

	if(res==-1)
		{
		printk(KERN_INFO "Recv MS for unknown app_id:%d",(int)(cqcH_ret.app_id));
		return -2;
		}
	
	if (cqcH_ret.type==CQC_TP_DONE)
		{

			OS_wakeup_p(self,res);
		}
	else if (cqcH_ret.type==CQC_TP_MEASOUT)
		{
			self->Sleeping[res]->codevar[self->Sleeping[res]->code[self->Sleeping[res]->cur -1]->second]=cqcN_ret.outcome;
			if(!noti_f)
				OS_wakeup_p(self,res);
			
		}
	else if (cqcH_ret.type==CQC_ERR_INUSE)
		{
			printk(KERN_INFO "Requested qubit already in use. app_id: %d",(int)(cqcH_ret.app_id));
			if(noti_f)
				OS_wakeup_p(self,res);
		}	
	
	return 0;
}

























#define com_identity		0
#define com_hello 		1
#define com_ping 		2
#define com_ping_return		3
#define com_message		4

#define com_start_prot		10
#define com_send_int		11


comHeader comH;

int classical_recv(OS *self)
{
	//printk("staring classical recv\n");
	int size,id,res;
	//int bufsize = 100;
        //unsigned char buf[bufsize+1];
	size = ksocket_receive(sock, &addr, comH.str, sizeof(comH.str));
	//printk("smth was recv%d\n ",size);
	if(size>0)
		{
		id=get_id_of_ip(addr.sin_addr.s_addr);
		if(id==-1)
			{
			id=get_empty_id();
			printk("gotten new id%d\n",id);
			regist[id]=connect_to_ip(addr.sin_addr.s_addr,comH.send_name);
			}
		printk("Inaddr: current s_addr:%d,id:%d", addr.sin_addr.s_addr,id);

		printk("data: version %d,prog_id %d, cmd %d, identifier: %d,ms: %d\n",comH.version,comH.recv_id,comH.type,comH.identifier,comH.ms);
		if( comH.send_name!=regist[id]->name_num)
			{
			printk("name does not fit or is new. change %d to %d",regist[id]->name_num,comH.send_name);
			regist[id]->name_num=comH.send_name;
			}
		if(comH.type==com_send_int)
			{
			res=OS_find_appid(self,comH.recv_id);
			if(self->Sleeping[res]->code[self->Sleeping[res]->cur-1]->first!=comH.send_name)
				printk("The id of the programs do not match. Try to continue however: %d,%d\n",self->Sleeping[res]->code[self->Sleeping[res]->cur-1]->first ,comH.send_name);
			self->Sleeping[res]->codevar[self->Sleeping[res]->code[self->Sleeping[res]->cur -1]->second]=comH.ms;
			OS_wakeup_p(self,res);
			}
		return 1;
		}
	return -1;	
}

/*int add_person(struct socket *sock)
{
	
	com.version=0;
	com.my_name=PORT%10;
	com.my_id=PORT;
	com.other_name=0;
	tcp_client_send(sock,com.str, sizeof(com.str), MSG_DONTWAIT);
	return 0;
}*/
command my_c0;
command my_c1;
command my_c2;
command my_c3;
Progs p;
OS my_os;
int prog_id;
//char cqc_back_cmd[100];
cqcHeader cqc_back_cmd;
#define waiting 1
char test_str[200];
char test_strr[500];
int test_i,test_res;
comHeader com_test;
int looping(void)
{	
	struct socket *test_con;
	u8 ip[]={192,168,1,3-dev_int_name,'\0'};
	int lauf;
	//int id;
	int run;
	run=socket_open();
	printk("sock_con res:%d\n",run);
	regist=kmalloc(sizeof(struct con_infos *)*max_con, GFP_KERNEL);
	for(run=0; run<max_con;++run)
		regist[run]=NULL;
	regist[0]=connect_to_ip(htonl(create_address(ip)),ip[3]);
	
	run=0;
	for(lauf=0;lauf<20*20*50+(1-waiting)*10000000;++lauf)
	{
		if(kthread_should_stop()) 
			{
			do_exit(0);
			}		
		if(waiting) msleep(50);
		if(lauf %(40 +(1-waiting)*100000) == 0) 
			{
			printk("starting round %d,it is in: NS %d%d%d%d , RUN %d%d%d%d , SL %d%d%d%d \n",lauf,my_os.used_n[0],my_os.used_n[1],my_os.used_n[2],my_os.used_n[3],my_os.used_r[0],my_os.used_r[1],my_os.used_r[2],my_os.used_r[3],my_os.used_s[0],my_os.used_s[1],my_os.used_s[2],my_os.used_s[3]);
			print_con();
			}
	   	if(OS_run_p(&my_os)<0)
			{
			//printk("no task performed in this round,run: %d\n",run);
			run+=1;
			}
		
		run=OS_givenext_p(&my_os);
		if(run>-1)
			{
			OS_start_p(&my_os,run);
			OS_make_prio(&my_os);
			}
		run=0;
			
		//msleep(100);
		classical_recv(&my_os);
		cqc_response(&my_os);
	}
	return 1;
}
/*void prog(void)
{
	int i,id,size;
	int bufsize = 100;
        unsigned char buf[bufsize+1];
	allow_signal(SIGKILL);
        current->flags |= PF_NOFREEZE;
	regist=kmalloc(sizeof(struct con_infos *)*max_con, GFP_KERNEL);
	for(i=0;i<max_con;++i)
		regist[i]=NULL;
	socket_open();
	
	for(i=0;i<1000;++i)
	{
		memset(&buf, 0, bufsize+1);
                size = ksocket_receive(sock, &addr, buf, bufsize);

		if(size>0)
		{
			id=get_id_of_ip(addr.sin_addr.s_addr);
			if(id==-1)
				{
				id=get_empty_id();
				printk("gotten new id%d\n",id);
				regist[id]=connect_to_ip(addr.sin_addr.s_addr);
				}
			printk("Inaddr: current s_addr:%d,id:%d", addr.sin_addr.s_addr,id);
			//printk(KERN_INFO MODULE_NAME": received %d bytes\n", size);
		        printk("data: %s\n", buf);
		}
	}
	printk("Prog over\n");
}*/

struct task_struct *main_thread;

static int __init ebbchar_init(void)
{
   int i;
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
   if (IS_ERR(ebbcharClass)){                // Check for erro
      return PTR_ERR(ebbcharDevice);
   }
   ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
      class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(ebbcharDevice);
   }
   printk(KERN_INFO "EBBChar: device class created correctly\n"); // Made it! device was initialized
	//network stuff
   printk(KERN_INFO "CL:establish TCP connection\n");
   tcp_client_connect();
   OS_init(&my_os);
   
   net_people=kmalloc(MAX_CONNS*sizeof(people *),GFP_KERNEL);
   for(i=0; i<MAX_CONNS;++i)
	net_people[i]=NULL;
   printk("connection established");

   main_thread=kthread_run((void *)looping, NULL,DEVICE_NAME);
   


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
	if(sock!=NULL)
		sock_release(sock);
	printk(KERN_INFO "socket released");
	kfree(response);
	kfree(reply);
	kfree(p.code);
	printk(KERN_INFO "half is released,pointer: %p\n",main_thread);
	/*if(main_thread!=NULL)
		kthread_stop(main_thread);
	else
		printk("task already terminated");*/
	/*kfree(my_os.Running);
	kfree(my_os.Sleeping);
	kfree(my_os.Not_started);*/
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
union Get_prog{
	struct{
		char name[100];
		int prio;
		int size;
		command cmds[100];
		}__attribute__((__packed__));
	char str[100+4+4+100*16];
	};
Progs *create_prog(union Get_prog *prog_input,uint16_t app_id)
{
	int i;
	Progs *new_prog=kmalloc(sizeof(Progs),GFP_KERNEL);
	new_prog->code=kmalloc(sizeof(command *)  *  prog_input->size,GFP_KERNEL);
	strcpy(new_prog->name,prog_input->name);
	new_prog->priority=prog_input->prio;
	new_prog->size=prog_input->size;
	new_prog->cur=0;
	new_prog->app_id=app_id;
	for(i=0;i<prog_input->size;++i)
		new_prog->code[i]=&(prog_input->cmds[i]);
	return new_prog;
}
int str_rep( char *into,const char *from,int len)
{
	int i;
	for(i=0; i<len;++i)
		into[i]=from[i];
	return 0;
}
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
   union Get_prog *prog_shell=kmalloc(sizeof(union Get_prog),GFP_KERNEL);
   Progs *new_prog;
   
   str_rep(prog_shell->str,buffer,len); 
   printk("shell prio:%d\n",prog_shell->prio);
   new_prog=create_prog(prog_shell,300+numberOpens);
   //kfree(prog_shell);
   printk("new prog,%p \n",new_prog);
   printk("New prog created, prio: %d,size:%d,app_id%d\n",new_prog->priority,new_prog->size,new_prog->app_id);
   OS_insert_p(&my_os,new_prog);
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
