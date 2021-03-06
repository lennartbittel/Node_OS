#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>


#include <linux/net.h>
#include <linux/socket.h>

#include <linux/errno.h>
#include <linux/types.h>

#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>

#include <linux/delay.h>

#define DEFAULT_PORT 8852
#define CONNECT_PORT DEFAULT_PORT
#define MODULE_NAME "ksocket2"
#define INADDR_SEND ((unsigned long int)0xC0A80102) /* 127.0.0.1 */
//#define INADDR_SEND INADDR_LOOPBACK

/*
        2006/06/27 - Added ksocket_send, so, after receive a packet, the kernel send another back to the CONNECT_PORT
                - Rodrigo Rubira Branco <rodrigo@kernelhacking.com>

        2006/05/14 - Initial version
                - Toni Garcia-Navarro <topi@phreaker.net>
*/
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
struct kthread_t
{
        struct task_struct *thread;
        struct socket *sock;
        struct sockaddr_in addr;
        struct socket *sock_send;
        struct sockaddr_in addr_send;
        int running;
};

struct kthread_t *kthread = NULL;

/* function prototypes */
int ksocket_receive(struct socket *sock, struct sockaddr_in *addr, unsigned char *buf, int len);
int ksocket_send(struct socket *sock, struct sockaddr_in *addr, unsigned char *buf, int len);
struct socket *sock;
struct sockaddr_in addr;
int socket_open(void)
{

	int  err;

	kthread->running = 1;
        current->flags |= PF_NOFREEZE;
	allow_signal(SIGKILL);
	if ( (err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock)) < 0)
	{
		printk(KERN_INFO MODULE_NAME": Could not create a datagram socket, error = %d\n", -ENXIO);
                goto out;
	}
	memset(&addr, 0, sizeof(struct sockaddr));
	addr.sin_family      = AF_INET;
	addr.sin_addr.s_addr      = htonl(INADDR_ANY);
	addr.sin_port      = htons(DEFAULT_PORT);
	if ( (err = sock->ops->bind(sock, (struct sockaddr *) &addr, sizeof(struct sockaddr) ) ) < 0)
        {
                printk(KERN_INFO MODULE_NAME": Could not bind or connect to socket, error = %d\n", -err);
                goto close_and_out;
        }
	printk(KERN_INFO MODULE_NAME": listening on port %d\n", DEFAULT_PORT);
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
        kthread->thread = NULL;
        kthread->running = 0;
	return -2;
}
struct con_infos{
	char name[10];
	int name_num;
	unsigned long ip;
	struct socket *sock;
	struct sockaddr_in *addr;
};
#define max_con 20
struct con_infos **regist;

struct con_infos *connect_to_ip(unsigned long address)
{
	int err;
	struct socket *sock_send;
        struct sockaddr_in addr_send;
	struct con_infos *infos=kmalloc(sizeof(struct con_infos), GFP_KERNEL);
	if((err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock_send)) < 0 )
	{
                printk(KERN_INFO MODULE_NAME": could not create socket for new person %d\n", -ENXIO);
                return NULL;
        }
	memset(&addr_send, 0, sizeof(struct sockaddr));
	addr_send.sin_family = AF_INET;
	addr_send.sin_addr.s_addr = address;
	addr_send.sin_port = htons(DEFAULT_PORT);	
	if(( err = sock_send->ops->connect(sock_send, (struct sockaddr *)&addr_send, sizeof(struct sockaddr), 0) )< 0 )
	{
	        printk(KERN_INFO MODULE_NAME": Could not connect to IP,err:%d\n", -err);
                goto close_and_out;
	}
	infos->sock=sock_send;
	infos->addr=&addr_send;
	infos->ip=address;
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
int get_empty_id(void)
{
	int i;
	for(i=0;i<max_con;++i)
		if(regist[i]==NULL)
			return i;
	return max_con;
}
void prog(void)
{
	int i,id,size;
	int bufsize = 100;
        unsigned char buf[bufsize+1];
        kthread->running = 1;
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

			
			
		if (signal_pending(current))
                        break;
		msleep(50);
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
	kthread->running =0;
}
static void ksocket_start(void)
{
        int size, err;
        int bufsize = 100;
        unsigned char buf[bufsize+1];

        /* kernel thread initialization */
        //lock_kernel();
        kthread->running = 1;
        current->flags |= PF_NOFREEZE;

        /* daemonize (take care with signals, after daemonize() they are disabled) */
        //daemonize(MODULE_NAME);
        allow_signal(SIGKILL);
        ///unlock_kernel();

        /* create a socket */
        if ( ( (err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &kthread->sock)) < 0) ||
             ( (err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &kthread->sock_send)) < 0 ))
        {
                printk(KERN_INFO MODULE_NAME": Could not create a datagram socket, error = %d\n", -ENXIO);
                goto out;
        }

        memset(&kthread->addr, 0, sizeof(struct sockaddr));
        memset(&kthread->addr_send, 0, sizeof(struct sockaddr));
        kthread->addr.sin_family      = AF_INET;
        kthread->addr_send.sin_family = AF_INET;

        kthread->addr.sin_addr.s_addr      = htonl(INADDR_ANY);
        kthread->addr_send.sin_addr.s_addr = htonl(INADDR_SEND);

        kthread->addr.sin_port      = htons(DEFAULT_PORT);
        kthread->addr_send.sin_port = htons(CONNECT_PORT);

        if ( ( (err = kthread->sock->ops->bind(kthread->sock, (struct sockaddr *)&kthread->addr, sizeof(struct sockaddr) ) ) < 0) ||
               (err = kthread->sock_send->ops->connect(kthread->sock_send, (struct sockaddr *)&kthread->addr_send, sizeof(struct sockaddr), 0) < 0 ))
        {
                printk(KERN_INFO MODULE_NAME": Could not bind or connect to socket, error = %d\n", -err);
                goto close_and_out;
        }

        printk(KERN_INFO MODULE_NAME": listening on port %d\n", DEFAULT_PORT);

        /* main loop */
        for (;;)
        {
                memset(&buf, 0, bufsize+1);
                size = ksocket_receive(kthread->sock, &kthread->addr, buf, bufsize);

                if (signal_pending(current))
                        break;

                if (size < 0)
                        printk(KERN_INFO MODULE_NAME": error getting datagram, sock_recvmsg error = %d\n", size);
                else 
                {
                        printk(KERN_INFO MODULE_NAME": received %d bytes\n", size);
                        /* data processing */
                        printk("\n data: %s\n", buf);
                        /* sending */
                        memset(&buf, 0, bufsize+1);
                        strcat(buf, "testing...");
                        ksocket_send(kthread->sock_send, &kthread->addr_send, buf, strlen(buf));
                }
        }

close_and_out:
        sock_release(kthread->sock);
        sock_release(kthread->sock_send);
        kthread->sock = NULL;
        kthread->sock_send = NULL;

out:
        kthread->thread = NULL;
        kthread->running = 0;
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

int __init ksocket_init(void)
{
        kthread = kmalloc(sizeof(struct kthread_t), GFP_KERNEL);
        memset(kthread, 0, sizeof(struct kthread_t));

        /* start kernel thread */
        //kthread->thread = kthread_run((void *)ksocket_start, NULL, MODULE_NAME);
	kthread->thread = kthread_run((void *)prog, NULL, MODULE_NAME);
        if (IS_ERR(kthread->thread)) 
        {
                printk(KERN_INFO MODULE_NAME": unable to start kernel thread\n");
                kfree(kthread);
                kthread = NULL;
                return -ENOMEM;
        }

        return 0;
}

void __exit ksocket_exit(void)
{
        int err;

        if (kthread->thread==NULL)
                printk(KERN_INFO MODULE_NAME": no kernel thread to kill\n");
        else 
        {
                //lock_kernel();
                err = my_kill_proc(kthread->thread->pid, SIGKILL);
                //unlock_kernel();

                /* wait for kernel thread to die */
                if (err < 0)
                        printk(KERN_INFO MODULE_NAME": unknown error %d while trying to terminate kernel thread\n",-err);
                else 
                {
                        //while (kthread->running == 1)
                                msleep(10);
                        printk(KERN_INFO MODULE_NAME": succesfully killed kernel thread!\n");
                }
        }

        /* free allocated resources before exit */
        if (sock != NULL) 
        {
                sock_release(sock);
                sock = NULL;
        }

        kfree(kthread);
        kthread = NULL;

        printk(KERN_INFO MODULE_NAME": module unloaded\n");
}

/* init and cleanup functions */
module_init(ksocket_init);
module_exit(ksocket_exit);

/* module information */
MODULE_DESCRIPTION("kernel thread listening on a UDP socket (code example)");
MODULE_AUTHOR("Toni Garcia-Navarro <topi@phreaker.net>");
MODULE_LICENSE("GPL");
