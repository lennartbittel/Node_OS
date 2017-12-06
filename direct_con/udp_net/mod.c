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
#include "mod.h"



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


struct con_infos *connect_to_ip(unsigned long address)
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

