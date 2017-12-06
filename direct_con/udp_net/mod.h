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
struct con_infos *connect_to_ip(unsigned long address);
int get_id_of_ip(unsigned long address);
int get_empty_id(void);
