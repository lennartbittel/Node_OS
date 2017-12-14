#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <sys/time.h>
int main()
{
int i;
pid_t my=getpid();
struct sched_param sp,spnow;
struct timeval time;
sp.sched_priority=99;
i=sched_getparam(0, &spnow);

printf("prio:%d,%d,%d\n",spnow.sched_priority,i,sched_get_priority_max(SCHED_RR));

printf("pid: %d,sched:%d,res: %d\n",my,sched_getscheduler(my),i);
gettimeofday( &time, 0 );
long start_t = 1000000 * time.tv_sec + time.tv_usec;
for(int j=0;j<10000000000;++j)
	i+=1;
gettimeofday( &time, 0 );
long inter_t = 1000000 * time.tv_sec + time.tv_usec;
i=sched_setscheduler(my, SCHED_RR, &sp);
for(int j=0;j<10000000000;++j)
	i+=1;
gettimeofday( &time, 0 );
long end_t = 1000000 * time.tv_sec + time.tv_usec;
printf("time it takes in us: %ld= %ld+ %ld\n",end_t-start_t,inter_t-start_t,end_t-inter_t);
printf("%d\n",i);
}
