#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#define DEV_PATH "/dev/input/event4"

int main()
{
int keys_fd;
char ret[2];
struct input_event t;
keys_fd=open(DEV_PATH,O_RDONLY);
if (keys_fd<=0)
{
printf("Open  device error!\n");
return -1;
}
bool up=0,down=0,left=0,right=0;
double vel=0,yaw=0;
while(1)
{
printf("1");
if (read(keys_fd,&t,sizeof(t))!=sizeof(t)) continue;
if (t.type!=EV_KEY) continue;
if (t.code==KEY_ESC) break;
if (t.code==KEY_LEFT) left=t.value;
if (t.code==KEY_RIGHT) right=t.value;
if (t.code==KEY_UP) up=t.value;
if (t.code==KEY_DOWN) down=t.value;

yaw=left?-1:right?1:0;
vel=up?-1:down?1:0;


printf("Control vel %.1lf,yaw%.1lf\n",vel,yaw);

}

close(keys_fd);
return 0;


}