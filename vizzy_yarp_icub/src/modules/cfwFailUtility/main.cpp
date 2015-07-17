/* system example : DIR */
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <yarp/os/LogStream.h>
#define MAXBUF 1024
static bool keepRunning = true;
void intHandler(int) {
    keepRunning = false;
}
int main ()
{
    char buffer[MAXBUF] = {0};
    char *pch;
    struct sigaction act;
    act.sa_handler = intHandler;
    //sigaction(SIGINT, &act, NULL);
    while (1){
	    sleep(60);
	    FILE *fd = popen("dmesg | grep \"Cfw rxfull0\"", "r");
	    if (NULL == fd)
	    {
		printf("Error in popen");
		return 0;
	    }
	    fread(buffer, MAXBUF, 1, fd);
	    printf("%s",buffer);
	    pch = strstr(buffer,"Cfw rxfull0");
	    if (pch==NULL)
		yDebug() << "Cfw running properly";
	    else
		yError() << "Cfw rxfull0 XXX error -- please restart the vizzy-desktop";
	    pclose(fd);
    }
}
