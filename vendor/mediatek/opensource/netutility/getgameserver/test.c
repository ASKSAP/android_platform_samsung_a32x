/* Test  */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include "cutils/properties.h"


//#define PATH "/vendor/bin/getgameserver"

#define PATH "/data/local/tmp/getgameserver"
//property  set
#define NETDAGENT_VND_GAMESERVER "vendor.netdagent.gameserver"


#define LOG_TAG "PacketMonitorTest"
#include "log/log.h"

pid_t m_pid = -1 ;

static void sig_handler(int sig) {
 	   
	/* master process */ 	     
		 
	if (sig == SIGINT){		 
	  ALOGI("sig_handler:SIGQUIT CTR+C from keyboard \n");
		if(m_pid>0)
			kill(m_pid,SIGUSR2) ;
        exit(-232);
	}else if (sig == SIGTERM){
       ALOGI("sig_handler: SIGTERM \n");
	   if(m_pid>0)
	   	kill(m_pid,SIGUSR2) ;
       exit(-233);
	}	
	else			 
		return;		
}

static void setup_signal_handling() {    
	struct sigaction sigact;    
	sigact.sa_handler = sig_handler;    
	sigact.sa_flags = 0;    
	sigemptyset(&sigact.sa_mask);    
	sigaction(SIGINT, &sigact, NULL);  
	sigaction(SIGTERM, &sigact, NULL);

}




static int find_process_id(char * name){


	DIR *dir;
	struct dirent *ptr;
	FILE *fp;
	char filepath[50];
    char filetext[50];
    int pid ;
	dir = opendir("/proc") ;
	if(NULL!=dir){

		while(NULL !=(ptr=readdir(dir))){
			if((0==strcmp(ptr->d_name,"."))||(0==strcmp(ptr->d_name,"..")))
				continue ;
			if(DT_DIR !=ptr->d_type)
				continue ;
			sprintf(filepath ,"/proc/%s/cmdline",ptr->d_name);
			fp=fopen(filepath,"r");
			if(NULL!=fp){
				fread(filetext,1,50,fp);
				filetext[49]='\0' ;
				if(NULL !=strstr(filetext,name)){
                	ALOGI ("%s process PID %s" ,name,ptr->d_name);
					pid = atoi(ptr->d_name);
					fclose(fp) ;
					closedir(dir);
                    return pid ;
				}
			fclose(fp) ;
			}
		
		}
		
    closedir(dir);	
	}else{
     
      ALOGE ("opendir /proc fail" );
	}

 return -1 ;
 
}

int main(int argc, char **argv) {

  
    char * name = "getgameserver" ;
    char value[PROPERTY_VALUE_MAX];
	setup_signal_handling();
	m_pid =(pid_t) find_process_id(name);

    if(m_pid > 0)
    	ALOGI("@1.1 start test getgameserver@ pid %d  " , m_pid );
	else
		ALOGI(" can't get gameserver pid   " );
	
    while(1) {			
          	  kill(m_pid,SIGUSR1) ;
		  sleep(30);
		  property_get(NETDAGENT_VND_GAMESERVER, value,NULL);
		  ALOGI(" Game Server IP: %s   ",value );
		  kill(m_pid,SIGUSR2) ;
		  sleep(20);
        }
             
    return 0;
}



