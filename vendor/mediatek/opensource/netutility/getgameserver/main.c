#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>


#include <pcap.h>
#include <pcap/pcap.h>

#include <signal.h>
#include <pthread.h>
//#include <curses.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <locale.h>
#include <netinet/ip6.h>
#include <arpa/inet.h>


#include "cutils/properties.h"
#include "ip.h"
#include "tcp.h"
#include "sll.h"
#include "hash.h"
#include "addr_hash.h"
#include "main.h"


#define LOG_TAG "PacketMonitor"
#include "log/log.h"

//thread var
bool pcap_thread_start = false ;
pthread_t pcap_thread ;

//recorder var
hash_type* history;
history_type history_totals;

//timer var
pthread_mutex_t tick_mutex;
time_t last_timestamp;


//pcap lib var
pcap_t* pd; /* pcap descriptor */
struct bpf_program pcap_filter;
pcap_handler packet_handler;
#define CAPTURE_LENGTH 92 

//property  set
#define NETDAGENT_VND_GAMESERVER "vendor.netdagent.gameserver"
 


#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif


typedef struct{
	addr_pair ap ;
	double long tput;
} result_type ;


static void sig_handler(int sig) {
 	   
	/* master process */ 	     
		 
	if (sig == SIGUSR1)			 
		start_pcap_thread();		
	else if (sig == SIGUSR2)			
		stop_pcap_thread();	
	else if (sig == SIGQUIT){
        ALOGI("sig_handler: thread exit \n"); 
        pthread_exit(NULL);
	}			
	else			 
		return;		
}

static void setup_signal_handling() {    
	struct sigaction sigact;    
	sigact.sa_handler = sig_handler;    
	sigact.sa_flags = 0;    
	sigemptyset(&sigact.sa_mask);    
	sigaction(SIGUSR1, &sigact, NULL);  
	sigaction(SIGUSR2, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
}


static void init_history() {
    history = addr_hash_create();
    last_timestamp = time(NULL);
    memset(&history_totals, 0, sizeof history_totals);
}


static void analyze_data(hash_node_type* n ,double long  * max_tput  ,result_type * result){

 history_type* d  ;
 double long tput;
 addr_pair* ap = (addr_pair*)n->key ;
 	
 d = (history_type*)n->rec ;
 tput =d->total_sent + d->total_recv ;

 if((ap->protocol==IPPROTO_UDP)&&(tput > *max_tput)){
  result->ap = *(addr_pair*)n->key ;
  result->tput = tput ;
  *max_tput = tput ;  

  //debug 
  /*
  if( result->ap.af == AF_INET)
  	ALOGI("analyze_data IPV4 : %x(%x) -> %x(%x)  " ,result->ap.src,result->ap.src_port,result->ap.dst,result->ap.dst_port) ;
  else if ( result->ap.af == AF_INET6)
  	ALOGI("analyze_data IPV6 : %x(%x) -> %x(%x)  " ,result->ap.src6,result->ap.src_port,result->ap.dst6,result->ap.dst_port) ;
  else
	ALOGE("analyze_data: unknown ip type for game service ") ;
 //debug  end
  */
 }
 
}
static void find_server_ip(){

	int i;
    hash_node_type *n, *nn;
	double long max_tput = 0 ;
    result_type ippair_result ;
    char addr_p[INET6_ADDRSTRLEN] ;
	
    if(history == 0) {
      return;
    }

    memset(&ippair_result,0,sizeof(ippair_result)) ;
	
    for(i = 0; i < history->size; i++){
        n = history->table[i];
        while(n != NULL) {
            nn = n->next;
			analyze_data(n ,&max_tput,&ippair_result);			
            n = nn;
        }
	}
    memset(addr_p,0,INET6_ADDRSTRLEN) ;

    if( ippair_result.ap.af == AF_INET){
		if(NULL== inet_ntop(AF_INET,&(ippair_result.ap.dst) ,addr_p,INET6_ADDRSTRLEN)){
             ALOGE("inet_ntop return fail for IPV4   ") ;
		}         
		
	}else if( ippair_result.ap.af == AF_INET6) {
       if(NULL== inet_ntop(AF_INET6,&(ippair_result.ap.dst6) ,addr_p,INET6_ADDRSTRLEN)){
            ALOGE("inet_ntop return fail for IPV6   ") ;
	   }
	   	
	}else{
	   ALOGE("unknow ip type for game service   ") ;
	   return ;
	}
	
	ALOGI("Game Server IP : %s:(Port: %d->%d) " ,addr_p ,ippair_result.ap.src_port,ippair_result.ap.dst_port) ;	
    property_set(NETDAGENT_VND_GAMESERVER, addr_p); 

}


static history_type* history_create() {
    history_type* h;
    h = xcalloc(1, sizeof *h);
    return h;
}




static void tick(int debug) {
    time_t t;
    pthread_mutex_lock(&tick_mutex);
   
    t = time(NULL);
    if(t - last_timestamp >= RESOLUTION) { 
	find_server_ip();
        last_timestamp = t;
        //clear  history data  ,we should use latest data to caculate the game server;
	hash_delete_all(history) ;
    }
    else{
   
    }

    pthread_mutex_unlock(&tick_mutex);
}


static void assign_addr_pair(addr_pair* ap, struct ip* iptr, int flip) {
  unsigned short int src_port = 0;
  unsigned short int dst_port = 0;

  /* Arrange for predictable values. */
  memset(ap, '\0', sizeof(*ap));

  if(IP_V(iptr) == 4) {
    ap->af = AF_INET;
  /* Does this protocol use ports? */
  if(iptr->ip_p == IPPROTO_TCP || iptr->ip_p == IPPROTO_UDP) {
    /* We take a slight liberty here by treating UDP the same as TCP */
    ap->protocol = iptr->ip_p ;  
    /* Find the TCP/UDP header */
    struct tcphdr* thdr = ((void*)iptr) + IP_HL(iptr) * 4;
    src_port = ntohs(thdr->th_sport);
    dst_port = ntohs(thdr->th_dport);
  }

  if(flip == 0) {
    ap->src = iptr->ip_src;
    ap->src_port = src_port;
    ap->dst = iptr->ip_dst;
    ap->dst_port = dst_port;
  }
  else {
    ap->src = iptr->ip_dst;
    ap->src_port = dst_port;
    ap->dst = iptr->ip_src;
    ap->dst_port = src_port;
  }
  } /* IPv4 */
  else if (IP_V(iptr) == 6) {
    /* IPv6 packet seen. */
    struct ip6_hdr *ip6tr = (struct ip6_hdr *) iptr;

    ap->af = AF_INET6;

    if( (ip6tr->ip6_nxt == IPPROTO_TCP) || (ip6tr->ip6_nxt == IPPROTO_UDP) ) {
      struct tcphdr *thdr = ((void *) ip6tr) + 40;

	  ap->protocol = ip6tr->ip6_nxt ;	  
      src_port = ntohs(thdr->th_sport);
      dst_port = ntohs(thdr->th_dport);
    }

    if(flip == 0) {
      memcpy(&ap->src6, &ip6tr->ip6_src, sizeof(ap->src6));
      ap->src_port = src_port;
      memcpy(&ap->dst6, &ip6tr->ip6_dst, sizeof(ap->dst6));
      ap->dst_port = dst_port;
    }
    else {
      memcpy(&ap->src6, &ip6tr->ip6_dst, sizeof(ap->src6));
      ap->src_port = dst_port;
      memcpy(&ap->dst6, &ip6tr->ip6_src, sizeof(ap->dst6));
      ap->dst_port = src_port;
    }
  }
}

static void handle_ip_packet(struct ip* iptr, int hw_dir, int pld_len){
    int direction = 0; /* incoming */
    int len;
    history_type* ht;
    union {
      history_type **ht_pp;
      void **void_pp;
    } u_ht = { &ht };
    addr_pair ap;
    struct in6_addr scribdst;   /* Scratch pad. */
    struct in6_addr scribsrc;   /* Scratch pad. */
    /* Reinterpret packet type. */
    struct ip6_hdr* ip6tr = (struct ip6_hdr *) iptr;

    memset(&ap, '\0', sizeof(ap));

     tick(0);
 
    if (pld_len < sizeof (struct ip))
	return;
    if (IP_V(iptr) == 6 && pld_len < sizeof (struct ip6_hdr))
	return;

    if( (IP_V(iptr) == 4) || (IP_V(iptr) == 6 ) ) { 
        /*
         * Net filter is off, so assign direction based on MAC address
         */
        if(hw_dir == 1) {
            /* Packet leaving this interface. */
            assign_addr_pair(&ap, iptr, 0);
            direction = 1;
        }
        else if(hw_dir == 0) {
            /* Packet incoming */
            assign_addr_pair(&ap, iptr, 1);
            direction = 0;
        }
               
        /* Drop other uncertain packages. */
        else{
             ALOGE("unknow packet's direction ") ;
			return;

		}
    }

   
    

#if 1
    /* Test if link-local IPv6 packets should be dropped. */
    if( IP_V(iptr) == 6  && (IN6_IS_ADDR_LINKLOCAL(&ip6tr->ip6_dst)
                || IN6_IS_ADDR_LINKLOCAL(&ip6tr->ip6_src)) )
        return;
#endif

   
    if(hash_find(history, &ap, u_ht.void_pp) == HASH_STATUS_KEY_NOT_FOUND) {
        ht = history_create();
        hash_insert(history, &ap, ht);
    }

	len = pld_len;

    /* Update record */

    if( ((IP_V(iptr) == 4) && (direction == 1))
       || ((IP_V(iptr) == 6) && (direction == 1)) ){

        ht->total_sent += len;
    }
    else if( ((IP_V(iptr) == 4) && (direction == 0))
       || ((IP_V(iptr) == 6) && (direction == 0)) )  {

        ht->total_recv += len;
    }

    if(direction == 0) {
        /* incoming */
        history_totals.total_recv += len;
    }
    else {
        history_totals.total_sent += len;
    }
    
}


static void handle_cooked_packet(unsigned char *args, const struct pcap_pkthdr * thdr, const unsigned char * packet)
{
    struct sll_header *sptr;
    int dir = -1;
    sptr = (struct sll_header *) packet;

    switch (ntohs(sptr->sll_pkttype))
    {
    case LINUX_SLL_HOST:
        /*entering this interface*/
	dir = 0;
	break;
    case LINUX_SLL_OUTGOING:
	/*leaving this interface */
	dir=1;
	break;
    }
    handle_ip_packet((struct ip*)(packet+SLL_HDR_LEN), dir,
		     thdr->len - SLL_HDR_LEN);
}



/* set_filter_code:
 * Install some filter code. Returns NULL on success or an error message on
 * failure. */
static char *set_filter_code(const char *filter) {
    char *x;
    if (filter) {
        x = xmalloc(strlen(filter) + sizeof "() and (ip or ip6)");
        sprintf(x, "(%s) and (ip or ip6)", filter);
    } else
        x = xstrdup("ip or ip6");
    if (pcap_compile(pd, &pcap_filter, x, 1, 0) == -1) {
        xfree(x);
        return pcap_geterr(pd);
    }
    xfree(x);
    if (pcap_setfilter(pd, &pcap_filter) == -1)
        return pcap_geterr(pd);
    else
        return NULL;
}



/*
 * packet_init:
 *
 * performs pcap initialisation, called before ui is initialised
 */
static void packet_init() {
    char errbuf[PCAP_ERRBUF_SIZE];
    char *m;
    int result;


    pd = pcap_open_live("any", CAPTURE_LENGTH, 0, 1000, errbuf);
    // DEBUG: pd = pcap_open_offline("tcpdump.out", errbuf);
    if(pd == NULL) { 
        ALOGE("pcap_open_live(%s): %s\n", "any", errbuf); 
        exit(1);
    }
     packet_handler = handle_cooked_packet; 
	 
   /* if ((m = set_filter_code(filtercode))) {
        fprintf(stderr, "set_filter_code: %s\n", m);
        exit(1);
        return;
    }*/
}

/* packet_loop:
 * Worker function for packet capture thread. */
static void packet_loop(void* ptr) {
    pcap_loop(pd,-1,(pcap_handler)packet_handler,NULL);
}


static void start_pcap_thread(){

   int ret ;    

ALOGI(" start_pcap_thread  \n");

if(pcap_thread_start == false){

	  last_timestamp = time(NULL);
	  ret = pthread_create(&pcap_thread, NULL, (void*)&packet_loop, NULL);
	  if(0 != ret) {
		ALOGE("fail to create event thread (err :%d)\n",ret);
		return ;
		}

		ALOGI("start pcap_thread successfully \n");
	  if(pthread_setname_np(pcap_thread, "pcap_thread") != 0){	  	
			ALOGE("fail to change pcap_thread thread name\n");
		}
		pcap_thread_start = true;
}else {

       ALOGI(" pcap_thread has  been created \n");
	}

}

static void stop_pcap_thread(){

   int ret ;
   void * retval ;

ALOGI(" stop_pcap_thread  \n");

if(pcap_thread_start == true){
	
	//ret=pthread_cancel(pcap_thread); //androidn doesn't support this API 
	ret=pthread_kill(pcap_thread ,SIGQUIT);
	
	if(0 != ret){
    	ALOGE(" pthread_cancel fail exit  code = %d\n" ,ret);
	}

	ALOGI(" wait for pcap thread exit \n");

	ret=pthread_join(pcap_thread, &retval);

	if(0 != ret){
   		ALOGE(" pthread_join fail exit  code = %d(%d) \n" ,ret,(int)retval);
	}

	pcap_thread_start = false;
	hash_delete_all(history) ;
	property_set(NETDAGENT_VND_GAMESERVER, NULL);
	
	
}else {

       ALOGI(" pcap_thread is not running  \n");
	}



}

/* main: */
int main(int argc, char **argv) {

   
    setup_signal_handling();
    pthread_mutex_init(&tick_mutex, NULL);
    packet_init();
    init_history(); 
    while(1) {			
          sleep(5);
		  
        }
          
  pcap_close(pd);

    
    return 0;
}
