/*
 * iftop.h:
 *
 */

#ifndef __IFTOP_H_ /* include guard */
#define __IFTOP_H_


/*  10s  */
//#define HISTORY_LENGTH  20
#define RESOLUTION 15


/* At least OpenBSD and NexentaCore do not
 * define s6_addr32 for user land settings.
 */
#if !defined s6_addr32 && defined __sun__
#	define s6_addr32 _S6_un._S6_u32
#elif !defined s6_addr32 && \
		( defined __OpenBSD__ || defined __FreeBSD__ )
#	define s6_addr32 __u6_addr.__u6_addr32
#endif	/* !defined s6_addr32 */

typedef struct {
    double long total_sent;
    double long total_recv;
    int last_write;
} history_type;



void *xmalloc(size_t n);
void *xcalloc(size_t n, size_t m);
void *xrealloc(void *w, size_t n);
char *xstrdup(const char *s);
void xfree(void *v);
/*
static void setup_signal_handling();
static void init_history();
static void analyze_data(hash_node_type* n ,double long  * max_tput  ,result_type * result);
static void find_server_ip();
static history_type* history_create() ;
static void tick(int debug);
static void assign_addr_pair(addr_pair* ap, struct ip* iptr, int flip);
static void handle_ip_packet(struct ip* iptr, int hw_dir, int pld_len);
static void handle_cooked_packet(unsigned char *args, const struct pcap_pkthdr * thdr, const unsigned char * packet);
static char *set_filter_code(const char *filter);
static void packet_init();
static void packet_loop(void* ptr); */
static void start_pcap_thread();
static void stop_pcap_thread();














/* options.c */
void options_read(int argc, char **argv);

struct pfloghdr {
      unsigned char		length;
      unsigned char		af;
      unsigned char		action;
      unsigned char		reason;
      char				ifname[16];
      char				ruleset[16];
      unsigned int		rulenr;
      unsigned int		subrulenr;
      unsigned char		dir;
      unsigned char		pad[3];
};

#endif /* __IFTOP_H_ */
