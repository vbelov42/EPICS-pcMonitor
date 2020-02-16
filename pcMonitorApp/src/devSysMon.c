/* devSysMon.c Miroslaw Dach SLS/PSI 16.10.2004*/
/* originally created for EPICS 3.14.1 */
/* For EPICS 3.14.7 compatibility it was introduced */
/* #include "epicsExport.h" */
/* and epicsExportAddress function for every device support */  
/* This device support reads the uptime from /proc/uptime and converts it 
   to human readable format */
/* it examines also following files to obtain CPU load, load avg and mem info
      /proc/loadavg
      /proc/stat
      /proc/meminfo
*/

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "epicsVersion.h"
#include "alarm.h"
#include "cvtTable.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "recGbl.h"
#include "recSup.h"
#include "devSup.h"
#include "link.h"
#include "stringinRecord.h"
#include "aiRecord.h"
#if EPICS_VERSION >= 3 && EPICS_REVISION >= 14
#include "epicsTime.h"
#endif
#if EPICS_VERSION >= 3 && EPICS_REVISION >= 14 && EPICS_MODIFICATION > 4
#include "epicsExport.h"
#endif

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
# include <stdint.h>
#else
# define INT32_MAX  (2147483647)
# define UINT32_MAX (4294967295U)
# if __WORDSIZE == 64
#  define INT64_MAX  9223372036854775807L
#  define UINT64_MAX 18446744073709551615UL
# else
#  define INT64_MAX  9223372036854775807LL
#  define UINT64_MAX 18446744073709551615ULL
# endif
#endif
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

/*------------------------- Common definitions and functions ------------------*/

typedef epicsUInt64  counter_t; /* 32 bits are not enough on modern systems */
typedef epicsFloat64 avalue_t; /* analog */
typedef epicsInt32   bvalue_t; /* binary */
typedef struct { avalue_t val; } GAUGE;
typedef struct { counter_t cnt; avalue_t val; } COUNTER;
#define LINE_SIZE 256
typedef long (*PROCESSFUN)(int iter);

/*static void update_GAUGE (GAUGE *data, counter_t new, double rdt) { data->val = new; } */
#define update_GAUGE(D,N,F) do { (D)->val = (N); } while(0)
static void update_COUNTER (COUNTER *data, counter_t new, double rdt) {
  if (new < data->cnt) {
    /* counter stepped down, this is unusual */
    counter_t d = data->cnt - new;
    if (d > INT64_MAX)
      /* 64-bit counter overflowed */
      d = UINT64_MAX - data->cnt;
    else if (data->cnt <= UINT32_MAX && d > INT32_MAX)
      /* 32-bit counter overflowed */
      d = UINT32_MAX - data->cnt;
    else if (new < (data->cnt >> 1))
      /* counter was reset */
      d = 0;
    else {
      /* I don't know */
      data->val = 0.; data->cnt = new; return;
    }
    data->val = (d + new)*rdt;
  } else
    data->val = (new - data->cnt)*rdt;
  data->cnt = new; }

static long read_ai(aiRecord *prec)
{
  if (!prec->dpvt)
    return S_dev_NoInit;
  const char *opt = prec->inp.value.instio.string;
  if (opt[0]=='P' && opt[1]=='R' && opt[2]=='O' && opt[3]=='C')
    ((PROCESSFUN)prec->dpvt)(1);
  else
    prec->val = *(double*)prec->dpvt;
  return 2; /* don't convert */
}
static long read_si(stringinRecord *prec)
{
  if (!prec->dpvt)
    return S_dev_NoInit;
  const char *opt = prec->inp.value.instio.string;
  if (opt[0]=='P' && opt[1]=='R' && opt[2]=='O' && opt[3]=='C')
    ((PROCESSFUN)prec->dpvt)(1);
  else
    sprintf(prec->val,"%.*s",(int)sizeof(prec->val)-1,(char*)prec->dpvt);
  return 0;
}

static const char* parse_options(const char* line, char *key, unsigned key_size, char *val, unsigned val_size)
{
  int m = 0;
  if (line == NULL || key == NULL || val == NULL)
    return NULL;
  const char *p = line;
  char *k = key, *v = val, *k2 = key+key_size-1, *v2 = val+val_size-1;
  for ( ; *p != '\0'; p++) {
    switch (m) {
    case 0: /* before key */
      if (isspace(*p)) {
        ;
      } else if (isalnum(*p)) {
        m = 1;
        *(k++) = *p;
      } else if (*p == '=') {
        m = 2;
      } else
        m = -1;
      break;
    case 1: /* key */
      if (isalnum(*p)) {
        if (k < k2)
          *(k++) = *p;
      } else if (*p == '=') {
        m = 2;
      } else
        m = -1;
      break;
    case 2: /* value */
      if (v==val && *p=='\'') {
        m = 3;
      } else if (isalnum(*p)) {
        if (v < v2)
          *(v++) = *p;
      } else
        m = -1;
      break;
    case 3: /* value in quotes */
      if (*p=='\'') {
        p++;
        m = -1;
      } else {
        if (v < v2)
          *(v++) = *p;
      }
    default:
      ;
    }
    if (m == -1) break; /* skips p++ */
  }
  *(k++) = '\0'; *(v++) = '\0';
  /* while (isspace(*p)) p++; */
  return p;
}


/*------------------------------------------------------------*/
/*         System information                                 */
/*------------------------------------------------------------*/

static long sys_info_init(int after);
static long sys_info_init_record_si(stringinRecord *prec);
static long sys_info_init_record_ai(aiRecord *prec);
static int  sys_info_parse_options(const char *name, const char *opt);
static long sys_info_process(int iter);
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
} devSysInfoSi = {
        5,
        NULL,
        sys_info_init,
        sys_info_init_record_si,
        NULL,
        read_si,
};
epicsExportAddress(dset,devSysInfoSi); 
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record_ai;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_ai;
        DEVSUPFUN       special_linconv;
} devSysInfoAi = {
        6,
        NULL,
        sys_info_init,
        sys_info_init_record_ai,
        NULL,
        read_ai,
        NULL
};
epicsExportAddress(dset,devSysInfoAi); 

struct system_info {
  /* config */
  char ifname[16]; /* interface to take address from, default is 'eth0' */
  char timefmt[16]; /* format to convert time to string, default is '%a %b %d %H:%M' */
  /* internal */
  epicsTimeStamp time;
  char buf[LINE_SIZE];
  FILE *fp;
  /* results */
  char system[48];
  char arch[48];
  char release[48];
  char version[48];
  char hostname[48]; /* actually can be as long as 256 */
  char ipaddr[48];
  char osname[48];
  time_t time_current;
  time_t time_boot;
  time_t time_up;
  char time_boot_f[16];
  char time_boot_s[32];
  GAUGE time_up_i;
  char time_up_s[16];
  GAUGE load_short; /*  1 min */
  GAUGE load_mid;   /*  5 min */
  GAUGE load_long;  /* 15 min */
  GAUGE proc_sched;
  GAUGE proc_running;
  GAUGE proc_blocked;
  COUNTER proc_new;
};
struct system_info *sys_info = NULL;

static long sys_info_init(int after)
{
  if (sys_info!=NULL)
    return 0;

  sys_info = (struct system_info*)malloc(sizeof(struct system_info));
  memset(sys_info,0,sizeof(struct system_info));
  const char* filename = "/proc/loadavg";
  FILE *fp = fopen(filename,"r");
  if (fp==NULL) {
    fprintf(stderr, "sys_info_init: can't open file '%s': %s\n", filename, strerror(errno));
  } else
    sys_info->fp = fp;
  strncpy(sys_info->ifname,"eth0",sizeof(sys_info->ifname)-1);
  strncpy(sys_info->timefmt,"%a %b %d %H:%M",sizeof(sys_info->timefmt)-1);
  sys_info_process(0);
  return 0;
}
static long sys_info_init_record_si(stringinRecord *prec)
{
  switch (prec->inp.type) {
  case CONSTANT:
    if(recGblInitConstantLink(&prec->inp,DBF_DOUBLE,&prec->val))
      prec->udf = FALSE;
    break;
  case INST_IO: {
    const char *opt = prec->inp.value.instio.string;
    int j, k; char name[16];
    name[0] = '\0'; j = 0; k = sscanf(opt, "%15s %n", name, &j); opt += j;

    if      (strcmp(name,"PROC")==0) {
      prec->dpvt = sys_info_process;
      sys_info_parse_options(name, opt); }
    else if (strcmp(name,"SYSNAME")==0)  prec->dpvt = &sys_info->system;
    else if (strcmp(name,"MACHINE")==0)  prec->dpvt = &sys_info->arch;
    else if (strcmp(name,"RELEASE")==0)  prec->dpvt = &sys_info->release;
    else if (strcmp(name,"VERSION")==0)  prec->dpvt = &sys_info->version;
    else if (strcmp(name,"HOSTNAME")==0) prec->dpvt = &sys_info->hostname;
    else if (strcmp(name,"IPADDR")==0)   prec->dpvt = &sys_info->ipaddr;
    else if (strcmp(name,"OSNAME")==0)   prec->dpvt = &sys_info->osname;
    else if (strcmp(name,"BOOTIME")==0)  prec->dpvt = &sys_info->time_boot_s;
    else if (strcmp(name,"UPTIME")==0)   prec->dpvt = &sys_info->time_up_s;
    else prec->dpvt = NULL;

    if (prec->dpvt!=NULL)
      prec->udf = FALSE;
  } break;
  default:
    recGblRecordError(S_db_badField, (void*)prec, "init_record: illegial INP field");
    return S_db_badField;
  }
  return 0;
}
static long sys_info_init_record_ai(aiRecord *prec)
{
  switch (prec->inp.type) {
  case CONSTANT:
    if(recGblInitConstantLink(&prec->inp,DBF_DOUBLE,&prec->val))
      prec->udf = FALSE;
    break;
  case INST_IO: {
    const char *opt = prec->inp.value.instio.string;
    int j, k; char name[16];
    name[0] = '\0'; j = 0; k = sscanf(opt, "%15s %n", name, &j); opt += j;

    if      (strcmp(name,"PROC")==0) {
      prec->dpvt = sys_info_process;
      sys_info_parse_options(name, opt); }
    else if (strcmp(name,"LA1min")==0)   prec->dpvt = &sys_info->load_short.val;
    else if (strcmp(name,"LA5min")==0)   prec->dpvt = &sys_info->load_mid.val;
    else if (strcmp(name,"LA15min")==0)  prec->dpvt = &sys_info->load_long.val;
    else if (strcmp(name,"UPTIME")==0)   prec->dpvt = &sys_info->time_up_i.val;
    else prec->dpvt = NULL;

    if (prec->dpvt!=NULL)
      prec->udf = FALSE;
  } break;
  default:
    recGblRecordError(S_db_badField, (void*)prec, "init_record: illegial INP field");
    return S_db_badField;
  }
  return 0;
}
static int sys_info_parse_options(const char *name, const char *opt)
{
  int n = 0; char key[16], val[32];
  while (opt != NULL && *opt != '\0') {
    opt = parse_options(opt, key, sizeof(key), val, sizeof(val));
    if (key[0]=='\0' && val[0]=='\0') {
      break; /* end of options */
    } else if (strcmp(key,"IFNAME")==0) {
      strncpy(sys_info->ifname, val, sizeof(sys_info->ifname)-1);
    } else if (strcmp(key,"TIMEFMT")==0) {
      strncpy(sys_info->timefmt, val, sizeof(sys_info->timefmt)-1);
    } else {
      fprintf(stderr, "SysInfo @%s : unknown option '%s'\n", name, key);
    }
    n++;
  }
  return n;
}
#include <sys/utsname.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
static long sys_info_process(int iter)
{
  epicsTimeStamp now; double dt;
  if (sys_info==NULL)
    return -1;
  epicsTimeGetCurrent(&now);
  dt = epicsTimeDiffInSeconds(&now, &sys_info->time);
  if (iter==0) {
    struct utsname un;
    if (uname(&un)==0) {
      strncpy(sys_info->system, un.sysname, sizeof(sys_info->system)-1);
      strncpy(sys_info->arch, un.machine, sizeof(sys_info->arch)-1);
      strncpy(sys_info->release, un.release, sizeof(sys_info->release)-1);
      strncpy(sys_info->version, un.version, sizeof(sys_info->version)-1);
      strncpy(sys_info->hostname, un.nodename, sizeof(sys_info->hostname)-1);
    } else {
      fprintf(stderr, "sys_info_init: call 'uname' failed: %s\n", strerror(errno));
    }
  }
  if (iter > 0 && sys_info->ipaddr[0]=='\0') {
    /* the following code is very specific, so we don't care about errors */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock != -1) {
      struct ifreq ifr;
      memset(&ifr, 0, sizeof(ifr));
      strncpy(ifr.ifr_name, sys_info->ifname, sizeof(ifr.ifr_name)-1);
      if (ioctl(sock, SIOCGIFADDR, &ifr) != -1) {
        /* For compatibility only AF_INET addresses are returned by SIOCGIFADDR */
        struct sockaddr_in *sa = (struct sockaddr_in *)&ifr.ifr_addr;
        inet_ntop(sa->sin_family, &sa->sin_addr, sys_info->ipaddr, sizeof(sys_info->ipaddr));
      }
      close(sock);
    }
  }
  if (iter==0) {
    /* let's rely on LSB, but it isn't important */
    FILE *fp = popen("lsb_release -s -d","r");
    if (fp != NULL) {
      if (fgets(sys_info->buf, sizeof(sys_info->buf), fp) != NULL) {
        char *p = sys_info->buf; char *q = p+strlen(p)-1;
        if (*q=='\n') q--; if (*q=='\r') q--; /* there will be new line */
        if (*p=='"') { p++; if (*q=='"') q--; } /* and quotes, maybe */
        *(++q) = '\0';
        strncpy(sys_info->osname, p, sizeof(sys_info->osname)-1);
      }
      pclose(fp);
    }
  }
  {
    epicsTimeToTime_t(&sys_info->time_current, &now);
    if (sys_info->time_boot != 0) {
      if (sys_info->time_boot_s[0] == '\0') {
        /* first apperance, need to make a string version */
        epicsTimeStamp ets;
        epicsTimeFromTime_t(&ets, sys_info->time_boot);
        epicsTimeToStrftime(sys_info->time_boot_s, sizeof(sys_info->time_boot_s), sys_info->timefmt, &ets);
      }
      sys_info->time_up = sys_info->time_current-sys_info->time_boot;
      sys_info->time_up_i.val = sys_info->time_up;
      {
        ldiv_t td = ldiv(sys_info->time_up, 86400);
        div_t tH = div(td.rem, 3600);
        div_t tM = div(tH.rem, 60);
        if (td.quot==0)
          snprintf(sys_info->time_up_s, sizeof(sys_info->time_up_s), "%2d:%02d", tH.quot, tM.quot);
        else if (td.quot==1)
          snprintf(sys_info->time_up_s, sizeof(sys_info->time_up_s), "1 day %2d:%02d", tH.quot, tM.quot);
        else
          snprintf(sys_info->time_up_s, sizeof(sys_info->time_up_s), "%d days %2d:%02d", (int)td.quot, tH.quot, tM.quot);
      }
    }
  }
  if (sys_info->fp != NULL) {
    int i, n;
    rewind(sys_info->fp);
    for(i=0; fgets(sys_info->buf,sizeof(sys_info->buf),sys_info->fp)!=NULL; i++) {
      double a[3]; int b[3];
      n = sscanf(sys_info->buf, "%lf %lf %lf %d/%d %d", &a[0], &a[1], &a[2], &b[0], &b[1], &b[2]);
      sys_info->load_short.val = a[0];
      sys_info->load_mid.val   = a[1];
      sys_info->load_long.val  = a[2];
      sys_info->proc_sched.val = b[0];
      sys_info->proc_running.val = b[1];
      /* proc_blocked and proc_new are filled by cpu_info_process() */
    }
  }

  memcpy(&sys_info->time, &now, sizeof(now));
  return 0;
}


/*------------------------------------------------------------*/
/*         CPU record specification                           */
/*------------------------------------------------------------*/

static long cpu_info_init(int after);
static long cpu_info_init_record(aiRecord *prec);
static int  cpu_info_parse_options(const char *name, const char *opt);
static long cpu_info_process(int iter);
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record_ai;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_ai;
        DEVSUPFUN       special_linconv;
} devCPULoad = {
        6,
        NULL,
        cpu_info_init,
        cpu_info_init_record,
        NULL,
        read_ai,
        NULL
};

epicsExportAddress(dset,devCPULoad);

#define CPU_UNITS_NORM    1
#define CPU_UNITS_PERCPU  2
#define CPU_UNITS_PERCENT 4
struct cpu_info {
  /* config */
  char units;
  double tick; /* USER_HZ, units for cpu counters */
  /* internal */
  epicsTimeStamp time;
  char buf[LINE_SIZE];
  FILE* fp;
  /* results */
  int count; /* number of cores */
  COUNTER user;
  COUNTER nice;
  COUNTER system;
  COUNTER idle;
  COUNTER iowait;
  COUNTER irq;
  COUNTER softirq;
  COUNTER steal;
  COUNTER guest;
  COUNTER guest_nice;
};
struct cpu_info *cpu_info = NULL;

static long cpu_info_init(int after)
{
  if (cpu_info!=NULL)
    return 0;

  const char* filename = "/proc/stat";
  FILE *fp = fopen(filename,"r");
  if (fp==NULL) {
    fprintf(stderr, "cpu_info_init: can't open file '%s': %s\n", filename, strerror(errno));
    return S_dev_noDeviceFound;
  }
  cpu_info = (struct cpu_info*)malloc(sizeof(struct cpu_info));
  memset(cpu_info,0,sizeof(struct cpu_info));
  cpu_info->fp = fp;
  cpu_info->units = (CPU_UNITS_NORM | CPU_UNITS_PERCPU | CPU_UNITS_PERCENT);
  cpu_info_process(0);
  return 0;
}
static long cpu_info_init_record(aiRecord *prec)
{
  switch (prec->inp.type) {
  case CONSTANT:
    if(recGblInitConstantLink(&prec->inp,DBF_DOUBLE,&prec->val))
      prec->udf = FALSE;
    break;
  case INST_IO: {
    const char *opt = prec->inp.value.instio.string;
    int j, k; char name[16];
    name[0] = '\0'; j = 0; k = sscanf(opt, "%15s %n", name, &j); opt += j;

    if      (strcmp(name,"PROC")==0) {
      prec->dpvt = cpu_info_process;
      cpu_info_parse_options(name, opt); }
    else if (strcmp(name,"USER")==0)   prec->dpvt = &cpu_info->user.val;
    else if (strcmp(name,"NICE")==0)   prec->dpvt = &cpu_info->nice.val;
    else if (strcmp(name,"SYSTEM")==0) prec->dpvt = &cpu_info->system.val;
    else if (strcmp(name,"IDLE")==0)   prec->dpvt = &cpu_info->idle.val;
    else prec->dpvt = NULL;

    if (prec->dpvt != NULL) {
      prec->udf = FALSE;
      strncpy(prec->egu, ((cpu_info->units&CPU_UNITS_PERCENT)?"%":""), sizeof(prec->egu)-1);
    }
  } break;
  default:
    recGblRecordError(S_db_badField, (void*)prec, "init_record: illegial INP field");
    return S_db_badField;
  }
  return 0;
}
static int cpu_info_parse_options(const char *name, const char *opt)
{
  int n = 0; char key[16], val[32];
  while (opt != NULL && *opt != '\0') {
    opt = parse_options(opt, key, sizeof(key), val, sizeof(val));
    if (key[0]=='\0' && val[0]=='\0') {
      break; /* end of options */
    } else if (strcmp(key,"NORM")==0) {
      if (val[0]=='\0' || (val[0]=='1' && val[1]=='\0'))
        cpu_info->units |= CPU_UNITS_NORM;
      else if (val[0]=='0' && val[1]=='\0')
        cpu_info->units &= ~CPU_UNITS_NORM;
      else
        fprintf(stderr, "CpuLoad @%s : bad value '%s'\n", name, val);
    } else if (strcmp(key,"PERCPU")==0) {
      if (val[0]=='\0' || (val[0]=='1' && val[1]=='\0'))
        cpu_info->units |= CPU_UNITS_PERCPU;
      else if (val[0]=='0' && val[1]=='\0')
        cpu_info->units &= ~CPU_UNITS_PERCPU;
      else
        fprintf(stderr, "CpuLoad @%s : bad value '%s'\n", name, val);
    } else if (strcmp(key,"PERCENT")==0) {
      if (val[0]=='\0' || (val[0]=='1' && val[1]=='\0'))
        cpu_info->units |= CPU_UNITS_PERCENT;
      else if (val[0]=='0' && val[1]=='\0')
        cpu_info->units &= ~CPU_UNITS_PERCENT;
      else
        fprintf(stderr, "CpuLoad @%s : bad value '%s'\n", name, val);
    } else {
      fprintf(stderr, "CpuLoad @%s : unknown option '%s'\n", name, key);
    }
    n++;
  }
  return n;
}
static long cpu_info_process(int iter)
{
  int i, j;
  epicsTimeStamp now; double dt;
  int ncpu = 0, ncnt = 0; unsigned long cnt[10];
  if (cpu_info==NULL)
    return -1;
  rewind(cpu_info->fp);
  epicsTimeGetCurrent(&now);
  dt = epicsTimeDiffInSeconds(&now, &cpu_info->time);
  for(i=0; fgets(cpu_info->buf,sizeof(cpu_info->buf),cpu_info->fp)!=NULL; i++) {
    /* only 'intr' can be up to 1000 bytes, all the rest is < 100. */
    if (strncmp(cpu_info->buf,"cpu ",4)==0) {
      ncnt = sscanf(cpu_info->buf+4,"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                    &cnt[0],&cnt[1],&cnt[2],&cnt[3],&cnt[4],&cnt[5],&cnt[6],&cnt[7],&cnt[8],&cnt[9]);
      for (j = ncnt; j < sizeof(cnt)/sizeof(cnt[0]); j++) cnt[j] = 0; /* set unread counters */
    } else if (strncmp(cpu_info->buf,"cpu",3)==0 && isdigit(cpu_info->buf[3])) { /* cpu\d+ */
      ncpu++;
    } else if (strncmp(cpu_info->buf,"btime ",6)==0) {
      if (sys_info!=NULL && sys_info->time_boot==0)
        sys_info->time_boot = strtoul(cpu_info->buf+6, NULL, 10);
    } else if (strncmp(cpu_info->buf,"processes ",10)==0) {
      if (sys_info!=NULL) {
        update_COUNTER(&sys_info->proc_new, strtoul(cpu_info->buf+10,NULL,10), 1.);
      }
    } else if (strncmp(cpu_info->buf,"procs_running ",14)==0) {
      if (sys_info!=NULL)
        update_GAUGE(&sys_info->proc_running, strtoul(cpu_info->buf+10,NULL,10), 1.);
    } else if (strncmp(cpu_info->buf,"procs_blocked ",14)==0) {
      if (sys_info!=NULL)
        update_GAUGE(&sys_info->proc_blocked, strtoul(cpu_info->buf+10,NULL,10), 1.);
    }
  }
  if (iter==0) {
    cpu_info->tick = sysconf(_SC_CLK_TCK);
    cpu_info->count = ncpu;
    cpu_info->user.cnt = cnt[0];
    cpu_info->nice.cnt = cnt[1];
    cpu_info->system.cnt = cnt[2];
    cpu_info->idle.cnt = cnt[3];
    cpu_info->iowait.cnt = cnt[4];
    cpu_info->irq.cnt = cnt[5];
    cpu_info->softirq.cnt = cnt[6];
    cpu_info->steal.cnt = cnt[7];
    cpu_info->guest.cnt = cnt[8];
    cpu_info->guest_nice.cnt = cnt[9];
  } else {
    double f = 1., s = 0.;
    if (!(cpu_info->units&CPU_UNITS_NORM)) {
      f = 1./cpu_info->tick/dt;
      if ( (cpu_info->units&CPU_UNITS_PERCPU))  f /= ncpu;
      if ( (cpu_info->units&CPU_UNITS_PERCENT)) f *= 100;
    }
    update_COUNTER(&cpu_info->guest_nice, cnt[9], f); s += cpu_info->guest_nice.val;
    update_COUNTER(&cpu_info->guest, cnt[8], f); s += cpu_info->guest.val;
    update_COUNTER(&cpu_info->steal, cnt[7], f); s += cpu_info->steal.val;
    update_COUNTER(&cpu_info->softirq, cnt[6], f); s += cpu_info->softirq.val;
    update_COUNTER(&cpu_info->irq, cnt[5], f); s += cpu_info->irq.val;
    update_COUNTER(&cpu_info->iowait, cnt[4], f); s += cpu_info->iowait.val;
    update_COUNTER(&cpu_info->user, cnt[0], f); s += cpu_info->user.val;
    update_COUNTER(&cpu_info->nice, cnt[1], f); s += cpu_info->nice.val;
    update_COUNTER(&cpu_info->system, cnt[2], f); s += cpu_info->system.val;
    update_COUNTER(&cpu_info->idle, cnt[3], f); s += cpu_info->idle.val;
    if ( (cpu_info->units&CPU_UNITS_NORM) && s>0.) {
      f = 1./s;
      if (!(cpu_info->units&CPU_UNITS_PERCPU))  f *= ncpu;
      if ( (cpu_info->units&CPU_UNITS_PERCENT)) f *= 100;
      cpu_info->guest_nice.val *= f;
      cpu_info->guest.val *= f;
      cpu_info->steal.val *= f;
      cpu_info->softirq.val *= f;
      cpu_info->irq.val *= f;
      cpu_info->iowait.val *= f;
      cpu_info->user.val *= f;
      cpu_info->nice.val *= f;
      cpu_info->system.val *= f;
      cpu_info->idle.val *= f;
    }
  }
  memcpy(&cpu_info->time, &now, sizeof(now));
  return 0;
}


/*------------------------------------------------------------*/
/*         MEM record specification                           */
/*------------------------------------------------------------*/

static long mem_info_init(int after);
static long mem_info_init_record(aiRecord *prec);
static int  mem_info_parse_options(const char *name, const char *opt);
static long mem_info_process(int iter);
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record_ai;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_ai;
        DEVSUPFUN       special_linconv;
} devMEMLoad = {
        6,
        NULL,
        mem_info_init,
        mem_info_init_record,
        NULL,
        read_ai,
        NULL
};

epicsExportAddress(dset,devMEMLoad);

struct mem_info {
  /* config */
  int page_size; /* system memory page size */
  /* internal */
  epicsTimeStamp time;
  char buf[LINE_SIZE];
  FILE* fp;
  /* results */
  GAUGE mem_total;
  GAUGE mem_free;
  GAUGE mem_used;
  GAUGE mem_shared;
  GAUGE mem_buffers;
  GAUGE mem_cached;
  GAUGE swap_total;
  GAUGE swap_free;
  GAUGE swap_used;
  GAUGE swap_cached;
};
struct mem_info *mem_info = NULL;

static long mem_info_init(int after)
{
  if (mem_info!=NULL)
    return 0;

  const char* filename = "/proc/meminfo";
  FILE *fp = fopen(filename,"r");
  if (fp==NULL) {
    fprintf(stderr, "mem_info_init: can't open file '%s': %s\n", filename, strerror(errno));
    return S_dev_noDeviceFound;
  }
  mem_info = (struct mem_info*)malloc(sizeof(struct mem_info));
  memset(mem_info, 0, sizeof(struct mem_info));
  mem_info->fp = fp;
  mem_info->page_size = sysconf(_SC_PAGESIZE);
  return 0;
}
static long mem_info_init_record(aiRecord *prec)
{
  switch (prec->inp.type) {
  case CONSTANT:
    if(recGblInitConstantLink(&prec->inp,DBF_DOUBLE,&prec->val))
      prec->udf = FALSE;
    break;
  case INST_IO: {
    const char *opt = prec->inp.value.instio.string;
    int j, k; char name[16];
    name[0] = '\0'; j = 0; k = sscanf(opt, "%15s %n", name, &j); opt += j;

    if      (strcmp(name,"PROC")==0) {
      prec->dpvt = mem_info_process;
      mem_info_parse_options(name, opt); }
    else if (strcmp(name,"MEMAV")==0)    prec->dpvt = &mem_info->mem_total.val;
    else if (strcmp(name,"MEMFREE")==0)  prec->dpvt = &mem_info->mem_free.val;
    else if (strcmp(name,"MEMUSED")==0)  prec->dpvt = &mem_info->mem_used.val;
    else if (strcmp(name,"MEMSHRD")==0)  prec->dpvt = &mem_info->mem_shared.val;
    else if (strcmp(name,"MEMBUFF")==0)  prec->dpvt = &mem_info->mem_buffers.val;
    else if (strcmp(name,"MEMCACH")==0)  prec->dpvt = &mem_info->mem_cached.val;
    else if (strcmp(name,"SWAPAV")==0)   prec->dpvt = &mem_info->swap_total.val;
    else if (strcmp(name,"SWAPUSED")==0) prec->dpvt = &mem_info->swap_used.val;
    else if (strcmp(name,"SWAPFREE")==0) prec->dpvt = &mem_info->swap_free.val;
    else if (strcmp(name,"SWAPCACH")==0) prec->dpvt = &mem_info->swap_cached.val;
    else prec->dpvt = NULL;

    if (prec->dpvt!=NULL)
      prec->udf = FALSE;
  } break;
  default:
    recGblRecordError(S_db_badField, (void*)prec, "init_record: illegial INP field");
    return S_db_badField;
  }
  return 0;
}
static int mem_info_parse_options(const char *name, const char *opt)
{
  int n = 0; char key[16], val[32];
  while (opt != NULL && *opt != '\0') {
    opt = parse_options(opt, key, sizeof(key), val, sizeof(val));
    if (key[0]=='\0' && val[0]=='\0') {
      break; /* end of options */
    } else if (strcmp(key,"UNITS")==0) {
      ;
    } else {
      fprintf(stderr, "MemLoad @%s : unknown option '%s'\n", name, key);
    }
    n++;
  }
  return n;
}
static long mem_info_process(int iter)
{
  int i;
  epicsTimeStamp now; double dt;
  char field[16]; unsigned long val; counter_t cnt;
  if (mem_info==NULL)
    return -1;
  rewind(mem_info->fp);
  epicsTimeGetCurrent(&now);
  dt = epicsTimeDiffInSeconds(&now, &mem_info->time);
  field[15] = '\0';
  for(i=0; fgets(mem_info->buf,sizeof(mem_info->buf),mem_info->fp)!=NULL; i++) {
    int n = sscanf(mem_info->buf, "%15s %lu", field, &val);
    if (n < 2) continue; /* no value read */
    cnt = val; /* cnt <<= 10; read values are in kB */
    if      (strcmp(field,"MemTotal:")==0)   update_GAUGE(&mem_info->mem_total, cnt, 1.);
    else if (strcmp(field,"MemFree:")==0)    update_GAUGE(&mem_info->mem_free, cnt, 1.);
    else if (strcmp(field,"MemShared:")==0)  update_GAUGE(&mem_info->mem_shared, cnt, 1.); /* */
    else if (strcmp(field,"Shmem:")==0)      update_GAUGE(&mem_info->mem_shared, cnt, 1.); /* since 2.6.32 */
    else if (strcmp(field,"Buffers:")==0)    update_GAUGE(&mem_info->mem_buffers, cnt, 1.);
    else if (strcmp(field,"Cached:")==0)     update_GAUGE(&mem_info->mem_cached, cnt, 1.);
    else if (strcmp(field,"SwapTotal:")==0)  update_GAUGE(&mem_info->swap_total, cnt, 1.);
    else if (strcmp(field,"SwapFree:")==0)   update_GAUGE(&mem_info->swap_free, cnt, 1.);
    else if (strcmp(field,"SwapCached:")==0) update_GAUGE(&mem_info->swap_cached, cnt, 1.);
  }
  update_GAUGE(&mem_info->mem_used, mem_info->mem_total.val - mem_info->mem_free.val, 1.);
  update_GAUGE(&mem_info->swap_used, mem_info->swap_total.val - mem_info->swap_free.val, 1.);
  memcpy(&mem_info->time, &now, sizeof(now));
  return 0;
}

