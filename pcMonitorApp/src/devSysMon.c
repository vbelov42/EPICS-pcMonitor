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

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
//#include "/usr/include/linux/version.h"
#include <errno.h>

#include <sys/time.h>
#include <time.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netdb.h>

#include <sys/utsname.h>
/* in vesrion 3.14.7 #include "epicsExport.h" */
#include "epicsExport.h"

/*------------------------- Common definitions and functions ------------------*/

typedef epicsUInt64  counter_t; /* 32 bits are not enough on modern systems */
typedef epicsFloat64 avalue_t; /* analog */
typedef epicsInt32   bvalue_t; /* binary */
typedef struct { avalue_t val; } GAUGE;
typedef struct { counter_t cnt; avalue_t val; } COUNTER;
#define LINE_SIZE 256
typedef long (*PROCESSFUN)(int iter);

static long read_ai(aiRecord *prec)
{
  if (!prec->dpvt)
    return S_dev_NoInit;
  const char *opt = prec->inp.value.vmeio.parm;
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
  if (strcmp(prec->inp.value.vmeio.parm,"PROC")==0)
    ((PROCESSFUN)prec->dpvt)(1);
  else
    strncpy(prec->val,(char*)prec->dpvt,sizeof(prec->val));
  return 0;
}

static int get_uptime(void);           /* get formated time from /proc/uptime */
static void format_uptime(char *str1); /* set formated up time in str1 */
static double getAvgLoad(char * parm); /* get the avg load for parm= 1min ora 5min ora 15min */

/*------------------------- Create the dset for devUptime ------------------*/
static long init_record_s();
static long read_uptime();
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
}devUpTime={
        5,
        NULL,
        NULL,
        init_record_s,
        NULL,
        read_uptime,
};
epicsExportAddress(dset,devUpTime); 


static long init_record_s(pStringIn)
    struct stringinRecord    *pStringIn;
{
    if(recGblInitConstantLink(&pStringIn->inp,DBF_STRING,&pStringIn->val))
         pStringIn->udf = FALSE;
    return(0);
}


static long read_uptime(pStringIn)
    struct stringinRecord    *pStringIn;
{
  /*     long status; */
    struct timeval        iheure;
    int    iCurTime=0;
    int    iUpTime=0;

    struct vmeio *pvmeio = &pStringIn->inp.value.vmeio;

    /* status = dbGetLink(&(pStringIn->inp),DBF_STRING, &(pStringIn->val),0,0); */
    /*If return was succesful then set undefined false*/
    /* sprintf(pStringIn->val,"ala %d",get_uptime()); */
    if(!strcmp(pvmeio->parm,"CURTIME")){
      
      gettimeofday(&iheure,NULL);
      sprintf(pStringIn->val,"%.16s",&(ctime((time_t *)&iheure.tv_sec))[0]);

    }
    else if(!strcmp(pvmeio->parm,"BOOTIME")){
	 gettimeofday(&iheure,NULL);
	 iCurTime=iheure.tv_sec;
	 iUpTime=get_uptime();
	 iCurTime=iCurTime-iUpTime;
	 sprintf(pStringIn->val,"%.16s",&(ctime((time_t *)&iCurTime))[0]);
       }
    else if(!strcmp(pvmeio->parm,"UPTIME")){
      format_uptime(pStringIn->val);
    }
    /* if(!status) */  pStringIn->udf = FALSE;
    return(0);
}

/*-----------------  get uptime in seconds -----------------------*/

int get_uptime(void){

  char str_seconds[80];  
  FILE *proc_file;
  int int_seconds;



  if((proc_file=fopen("/proc/uptime","r"))==NULL)
    { fprintf(stderr,"Cannot open /proc/uptime for reading!\n");
    return 0; /*before  exit(127); */
    } 
  
  fscanf(proc_file,"%s",str_seconds);   
  int_seconds=strtol(str_seconds,NULL,10);

  fclose(proc_file);


return int_seconds;

}/* get_uptime */

void format_uptime(char *str1){
  int curr,sec,min,hour,day;

  curr=get_uptime();

  day=curr/86400;
  curr=curr%86400;
  hour=curr/3600;
  curr=curr%3600;
  min=curr/60;
  sec=curr%60;
  if(day==0)
  	sprintf(str1,"%02d:%02d",hour,min);
  else if (day==1)
	  sprintf(str1,"%02d day %02d:%02d",day,hour,min);
  else
	  sprintf(str1,"%02d days %02d:%02d",day,hour,min);

}

/*---------------------------  get the AVG load ------------------------------------*/

static long init_record_ai();
static long read_avg_load();
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record_ai;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_ai;
        DEVSUPFUN       special_linconv;
}devAvgLoad={
        6,
        NULL,
        NULL,
        init_record_ai,
        NULL,
        read_avg_load,
	NULL
};

epicsExportAddress(dset,devAvgLoad);

static long init_record_ai(pAiIn)
    struct aiRecord    *pAiIn;
{
    if(recGblInitConstantLink(&pAiIn->inp,DBF_DOUBLE,&pAiIn->val))
         pAiIn->udf = FALSE;
    return(0);
}


static long read_avg_load(pAiIn)
    struct aiRecord    *pAiIn;
{

    long status=0;

    struct vmeio *pvmeio = &pAiIn->inp.value.vmeio;
    /* status = dbGetLink(&(pAiIn->inp),DBF_DOUBLE, &(pAiIn->val),0,0); */
    /*If return was succesful then set undefined false*/  
    
    pAiIn->val=getAvgLoad(pvmeio->parm);
   
    if(!status) pAiIn->udf = FALSE;
    return(2); /* no convertion fron rval to val */
}


double getAvgLoad(char * parm){
 
  /* FILE *proc_file;
  double avg1min=0;
  double avg5min=0;
  double avg15min=0;
  char str1[30]="";
  char str2[30]=""; */
  double val=0;

  double avg[3];

  if(getloadavg(avg,sizeof(avg))<0)
    { fprintf(stderr,"load avearage was unobtainable.\n");
      return val; /* before exit(127); */
    } 

  /* if((proc_file=fopen("/proc/loadavg","r"))==NULL)
    { fprintf(stderr,"Cannot open /proc/loadavg for reading!\n");
    return val; 
    } 
  
    fscanf(proc_file,"%lf %lf %lf %s %s",&avg1min,&avg5min,&avg15min,str1,str2); */
  

  if(!strcmp(parm,"1min")){
    val=avg[0];
    /* val=avg1min; */
  }
  else if(!strcmp(parm,"5min")){
     val=avg[1];
     /* val=avg5min;*/
  }	
  else{
     val=avg[2];
     /* val=avg15min;*/
    } 

  

  /* fclose(proc_file);*/
	  return val;
}

/*------------------------------------------------------------*/
/*         CPU record specification                           */
/*------------------------------------------------------------*/

static long cpu_info_init(int after);
static long cpu_info_init_record(aiRecord *prec);
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

struct cpu_info {
  char buf[LINE_SIZE];
  FILE* fp;
  int count; /* number of CPUs */
  double tick; /* USER_HZ, units for cpu counters */
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
  case VME_IO: {
    const char *opt = prec->inp.value.vmeio.parm;
    if      (strcmp(opt,"PROC")==0)   prec->dpvt = cpu_info_process;
    else if (strcmp(opt,"USER")==0)   prec->dpvt = &cpu_info->user.val;
    else if (strcmp(opt,"NICE")==0)   prec->dpvt = &cpu_info->nice.val;
    else if (strcmp(opt,"SYSTEM")==0) prec->dpvt = &cpu_info->system.val;
    else if (strcmp(opt,"IDLE")==0)   prec->dpvt = &cpu_info->idle.val;
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
static long cpu_info_process(int iter)
{
  int i; int ncpu = 0, ncnt = 0;
  unsigned long cnt[10];
  if (cpu_info==NULL)
    return -1;
  rewind(cpu_info->fp);
  for(i=0; fgets(cpu_info->buf,sizeof(cpu_info->buf),cpu_info->fp)!=NULL; i++) {
    /* only 'intr' can be up to 1000 bytes, all the rest is < 100. */
    if (strncmp(cpu_info->buf,"cpu ",4)==0) {
      ncnt = sscanf(cpu_info->buf+4,"%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                    &cnt[0],&cnt[1],&cnt[2],&cnt[3],&cnt[4],&cnt[5],&cnt[6],&cnt[7],&cnt[8],&cnt[9]);
    } else if (strncmp(cpu_info->buf,"cpu ",3)==0) { /* cpu\d+ */
      ncpu++;
    }
  }
  if (iter==0) {
    cpu_info->tick = 1./sysconf(_SC_CLK_TCK);
    cpu_info->count = ncpu;
    cpu_info->user.cnt = cnt[0];
    cpu_info->nice.cnt = cnt[1];
    cpu_info->system.cnt = cnt[2];
    cpu_info->idle.cnt = cnt[3];
  } else {
    double f;
    counter_t d, s = 0;
#define CPU_DIV(FIELD,VAL,SUM) \
    d = (VAL-cpu_info->FIELD.cnt); cpu_info->FIELD.cnt = VAL; SUM += d; cpu_info->FIELD.val = d;
    switch (ncnt) {
      /* we asked scanf only for 10 numbers, so it can't be more */
    case 10: CPU_DIV(guest_nice,cnt[9],s);
    case  9: CPU_DIV(guest,cnt[8],s);
    case  8: CPU_DIV(steal,cnt[7],s);
    case  7: CPU_DIV(softirq,cnt[6],s);
    case  6: CPU_DIV(irq,cnt[5],s);
    case  5: CPU_DIV(iowait,cnt[4],s);
    case  4:
      CPU_DIV(user,cnt[0],s);
      CPU_DIV(nice,cnt[1],s);
      CPU_DIV(system,cnt[2],s);
      CPU_DIV(idle,cnt[3],s);
      break;
    default:
      /* something's going wrong */
      ;
    }
    if (s>0) {
      f = 100./s;
      switch (ncnt) {
        /* we asked scanf only for 10 numbers, so it can't be more */
      case 10: cpu_info->guest_nice.val *= f;
      case  9: cpu_info->guest.val *= f;
      case  8: cpu_info->steal.val *= f;
      case  7: cpu_info->softirq.val *= f;
      case  6: cpu_info->irq.val *= f;
      case  5: cpu_info->iowait.val *= f;
      case  4:
        cpu_info->user.val *= f;
        cpu_info->nice.val *= f;
        cpu_info->system.val *= f;
        cpu_info->idle.val *= f;
        break;
      default:
        /* something's going wrong */
        ;
      }
    }
    /* printf("cpu = %lu %f %f %f %f\n", s, cpu_info->user.val, cpu_info->nice.val, cpu_info->system.val, cpu_info->idle.val); */
  }
  return 0;
}


/*------------------------------------------------------------*/
/*         MEM record specification                           */
/*------------------------------------------------------------*/

static long mem_info_init(int after);
static long mem_info_init_record(aiRecord *prec);
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
  char buf[LINE_SIZE];
  FILE* fp;
  int page_size; /*  */
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
  case VME_IO: {
    const char *opt = prec->inp.value.vmeio.parm;
    if      (strcmp(opt,"PROC")==0)     prec->dpvt = mem_info_process;
    else if (strcmp(opt,"MEMAV")==0)    prec->dpvt = &mem_info->mem_total.val;
    else if (strcmp(opt,"MEMFREE")==0)  prec->dpvt = &mem_info->mem_free.val;
    else if (strcmp(opt,"MEMUSED")==0)  prec->dpvt = &mem_info->mem_used.val;
    else if (strcmp(opt,"MEMSHRD")==0)  prec->dpvt = &mem_info->mem_shared.val;
    else if (strcmp(opt,"MEMBUFF")==0)  prec->dpvt = &mem_info->mem_buffers.val;
    else if (strcmp(opt,"MEMCACH")==0)  prec->dpvt = &mem_info->mem_cached.val;
    else if (strcmp(opt,"SWAPAV")==0)   prec->dpvt = &mem_info->swap_total.val;
    else if (strcmp(opt,"SWAPUSED")==0) prec->dpvt = &mem_info->swap_used.val;
    else if (strcmp(opt,"SWAPFREE")==0) prec->dpvt = &mem_info->swap_free.val;
    else if (strcmp(opt,"SWAPCACH")==0) prec->dpvt = &mem_info->swap_cached.val;
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
static long mem_info_process(int iter)
{
  int i; char field[16]; unsigned long val; counter_t cnt;
  if (mem_info==NULL)
    return -1;
  rewind(mem_info->fp);
  field[15] = '\0';
  for(i=0; fgets(mem_info->buf,sizeof(mem_info->buf),mem_info->fp)!=NULL; i++) {
    int j = sscanf(mem_info->buf, "%15s %lu", field, &val);
    cnt = val; /* cnt <<= 10; read values are in kB */
    if      (strcmp(field,"MemTotal:")==0)   mem_info->mem_total.val   = cnt;
    else if (strcmp(field,"MemFree:")==0)    mem_info->mem_free.val    = cnt;
    else if (strcmp(field,"MemShared:")==0)  mem_info->mem_shared.val  = cnt; /* */
    else if (strcmp(field,"Shmem:")==0)      mem_info->mem_shared.val  = cnt; /* since 2.6.32 */
    else if (strcmp(field,"Buffers:")==0)    mem_info->mem_buffers.val = cnt;
    else if (strcmp(field,"Cached:")==0)     mem_info->mem_cached.val  = cnt;
    else if (strcmp(field,"SwapTotal:")==0)  mem_info->swap_total.val  = cnt;
    else if (strcmp(field,"SwapFree:")==0)   mem_info->swap_free.val   = cnt;
    else if (strcmp(field,"SwapCached:")==0) mem_info->swap_cached.val = cnt;
  }
  mem_info->mem_used.val = mem_info->mem_total.val - mem_info->mem_free.val;
  mem_info->swap_used.val = mem_info->swap_total.val - mem_info->swap_free.val;
  return 0;
}


/*------------------------------------------------------------*/
/*         System information                                 */
/*------------------------------------------------------------*/
static long init_record_IP();
static long read_uptime_IP();
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
}devIpAddr={
        5,
        NULL,
        NULL,
        init_record_IP,
        NULL,
        read_uptime_IP,
};

epicsExportAddress(dset,devIpAddr);

static long init_record_IP(pStringIn)
    struct stringinRecord    *pStringIn;
{
    if(recGblInitConstantLink(&pStringIn->inp,DBF_STRING,&pStringIn->val))
         pStringIn->udf = FALSE;
    return(0);
}


static long read_uptime_IP(pStringIn)
    struct stringinRecord    *pStringIn;
{
  int s;
  struct ifreq ifr;
  unsigned long iPAddr;

  if ((s=socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    return 0;

   strcpy(ifr.ifr_name, "eth0");

  if (! ioctl(s, SIOCGIFADDR, &ifr))
    {
     iPAddr=((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
     sprintf(pStringIn->val,"%lu:%lu:%lu:%lu",iPAddr%256,(iPAddr/256)%256,(iPAddr/65536)%256,(iPAddr/16777216)%256);
    }
    close(s);
    /* if(!status)*/  pStringIn->udf = FALSE;
    return(0);
}
/*-------------- get host info ---------------------------*/

static long init_record_Info();
static long read_uptime_Info();
struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read;
}devUname={
        5,
        NULL,
        NULL,
        init_record_Info,
        NULL,
        read_uptime_Info,
};

epicsExportAddress(dset,devUname);

static long init_record_Info(pStringIn)
    struct stringinRecord    *pStringIn;
{
    if(recGblInitConstantLink(&pStringIn->inp,DBF_STRING,&pStringIn->val))
         pStringIn->udf = FALSE;
    return(0);
}


static long read_uptime_Info(pStringIn)
    struct stringinRecord    *pStringIn;
{
  /* long status; */
    struct utsname name;

    struct vmeio *pvmeio = &pStringIn->inp.value.vmeio;
    /* status = dbGetLink(&(pStringIn->inp),DBF_STRING, &(pStringIn->val),0,0); */
    /*If return was succesful then set undefined false*/
    /* sprintf(pStringIn->val,"ala %d",get_uptime()); */

      if (uname (&name) == -1){
	fprintf(stderr,"UP time info: cannot get system name\n");
      }
      else{
	if(!strcmp(pvmeio->parm,"SYSNAME"))
	  sprintf(pStringIn->val,"%.*s",sizeof(pStringIn->val)-1,name.sysname);
      else if(!strcmp(pvmeio->parm,"RELEASE"))
	sprintf(pStringIn->val,"%.*s",sizeof(pStringIn->val)-1,name.release);
      else if(!strcmp(pvmeio->parm,"VERSION"))
	sprintf(pStringIn->val,"%.*s",sizeof(pStringIn->val)-1,name.version);
      else if(!strcmp(pvmeio->parm,"MACHINE"))
	sprintf(pStringIn->val,"%.*s",sizeof(pStringIn->val)-1,name.machine);
      }
       
      /* if(!status) */ pStringIn->udf = FALSE;
    return(0);
}

