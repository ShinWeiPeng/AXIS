#line 1 "..\\For_SSC_Tool\\Src\\ecatcoe.c"



 




 




















 





 

#line 1 "..\\For_SSC_Tool\\Src\\ecat_def.h"



 






 








 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 22 "..\\For_SSC_Tool\\Src\\ecat_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 23 "..\\For_SSC_Tool\\Src\\ecat_def.h"





 








 







 






 






 





 





 






 






 





 





 





 






 







 






 





 





 





 





 





 







 







 





 






 






 





 





 





 






 





 






 






 





 





 






 






 








 





 






 





 







 





 







 







 





 





 





 





 






 





 






 






 






 





 





 





 





 









 


 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 





 






 






 






 






 






 






 






 





 





 






 






 






 





 





 





 






 






 





 





 






 






 





 





 





 





 





 





 





 





 





 





 





 





 





 





 









 


 





 







 





 





 





 





 





 





 





 





 





 





 






 






 






 






 





 





 





 





 





 





 





 





 






 





 





 






 






 





 










 




#line 39 "..\\For_SSC_Tool\\Src\\ecatcoe.c"


#line 1 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 




 



















 






 
#line 1 "..\\For_SSC_Tool\\Src\\ecatcoe.h"



 




 












 





 
#line 1 "..\\For_SSC_Tool\\Src\\mailbox.h"



 




 
















 





 
#line 34 "..\\For_SSC_Tool\\Src\\mailbox.h"









 
#line 50 "..\\For_SSC_Tool\\Src\\mailbox.h"

#line 59 "..\\For_SSC_Tool\\Src\\mailbox.h"
















 





 
#line 91 "..\\For_SSC_Tool\\Src\\mailbox.h"




 
typedef struct 
{
    unsigned short                          Length;  
    unsigned short                          Address;  

    unsigned short                          Flags[1];  
#line 108 "..\\For_SSC_Tool\\Src\\mailbox.h"
}
TMBXHEADER;









 
typedef struct 
{
    TMBXHEADER                      MbxHeader;  
    unsigned short                          Data[((0x0080 - 6) >> 1)];  
}
TMBX;




 
typedef struct
{
    unsigned short    firstInQueue;  
    unsigned short    lastInQueue;  
    unsigned short    maxQueueSize;  
    TMBX  * queue[(10)+1];  
} TMBXQUEUE;










 






extern unsigned char                    bReceiveMbxIsLocked;  
extern unsigned char                    bSendMbxIsFull;  
extern unsigned char                    bMbxRunning;  
extern unsigned char                    bMbxRepeatToggle;  
extern unsigned short                  u16SendMbxSize;  
extern unsigned short                  u16ReceiveMbxSize;  
extern unsigned short                  u16EscAddrReceiveMbx;  
extern unsigned short                  u16EscAddrSendMbx;  
extern unsigned char                   u8MbxWriteCounter;  
extern unsigned char                   u8MbxReadCounter;  
extern unsigned char                   u8MailboxSendReqStored;  
extern TMBX  *           psWriteMbx;  
extern TMBX  *           psReadMbx;  
extern TMBX  *           psRepeatMbx;  
extern TMBX  *           psStoreMbx;  
extern TMBXQUEUE         sMbxSendQueue;  
extern TMBXQUEUE         sMbxReceiveQueue;  






 

extern   void     MBX_Init(void);
extern   unsigned short    MBX_StartMailboxHandler(void);
extern   void     MBX_StopMailboxHandler(void);
extern   void     MBX_MailboxWriteInd(TMBX  *pMbx);
extern   void     MBX_MailboxReadInd(void);
extern   void     MBX_MailboxRepeatReq(void);
extern   unsigned char    MBX_MailboxSendReq(TMBX  * pMbx, unsigned char flags);
extern   void     MBX_CheckAndCopyMailbox(void);
extern   unsigned char    MBX_CopyToSendMailbox(TMBX  *pMbx);
extern   void     MBX_Main(void);


 
#line 30 "..\\For_SSC_Tool\\Src\\ecatcoe.h"








 



 






 
#line 58 "..\\For_SSC_Tool\\Src\\ecatcoe.h"





 





typedef unsigned short TCOEHEADER;  




 
typedef struct 
{
  TMBXHEADER        MbxHeader;  
  TCOEHEADER        CoeHeader;  
  unsigned short            Data[(((0x0080 - 6))-(2)) >> 1];  
}
TCOEMBX;














 
extern    TMBX  *  pCoeSendStored;                



 





 

extern   void     COE_Init(void);
extern   unsigned char    COE_ServiceInd(TCOEMBX  *pCoeMbx);
extern   unsigned char     COE_ContinueInd(TMBX  * pMbx);


 
#line 38 "..\\For_SSC_Tool\\Src\\sdoserv.h"









 




 



 





 
#line 73 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 
#line 89 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 


 
typedef struct 
{
    unsigned short Sdo[2];  
 
#line 114 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 
}
TINITSDOHEADER;







 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDOMBX;






 
 









 
typedef struct 
{
    unsigned short      SegHeader;  
#line 163 "..\\For_SSC_Tool\\Src\\sdoserv.h"
    unsigned short      Data[((((0x0080 - 6))-(( 2 + 1 )) - 1 ) >> 1)];   
}
TSDOSEGHEADERDATA;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                Data[(4) >> 1];  
}
TINITSDODOWNLOADEXPREQMBX;



 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                CompleteSize[2];  
  unsigned short                Data[((((0x0080 - 6))-(( 2 + 4 + 4 ))) >> 1)];  
}
TINITSDODOWNLOADNORMREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDODOWNLOADRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TSDOSEGHEADERDATA     SdoHeader;  
}
TDOWNLOADSDOSEGREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  unsigned char                 SegHeader;  
}
TDOWNLOADSDOSEGRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
}
TINITSDOUPLOADREQMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                Data[((4) >> 1)];  
}
TINITSDOUPLOADEXPRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned short                CompleteSize[2];  
  unsigned short                Data[((((0x0080 - 6))-(( 2 + 4 + 4 ))) >> 1)];  
}
TINITSDOUPLOADNORMRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  unsigned char                 SegHeader;  
}
TUPLOADSDOSEGREQMBX;



 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TSDOSEGHEADERDATA     SdoHeader;  
}
TUPLOADSDOSEGRESMBX;




 
typedef struct 
{
  TMBXHEADER            MbxHeader;  
  TCOEHEADER            CoeHeader;  
  TINITSDOHEADER        SdoHeader;  
  unsigned long             AbortCode;  
}
TABORTSDOTRANSFERREQMBX;



 





 
#line 343 "..\\For_SSC_Tool\\Src\\sdoserv.h"






 
#line 380 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 
 





 
#line 396 "..\\For_SSC_Tool\\Src\\sdoserv.h"
 





 


 
typedef struct 
{
    unsigned short                ListType;  
#line 416 "..\\For_SSC_Tool\\Src\\sdoserv.h"
}
TSDOINFOLIST;




 
typedef struct 
{
    unsigned short            DataType;  
    unsigned short            ObjFlags;  

     










}
TSDOINFOOBJDESC;







 
typedef struct 
{
    unsigned short            Index;  
    TSDOINFOOBJDESC   Res;  
}
TSDOINFOOBJ;







 
typedef struct 
{
    unsigned short                DataType;  
    unsigned short                BitLength;  
    unsigned short                ObjAccess;  
                                     









 

#line 497 "..\\For_SSC_Tool\\Src\\sdoserv.h"
}
TSDOINFOENTRYDESC;




 
typedef struct 
{
    unsigned short                Index;  
    unsigned short                 Info;  





    TSDOINFOENTRYDESC    Res;  
}
TSDOINFOENTRY;




 
typedef struct 
{
    unsigned long                ErrorCode;  
}
TSDOINFOERROR;







 
typedef struct 
{
    unsigned short InfoHead;  





    unsigned short                FragmentsLeft;  

    union 
    {
        TSDOINFOLIST    List; 
        TSDOINFOOBJ     Obj; 
        TSDOINFOENTRY   Entry; 
        TSDOINFOERROR   Error; 
        unsigned short          Data[1]; 
    }
    Data; 

}
TSDOINFOHEADER;




 
typedef struct 
{
  TMBXHEADER        MbxHeader;  
  TCOEHEADER        CoeHeader;  
  TSDOINFOHEADER    SdoHeader;  
}
TSDOINFORMATION;


#line 579 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 







 




 








 









 
extern unsigned char         u8PendingSdo;  
extern unsigned char          bStoreCompleteAccess;  
extern unsigned char         u8StoreSubindex;  
extern unsigned short        u16StoreIndex;  
extern unsigned long        u32StoreDataSize;  
extern unsigned short  *pStoreData;  
extern unsigned char (* pSdoPendFunc)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
 


extern unsigned long                      aSdoInfoHeader[(((((( (6) + (2) + (4) ))+(2)))+3) >> 2)];  





 
extern unsigned short  *                 pSdoSegData;  
extern unsigned short                          nSdoInfoFragmentsLeft;  
 






 
extern    unsigned char SDOS_SdoInfoInd(TSDOINFORMATION  *pSdoInfoInd);
extern    unsigned char SDOS_SdoInd(TINITSDOMBX  *pSdoInd);

extern    void  SDOS_SdoRes(unsigned char abort, unsigned long objLength, unsigned short  *pData);
 
extern    void  SODS_ClearPendingResponse(void);
 



 
#line 42 "..\\For_SSC_Tool\\Src\\ecatcoe.c"
#line 1 "..\\For_SSC_Tool\\Src\\ecatcoe.h"



 




 












 





 
#line 1 "..\\For_SSC_Tool\\Src\\mailbox.h"



 




 
















 





 
#line 34 "..\\For_SSC_Tool\\Src\\mailbox.h"


#line 142 "..\\For_SSC_Tool\\Src\\mailbox.h"






 






extern unsigned char                    bReceiveMbxIsLocked;  
extern unsigned char                    bSendMbxIsFull;  
extern unsigned char                    bMbxRunning;  
extern unsigned char                    bMbxRepeatToggle;  
extern unsigned short                  u16SendMbxSize;  
extern unsigned short                  u16ReceiveMbxSize;  
extern unsigned short                  u16EscAddrReceiveMbx;  
extern unsigned short                  u16EscAddrSendMbx;  
extern unsigned char                   u8MbxWriteCounter;  
extern unsigned char                   u8MbxReadCounter;  
extern unsigned char                   u8MailboxSendReqStored;  
extern TMBX  *           psWriteMbx;  
extern TMBX  *           psReadMbx;  
extern TMBX  *           psRepeatMbx;  
extern TMBX  *           psStoreMbx;  
extern TMBXQUEUE         sMbxSendQueue;  
extern TMBXQUEUE         sMbxReceiveQueue;  






 

extern   void     MBX_Init(void);
extern   unsigned short    MBX_StartMailboxHandler(void);
extern   void     MBX_StopMailboxHandler(void);
extern   void     MBX_MailboxWriteInd(TMBX  *pMbx);
extern   void     MBX_MailboxReadInd(void);
extern   void     MBX_MailboxRepeatReq(void);
extern   unsigned char    MBX_MailboxSendReq(TMBX  * pMbx, unsigned char flags);
extern   void     MBX_CheckAndCopyMailbox(void);
extern   unsigned char    MBX_CopyToSendMailbox(TMBX  *pMbx);
extern   void     MBX_Main(void);


 
#line 30 "..\\For_SSC_Tool\\Src\\ecatcoe.h"

#line 85 "..\\For_SSC_Tool\\Src\\ecatcoe.h"











 
    TMBX  *  pCoeSendStored;                



 





 

   void     COE_Init(void);
   unsigned char    COE_ServiceInd(TCOEMBX  *pCoeMbx);
   unsigned char     COE_ContinueInd(TMBX  * pMbx);


 
#line 44 "..\\For_SSC_Tool\\Src\\ecatcoe.c"

 








 








 





 





 





 

void COE_Init(void)
{
    pCoeSendStored = 0;
    nSdoInfoFragmentsLeft = 0;
}










 

unsigned char COE_ServiceInd(TCOEMBX  *pCoeMbx)
{
    unsigned char result = 0;

    switch ((pCoeMbx->CoeHeader & 0xF000) >> 12)
    {
    case 0x02:
        

 
        result = SDOS_SdoInd( (TINITSDOMBX  *) pCoeMbx );
        break;

    case 0x08:
         
        result = SDOS_SdoInfoInd( (TSDOINFORMATION  *) pCoeMbx );
         
        break;


    case 0x03:
    case 0x01:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
         
        result = 0x04;
        break;

    default:
        result = 0x05;
        break;
    }
    return result;
}









 

unsigned char COE_ContinueInd(TMBX  * pMbx)
{
    if (pCoeSendStored)
    {
         
        MBX_MailboxSendReq(pCoeSendStored, 0);
        pCoeSendStored = 0;
    }
    else
    {
         
         
        pMbx = (TMBX  *) malloc((sizeof(TMBX)));
         
        if (pMbx == 0)
        {
            return 0x07;
        }
        else
        {
             
            memcpy(pMbx, aSdoInfoHeader, ((( (6) + (2) + (4) ))+(2)));
             
            SDOS_SdoInfoInd( (TSDOINFORMATION  *) pMbx );
        }
    }

    return 0;
}

 



