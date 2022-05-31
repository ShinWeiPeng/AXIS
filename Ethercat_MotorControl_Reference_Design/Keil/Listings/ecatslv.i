#line 1 "..\\For_SSC_Tool\\Src\\ecatslv.c"



 




 











































































































































































 





 

#line 1 "..\\For_SSC_Tool\\Src\\ecatslv.h"



 




 
























 





 

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





 








 







 






 






 





 





 






 






 





 





 





 






 







 






 





 





 





 





 





 







 







 





 






 






 





 





 





 






 





 






 






 





 





 






 






 








 





 






 





 







 





 







 







 





 





 





 





 






 





 






 






 






 





 





 





 





 









 


 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 





 






 






 






 






 






 






 






 





 





 






 






 






 





 





 





 






 






 





 





 






 






 





 





 





 





 





 





 





 





 





 





 





 





 





 





 









 


 





 







 





 





 





 





 





 





 





 





 





 





 






 






 






 






 





 





 





 





 





 





 





 





 






 





 





 






 






 





 










 




#line 43 "..\\For_SSC_Tool\\Src\\ecatslv.h"

#line 1 "..\\For_SSC_Tool\\Src\\esc.h"



 




 














 





 
#line 32 "..\\For_SSC_Tool\\Src\\esc.h"








 








 



































 


 


 
#line 102 "..\\For_SSC_Tool\\Src\\esc.h"
 














#line 123 "..\\For_SSC_Tool\\Src\\esc.h"









 
 
typedef struct 
{

  unsigned short                        PhysicalStartAddress;  
  unsigned short                        Length;  


      unsigned char                         Settings[4];  



 
#line 154 "..\\For_SSC_Tool\\Src\\esc.h"



 




 







 



}
TSYNCMAN;




 
#line 45 "..\\For_SSC_Tool\\Src\\ecatslv.h"
#line 1 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"




























 




 






 




 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 48 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\NuMicro.h"
 





 



#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
 







 






































 







 
 
 



 



 
typedef enum IRQn
{
     
    NonMaskableInt_IRQn           = -14,       
    MemoryManagement_IRQn         = -12,       
    BusFault_IRQn                 = -11,       
    UsageFault_IRQn               = -10,       
    SVCall_IRQn                   = -5,        
    DebugMonitor_IRQn             = -4,        
    PendSV_IRQn                   = -2,        
    SysTick_IRQn                  = -1,        

     

    BOD_IRQn                      = 0,         
    IRC_IRQn                      = 1,         
    PWRWU_IRQn                    = 2,         
    RAMPE_IRQn                    = 3,         
    CKFAIL_IRQn                   = 4,         
    RTC_IRQn                      = 6,         
    TAMPER_IRQn                   = 7,         
    WDT_IRQn                      = 8,         
    WWDT_IRQn                     = 9,         
    EINT0_IRQn                    = 10,        
    EINT1_IRQn                    = 11,        
    EINT2_IRQn                    = 12,        
    EINT3_IRQn                    = 13,        
    EINT4_IRQn                    = 14,        
    EINT5_IRQn                    = 15,        
    GPA_IRQn                      = 16,        
    GPB_IRQn                      = 17,        
    GPC_IRQn                      = 18,        
    GPD_IRQn                      = 19,        
    GPE_IRQn                      = 20,        
    GPF_IRQn                      = 21,        
    QSPI0_IRQn                    = 22,        
    SPI0_IRQn                     = 23,        
    BRAKE0_IRQn                   = 24,        
    EPWM0P0_IRQn                  = 25,        
    EPWM0P1_IRQn                  = 26,        
    EPWM0P2_IRQn                  = 27,        
    BRAKE1_IRQn                   = 28,        
    EPWM1P0_IRQn                  = 29,        
    EPWM1P1_IRQn                  = 30,        
    EPWM1P2_IRQn                  = 31,        
    TMR0_IRQn                     = 32,        
    TMR1_IRQn                     = 33,        
    TMR2_IRQn                     = 34,        
    TMR3_IRQn                     = 35,        
    UART0_IRQn                    = 36,        
    UART1_IRQn                    = 37,        
    I2C0_IRQn                     = 38,        
    I2C1_IRQn                     = 39,        
    PDMA_IRQn                     = 40,        
    DAC_IRQn                      = 41,        
    EADC00_IRQn                   = 42,        
    EADC01_IRQn                   = 43,        
    ACMP01_IRQn                   = 44,        
    EADC02_IRQn                   = 46,        
    EADC03_IRQn                   = 47,        
    UART2_IRQn                    = 48,        
    UART3_IRQn                    = 49,        
    QSPI1_IRQn                    = 50,        
    SPI1_IRQn                     = 51,        
    SPI2_IRQn                     = 52,        
    USBD_IRQn                     = 53,        
    USBH_IRQn                     = 54,        
    USBOTG_IRQn                   = 55,        
    CAN0_IRQn                     = 56,        
    CAN1_IRQn                     = 57,        
    SC0_IRQn                      = 58,        
    SC1_IRQn                      = 59,        
    SC2_IRQn                      = 60,        
    SPI3_IRQn                     = 62,        
    EMAC_TX_IRQn                  = 66,        
    EMAC_RX_IRQn                  = 67,        
    SDH0_IRQn                     = 64,        
    USBD20_IRQn                   = 65,        
    I2S0_IRQn                     = 68,        
    OPA_IRQn                      = 70,        
    CRPT_IRQn                     = 71,        
    GPG_IRQn                      = 72,        
    EINT6_IRQn                    = 73,        
    UART4_IRQn                    = 74,        
    UART5_IRQn                    = 75,        
    USCI0_IRQn                    = 76,        
    USCI1_IRQn                    = 77,        
    BPWM0_IRQn                    = 78,        
    BPWM1_IRQn                    = 79,        
    SPIM_IRQn                     = 80,        
    CCAP_IRQn                     = 81,        
    I2C2_IRQn                     = 82,        
    QEI0_IRQn                     = 84,        
    QEI1_IRQn                     = 85,        
    ECAP0_IRQn                    = 86,        
    ECAP1_IRQn                    = 87,        
    GPH_IRQn                      = 88,        
    EINT7_IRQn                    = 89,        
    SDH1_IRQn                     = 90,        
    HSUSBH_IRQn                   = 92,        
    USBOTG20_IRQn                 = 93,        
    TRNG_IRQn                     = 101,       
    UART6_IRQn                    = 102,       
    UART7_IRQn                    = 103,       
    EADC10_IRQn                   = 104,       
    EADC11_IRQn                   = 105,       
    EADC12_IRQn                   = 106,       
    EADC13_IRQn                   = 107,       
    CAN2_IRQn                     = 108,       
}
IRQn_Type;






 

 
#line 197 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   


#line 1 "..\\Library\\CMSIS\\Include\\core_cm4.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\Library\\CMSIS\\Include\\core_cm4.h"

















 




 



 

 













#line 120 "..\\Library\\CMSIS\\Include\\core_cm4.h"



 
#line 135 "..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 209 "..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 1 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"











 


#line 54 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

#line 211 "..\\Library\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

#line 212 "..\\Library\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "..\\Library\\CMSIS\\Include\\core_cmSimd.h"

 
#line 88 "..\\Library\\CMSIS\\Include\\core_cmSimd.h"

 






#line 213 "..\\Library\\CMSIS\\Include\\core_cm4.h"
















 
#line 256 "..\\Library\\CMSIS\\Include\\core_cm4.h"

 






 
#line 272 "..\\Library\\CMSIS\\Include\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 "..\\Library\\CMSIS\\Include\\core_cm4.h"

#line 1550 "..\\Library\\CMSIS\\Include\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4UL)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4UL)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4UL)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4UL)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4UL)) ? (uint32_t)(4UL) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4UL)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4UL));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4UL)) ? (uint32_t)(4UL) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4UL)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4UL));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4UL) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 202 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"
 





 








#line 17 "..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"




 

#line 29 "..\\Library\\Device\\Nuvoton\\M480\\Include\\system_M480.h"




extern uint32_t SystemCoreClock;      
extern uint32_t CyclesPerUs;          
extern uint32_t PllClock;             










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);







 
#line 203 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 204 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"




#pragma anon_unions


 
 
 

#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\sys_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile const  uint32_t PDID;                   
    volatile uint32_t RSTSTS;                 
    volatile uint32_t IPRST0;                 
    volatile uint32_t IPRST1;                 
    volatile uint32_t IPRST2;                 
     
    volatile const  uint32_t RESERVE0[1];
     
    volatile uint32_t BODCTL;                 
    volatile uint32_t IVSCTL;                 
     
    volatile const  uint32_t RESERVE1[1];
     
    volatile uint32_t PORCTL;                 
    volatile uint32_t VREFCTL;                
    volatile uint32_t USBPHY;                 
    volatile uint32_t GPA_MFPL;               
    volatile uint32_t GPA_MFPH;               
    volatile uint32_t GPB_MFPL;               
    volatile uint32_t GPB_MFPH;               
    volatile uint32_t GPC_MFPL;               
    volatile uint32_t GPC_MFPH;               
    volatile uint32_t GPD_MFPL;               
    volatile uint32_t GPD_MFPH;               
    volatile uint32_t GPE_MFPL;               
    volatile uint32_t GPE_MFPH;               
    volatile uint32_t GPF_MFPL;               
    volatile uint32_t GPF_MFPH;               
    volatile uint32_t GPG_MFPL;               
    volatile uint32_t GPG_MFPH;               
    volatile uint32_t GPH_MFPL;               
    volatile uint32_t GPH_MFPH;               
     
    volatile const  uint32_t RESERVE2[4];
     
    volatile uint32_t GPA_MFOS;               
    volatile uint32_t GPB_MFOS;               
    volatile uint32_t GPC_MFOS;               
    volatile uint32_t GPD_MFOS;               
    volatile uint32_t GPE_MFOS;               
    volatile uint32_t GPF_MFOS;               
    volatile uint32_t GPG_MFOS;               
    volatile uint32_t GPH_MFOS;               
     
    volatile const  uint32_t RESERVE3[8];
     
    volatile uint32_t SRAM_INTCTL;            
    volatile uint32_t SRAM_STATUS;            
    volatile const  uint32_t SRAM_ERRADDR;           
     
    volatile const  uint32_t RESERVE4[1];
     
    volatile uint32_t SRAM_BISTCTL;           
    volatile const  uint32_t SRAM_BISTSTS;           
     
    volatile const  uint32_t RESERVE5[3];
     
    volatile uint32_t HIRCTCTL;               
    volatile uint32_t HIRCTIEN;               
    volatile uint32_t HIRCTISTS;              
    volatile uint32_t IRCTCTL;                
    volatile uint32_t IRCTIEN;                
    volatile uint32_t IRCTISTS;               
     
    volatile const  uint32_t RESERVE6[1];
     
    volatile uint32_t REGLCTL;                
     
    volatile const  uint32_t RESERVE7[58];
     
    volatile uint32_t PORDISAN;               
     
    volatile const  uint32_t RESERVE8;
     
    volatile const  uint32_t CSERVER;                
    volatile uint32_t PLCTL;                  
    volatile const  uint32_t PLSTS;                  
     
    volatile const  uint32_t RESERVE9[128];
     
    volatile uint32_t AHBMCTL;                

} SYS_T;




 












































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   




 

typedef struct
{


    



















































































































 
    volatile uint32_t NMIEN;                  
    volatile const  uint32_t NMISTS;                 

} NMI_T;




 



























































































   
   
   


#pragma no_anon_unions


#line 216 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\clk_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t PWRCTL;                 
    volatile uint32_t AHBCLK;                 
    volatile uint32_t APBCLK0;                
    volatile uint32_t APBCLK1;                
    volatile uint32_t CLKSEL0;                
    volatile uint32_t CLKSEL1;                
    volatile uint32_t CLKSEL2;                
    volatile uint32_t CLKSEL3;                
    volatile uint32_t CLKDIV0;                
    volatile uint32_t CLKDIV1;                
    volatile uint32_t CLKDIV2;                
    volatile uint32_t CLKDIV3;                
    volatile uint32_t CLKDIV4;                
    volatile uint32_t PCLKDIV;                
     
    volatile const  uint32_t RESERVE1[2];
     
    volatile uint32_t PLLCTL;                 
     
    volatile const  uint32_t RESERVE2[3];
     
    volatile const  uint32_t STATUS;                 
     
    volatile const  uint32_t RESERVE3[3];
     
    volatile uint32_t CLKOCTL;                
     
    volatile const  uint32_t RESERVE4[3];
     
    volatile uint32_t CLKDCTL;                
    volatile uint32_t CLKDSTS;                
    volatile uint32_t CDUPB;                  
    volatile uint32_t CDLOWB;                 
     
    volatile const  uint32_t RESERVE5[4];
     
    volatile uint32_t PMUCTL;                 
    volatile uint32_t PMUSTS;                 
    volatile uint32_t LDOCTL;                 
    volatile uint32_t SWKDBCTL;               
    volatile uint32_t PASWKCTL;               
    volatile uint32_t PBSWKCTL;               
    volatile uint32_t PCSWKCTL;               
    volatile uint32_t PDSWKCTL;               
    volatile uint32_t IOPDCTL;                

} CLK_T;




 






































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 217 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\fmc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    

























































































































































































































































































































































































































 
    volatile uint32_t ISPCTL;                 
    volatile uint32_t ISPADDR;                
    volatile uint32_t ISPDAT;                 
    volatile uint32_t ISPCMD;                 
    volatile uint32_t ISPTRG;                 
    volatile const  uint32_t DFBA;                   
    
    volatile const  uint32_t RESERVE0[10];
    
    volatile uint32_t ISPSTS;                 
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CYCCTL;                 
    volatile  uint32_t KPKEY0;                 
    volatile  uint32_t KPKEY1;                 
    volatile  uint32_t KPKEY2;                 
    volatile uint32_t KPKEYTRG;               
    volatile uint32_t KPKEYSTS;               
    volatile const  uint32_t KPKEYCNT;               
    volatile const  uint32_t KPCNT;                  
    
    volatile const  uint32_t RESERVE2[5];
    
    volatile uint32_t MPDAT0;                 
    volatile uint32_t MPDAT1;                 
    volatile uint32_t MPDAT2;                 
    volatile uint32_t MPDAT3;                 
    
    volatile const  uint32_t RESERVE3[12];
    
    volatile const  uint32_t MPSTS;                  
    volatile const  uint32_t MPADDR;                 
    
    volatile const  uint32_t RESERVE4[2];
    
    volatile const  uint32_t XOMR0STS;               
    volatile const  uint32_t XOMR1STS;               
    volatile const  uint32_t XOMR2STS;               
    volatile const  uint32_t XOMR3STS;               
    volatile const  uint32_t XOMSTS;                 

} FMC_T;




 

































































































































































































   
   
   


#pragma no_anon_unions


#line 218 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\gpio_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    








































































































































































































 

    volatile uint32_t MODE;           
    volatile uint32_t DINOFF;         
    volatile uint32_t DOUT;           
    volatile uint32_t DATMSK;         
    volatile const  uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t INTTYPE;        
    volatile uint32_t INTEN;          
    volatile uint32_t INTSRC;         
    volatile uint32_t SMTEN;          
    volatile uint32_t SLEWCTL;        
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t PUSEL;          

} GPIO_T;

typedef struct
{

    





























 

    volatile uint32_t DBCTL;          

} GPIO_DBCTL_T;




 


























































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 219 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\pdma_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    





















































































 
    volatile uint32_t CTL;              
    volatile uint32_t SA;               
    volatile uint32_t DA;               
    volatile uint32_t NEXT;             
} DSCT_T;


typedef struct
{
    
















 
    volatile uint32_t STCR;            
    volatile uint32_t ASOCR;           
} STRIDE_T;

typedef struct
{
    
















 
    volatile uint32_t AICTL;          
    volatile uint32_t RCNT;           
} REPEAT_T;

typedef struct
{


    



























































































































































































































































































































































































 
    DSCT_T DSCT[16];
    volatile const  uint32_t CURSCAT[16];               
    
    volatile const  uint32_t RESERVE1[176];
    
    volatile uint32_t CHCTL;                  
    volatile  uint32_t PAUSE;                  
    volatile  uint32_t SWREQ;                  
    volatile const  uint32_t TRGSTS;                 
    volatile uint32_t PRISET;                 
    volatile  uint32_t PRICLR;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t ABTSTS;                 
    volatile uint32_t TDSTS;                  
    volatile uint32_t ALIGN;                  
    volatile const  uint32_t TACTSTS;                
    volatile uint32_t TOUTPSC;                
    volatile uint32_t TOUTEN;                 
    volatile uint32_t TOUTIEN;                
    volatile uint32_t SCATBA;                 
    volatile uint32_t TOC0_1;                 
    
    volatile const  uint32_t RESERVE2[7];
    
    volatile uint32_t CHRST;                  
    
    volatile const  uint32_t RESERVE3[7];
    
    volatile uint32_t REQSEL0_3;              
    volatile uint32_t REQSEL4_7;              
    volatile uint32_t REQSEL8_11;             
    volatile uint32_t REQSEL12_15;            
    
    volatile const  uint32_t RESERVE4[28];
    
    STRIDE_T     STRIDE[6];
    
    volatile uint32_t RESERVE5[52];
    
    REPEAT_T    REPEAT[2];
} PDMA_T;




 


















































































































































































































































































   
   
   


#pragma no_anon_unions


#line 220 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\timer_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    









































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CMP;                    
    volatile uint32_t INTSTS;                 
    volatile uint32_t CNT;                    
    volatile const  uint32_t CAP;                    
    volatile uint32_t EXTCTL;                 
    volatile uint32_t EINTSTS;                
    volatile uint32_t TRGCTL;                 
    volatile uint32_t ALTCTL;                 
     
    volatile const  uint32_t RESERVE0[7];
     
    volatile uint32_t PWMCTL;                 
    volatile uint32_t PWMCLKSRC;              
    volatile uint32_t PWMCLKPSC;              
    volatile uint32_t PWMCNTCLR;              
    volatile uint32_t PWMPERIOD;              
    volatile uint32_t PWMCMPDAT;              
    volatile uint32_t PWMDTCTL;               
    volatile const  uint32_t PWMCNT;                 
    volatile uint32_t PWMMSKEN;               
    volatile uint32_t PWMMSK;                 
    volatile uint32_t PWMBNF;                 
    volatile uint32_t PWMFAILBRK;             
    volatile uint32_t PWMBRKCTL;              
    volatile uint32_t PWMPOLCTL;              
    volatile uint32_t PWMPOEN;                
    volatile  uint32_t PWMSWBRK;               
    volatile uint32_t PWMINTEN0;              
    volatile uint32_t PWMINTEN1;              
    volatile uint32_t PWMINTSTS0;             
    volatile uint32_t PWMINTSTS1;             
    volatile uint32_t PWMEADCTS;              
    volatile uint32_t PWMSCTL;                
    volatile  uint32_t PWMSTRG;                
    volatile uint32_t PWMSTATUS;              
    volatile const  uint32_t PWMPBUF;                
    volatile const  uint32_t PWMCMPBUF;              

} TIMER_T;




 








































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 221 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\wdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





























































































 
    volatile uint32_t CTL;                    
    volatile uint32_t ALTCTL;                 
    volatile  uint32_t RSTCNT;                 

} WDT_T;




 








































   
   
   


#pragma no_anon_unions


#line 222 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\wwdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






































































 
    volatile  uint32_t RLDCNT;                 
    volatile uint32_t CTL;                    
    volatile uint32_t STATUS;                 
    volatile const  uint32_t CNT;                    

} WWDT_T;




 




























   
   
   


#pragma no_anon_unions


#line 223 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\rtc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t INIT;                   
    volatile uint32_t RWEN;                   
    volatile uint32_t FREQADJ;                
    volatile uint32_t TIME;                   
    volatile uint32_t CAL;                    
    volatile uint32_t CLKFMT;                 
    volatile uint32_t WEEKDAY;                
    volatile uint32_t TALM;                   
    volatile uint32_t CALM;                   
    volatile const  uint32_t LEAPYEAR;               
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t TICK;                   
    volatile uint32_t TAMSK;                  
    volatile uint32_t CAMSK;                  
    volatile uint32_t SPRCTL;                 
    volatile uint32_t SPR[20];                
    
    volatile const  uint32_t RESERVE0[28];
    
    volatile uint32_t LXTCTL;                 
    volatile uint32_t GPIOCTL0;               
    volatile uint32_t GPIOCTL1;               
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t DSTCTL;                 
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t TAMPCTL;                
    
    volatile const  uint32_t RESERVE3[1];
    
    volatile uint32_t TAMPSEED;               
    
    volatile const  uint32_t RESERVE4[1];
    
    volatile const  uint32_t TAMPTIME;               
    volatile const  uint32_t TAMPCAL;                

} RTC_T;




 





































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 224 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\epwm_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    














 
    volatile uint32_t RCAPDAT;  
    volatile uint32_t FCAPDAT;  
} ECAPDAT_T;

typedef struct
{


    














































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile uint32_t SYNC;                   
    volatile uint32_t SWSYNC;                 
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC[3];              
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    volatile uint32_t LOAD;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t PERIOD[6];              
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CMPDAT[6];              
    
    volatile const  uint32_t RESERVE2[2];
    
    volatile uint32_t DTCTL[3];               
    
    volatile const  uint32_t RESERVE3[1];
    
    volatile uint32_t PHS[3];                 
    
    volatile const  uint32_t RESERVE4[1];
    
    volatile const  uint32_t CNT[6];                 
    
    volatile const  uint32_t RESERVE5[2];
    
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    volatile uint32_t BNF;                    
    volatile uint32_t FAILBRK;                
    volatile uint32_t BRKCTL[3];              
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    volatile  uint32_t SWBRK;                  
    volatile uint32_t INTEN0;                 
    volatile uint32_t INTEN1;                 
    volatile uint32_t INTSTS0;                
    volatile uint32_t INTSTS1;                
    
    volatile const  uint32_t RESERVE6[1];
    
    volatile uint32_t DACTRGEN;               
    volatile uint32_t EADCTS0;                
    volatile uint32_t EADCTS1;                
    volatile uint32_t FTCMPDAT[3];            
    
    volatile const  uint32_t RESERVE7[1];
    
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    volatile uint32_t LEBCTL;                 
    volatile uint32_t LEBCNT;                 
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE8[3];
    
    volatile uint32_t IFA[6];                 
    
    volatile const  uint32_t RESERVE9[2];
    
    volatile uint32_t AINTSTS;                
    volatile uint32_t AINTEN;                 
    volatile uint32_t APDMACTL;               
    
    volatile const  uint32_t RESERVE10[41];
    
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    ECAPDAT_T CAPDAT[6];                   
    volatile uint32_t PDMACTL;                
    volatile const  uint32_t PDMACAP[3];             
    
    volatile const  uint32_t RESERVE11[1];
    
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    
    volatile const  uint32_t RESERVE12[43];
    
    volatile const  uint32_t PBUF[6];                
    volatile const  uint32_t CMPBUF[6];              
    volatile const  uint32_t CPSCBUF[3];             
    volatile const  uint32_t FTCBUF[3];              
    volatile uint32_t FTCI;                   

} EPWM_T;




 

















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 225 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\bpwm_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    














 
    volatile uint32_t RCAPDAT;  
    volatile uint32_t FCAPDAT;  
} BCAPDAT_T;

typedef struct
{


    


























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC;                 
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    
    volatile const  uint32_t RESERVE2[2];
    
    volatile uint32_t PERIOD;                 
    
    volatile const  uint32_t RESERVE3[7];
    
    volatile uint32_t CMPDAT[6];              
    
    volatile const  uint32_t RESERVE4[10];
    
    volatile const  uint32_t CNT;                    
    
    volatile const  uint32_t RESERVE5[7];
    
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    
    volatile const  uint32_t RESERVE6[5];
    
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    
    volatile const  uint32_t RESERVE7[1];
    
    volatile uint32_t INTEN;                  
    
    volatile const  uint32_t RESERVE8[1];
    
    volatile uint32_t INTSTS;                 
    
    volatile const  uint32_t RESERVE9[3];
    
    volatile uint32_t EADCTS0;                
    volatile uint32_t EADCTS1;                
    
    volatile const  uint32_t RESERVE10[4];
    
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    
    volatile const  uint32_t RESERVE11[2];
    
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE12[55];
    
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    BCAPDAT_T CAPDAT[6];                   
    
    volatile const  uint32_t RESERVE13[5];
    
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    
    volatile const  uint32_t RESERVE14[43];
    
    volatile const  uint32_t PBUF;                   
    
    volatile const  uint32_t RESERVE15[5];
    
    volatile const  uint32_t CMPBUF[6];              

} BPWM_T;




 






























































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 226 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\qei_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































 
    volatile uint32_t CNT;                    
    volatile uint32_t CNTHOLD;                
    volatile uint32_t CNTLATCH;               
    volatile uint32_t CNTCMP;                 
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t CNTMAX;                 
    volatile uint32_t CTL;                    
    
    volatile const  uint32_t RESERVE1[4];
    
    volatile uint32_t STATUS;                 

} QEI_T;




 

































































































   
   
   


#pragma no_anon_unions


#line 227 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\ecap_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    

















































































































































































































 
    volatile uint32_t CNT;                    
    volatile uint32_t HLD0;                   
    volatile uint32_t HLD1;                   
    volatile uint32_t HLD2;                   
    volatile uint32_t CNTCMP;                 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS;                 

} ECAP_T;




 































































































































   
   
   


#pragma no_anon_unions


#line 228 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\uart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t DAT;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t FIFO;                   
    volatile uint32_t LINE;                   
    volatile uint32_t MODEM;                  
    volatile uint32_t MODEMSTS;               
    volatile uint32_t FIFOSTS;                
    volatile uint32_t INTSTS;                 
    volatile uint32_t TOUT;                   
    volatile uint32_t BAUD;                   
    volatile uint32_t IRDA;                   
    volatile uint32_t ALTCTL;                 
    volatile uint32_t FUNCSEL;                
    volatile uint32_t LINCTL;                 
    volatile uint32_t LINSTS;                 
    volatile uint32_t BRCOMP;                 
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t DWKCOMP;                

} UART_T;




 

























































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 229 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\emac_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    












































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CAMCTL;                 
    volatile uint32_t CAMEN;                  
    volatile uint32_t CAM0M;                  
    volatile uint32_t CAM0L;                  
    volatile uint32_t CAM1M;                  
    volatile uint32_t CAM1L;                  
    volatile uint32_t CAM2M;                  
    volatile uint32_t CAM2L;                  
    volatile uint32_t CAM3M;                  
    volatile uint32_t CAM3L;                  
    volatile uint32_t CAM4M;                  
    volatile uint32_t CAM4L;                  
    volatile uint32_t CAM5M;                  
    volatile uint32_t CAM5L;                  
    volatile uint32_t CAM6M;                  
    volatile uint32_t CAM6L;                  
    volatile uint32_t CAM7M;                  
    volatile uint32_t CAM7L;                  
    volatile uint32_t CAM8M;                  
    volatile uint32_t CAM8L;                  
    volatile uint32_t CAM9M;                  
    volatile uint32_t CAM9L;                  
    volatile uint32_t CAM10M;                 
    volatile uint32_t CAM10L;                 
    volatile uint32_t CAM11M;                 
    volatile uint32_t CAM11L;                 
    volatile uint32_t CAM12M;                 
    volatile uint32_t CAM12L;                 
    volatile uint32_t CAM13M;                 
    volatile uint32_t CAM13L;                 
    volatile uint32_t CAM14M;                 
    volatile uint32_t CAM14L;                 
    volatile uint32_t CAM15MSB;               
    volatile uint32_t CAM15LSB;               
    volatile uint32_t TXDSA;                  
    volatile uint32_t RXDSA;                  
    volatile uint32_t CTL;                    
    volatile uint32_t MIIMDAT;                
    volatile uint32_t MIIMCTL;                
    volatile uint32_t FIFOCTL;                
    volatile  uint32_t TXST;                   
    volatile  uint32_t RXST;                   
    volatile uint32_t MRFL;                   
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t GENSTS;                 
    volatile uint32_t MPCNT;                  
    volatile const  uint32_t RPCNT;                  
     
    volatile const  uint32_t RESERVE0[2];
     
    volatile uint32_t FRSTS;                  
    volatile const  uint32_t CTXDSA;                 
    volatile const  uint32_t CTXBSA;                 
    volatile const  uint32_t CRXDSA;                 
    volatile const  uint32_t CRXBSA;                 
     
    volatile const  uint32_t RESERVE1[9];
     
    volatile uint32_t TSCTL;                  
     
    volatile const  uint32_t RESERVE2[3];
     
    volatile const  uint32_t TSSEC;                  
    volatile const  uint32_t TSSUBSEC;               
    volatile uint32_t TSINC;                  
    volatile uint32_t TSADDEND;               
    volatile uint32_t UPDSEC;                 
    volatile uint32_t UPDSUBSEC;              
    volatile uint32_t ALMSEC;                 
    volatile uint32_t ALMSUBSEC;              

} EMAC_T;




 














































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 230 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\sc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    













































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t DAT;                    
    volatile uint32_t CTL;                    
    volatile uint32_t ALTCTL;                 
    volatile uint32_t EGT;                    
    volatile uint32_t RXTOUT;                 
    volatile uint32_t ETUCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t STATUS;                 
    volatile uint32_t PINCTL;                 
    volatile uint32_t TMRCTL0;                
    volatile uint32_t TMRCTL1;                
    volatile uint32_t TMRCTL2;                
    volatile uint32_t UARTCTL;                
     
    volatile const  uint32_t RESERVE0[5];
     
    volatile uint32_t ACTCTL;                 

} SC_T;




 













































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 231 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\i2s_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    













































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CLKDIV;                 
    volatile uint32_t IEN;                    
    volatile uint32_t STATUS0;                
    volatile  uint32_t TXFIFO;                 
    volatile const  uint32_t RXFIFO;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS1;                

} I2S_T;




 




























































































































































































































   
   
   


#pragma no_anon_unions


#line 232 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\spi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








































































































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile  uint32_t TX;                     
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile const  uint32_t RX;                     
    
    volatile const  uint32_t RESERVE2[11];
    
    volatile uint32_t I2SCTL;                 
    volatile uint32_t I2SCLK;                 
    volatile uint32_t I2SSTS;                 

} SPI_T;




 





















































































































































































































































































   
   
   


#pragma no_anon_unions


#line 233 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\qspi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    











































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile  uint32_t TX;                     
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile const  uint32_t RX;                     

} QSPI_T;




 








































































































































































































   
   
   


#pragma no_anon_unions


#line 234 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\spim_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

























































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t RXCLKDLY;               
    volatile const  uint32_t RX[4];                  
    volatile uint32_t TX[4];                  
    volatile uint32_t SRAMADDR;               
    volatile uint32_t DMACNT;                 
    volatile uint32_t FADDR;                  
    volatile  uint32_t KEY1;                   
    volatile  uint32_t KEY2;                   
    volatile uint32_t DMMCTL;                 
    volatile uint32_t CTL2;                   

} SPIM_T;




 






















































































































   
   
   


#pragma no_anon_unions


#line 235 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\i2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

















































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t ADDR0;                  
    volatile uint32_t DAT;                    
    volatile const  uint32_t STATUS0;                
    volatile uint32_t CLKDIV;                 
    volatile uint32_t TOCTL;                  
    volatile uint32_t ADDR1;                  
    volatile uint32_t ADDR2;                  
    volatile uint32_t ADDR3;                  
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t ADDRMSK2;               
    volatile uint32_t ADDRMSK3;               
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS1;                
    volatile uint32_t TMCTL;                  
    volatile uint32_t BUSCTL;                 
    volatile uint32_t BUSTCTL;                
    volatile uint32_t BUSSTS;                 
    volatile uint32_t PKTSIZE;                
    volatile const  uint32_t PKTCRC;                 
    volatile uint32_t BUSTOUT;                
    volatile uint32_t CLKTOUT;                

} I2C_T;




 

























































































































































































































   
   
   


#pragma no_anon_unions


#line 236 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\uuart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t DATIN0;                 
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile uint32_t CTLIN0;                 
    
    volatile const  uint32_t RESERVE2[1];
    
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile uint32_t TXDAT;                  
    volatile uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    
    volatile const  uint32_t RESERVE3[4];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} UUART_T;




 



















































































































































































































   
   
   


#pragma no_anon_unions


#line 237 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\uspi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    












































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t DATIN0;                 
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile uint32_t CTLIN0;                 
    
    volatile const  uint32_t RESERVE2[1];
    
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    
    volatile const  uint32_t RESERVE3[4];
    
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} USPI_T;




 













































































































































































































   
   
   


#pragma no_anon_unions


#line 238 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\ui2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






















































































































































































































































































































































 
    volatile uint32_t CTL;                    
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t BRGEN;                  
    
    volatile const  uint32_t RESERVE1[8];
    
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t DEVADDR0;               
    volatile uint32_t DEVADDR1;               
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                
    
    volatile const  uint32_t RESERVE3[8];
    
    volatile uint32_t ADMAT;                  
    volatile uint32_t TMCTL;                  

} UI2C_T;




 






































































































































































   
   
   


#pragma no_anon_unions


#line 239 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\can_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{

    























































































































































































































 
    volatile uint32_t CREQ;          
    volatile uint32_t CMASK;         
    volatile uint32_t MASK1;         
    volatile uint32_t MASK2;         
    volatile uint32_t ARB1;          
    volatile uint32_t ARB2;          
    volatile uint32_t MCON;          
    volatile uint32_t DAT_A1;        
    volatile uint32_t DAT_A2;        
    volatile uint32_t DAT_B1;        
    volatile uint32_t DAT_B2;        
    
    volatile const uint32_t RESERVE0[13];
    
} CAN_IF_T;


typedef struct
{


    


























































































































































































































 
    volatile uint32_t CON;                    
    volatile uint32_t STATUS;                 
    volatile const  uint32_t ERR;                    
    volatile uint32_t BTIME;                  
    volatile const  uint32_t IIDR;                   
    volatile uint32_t TEST;                   
    volatile uint32_t BRPE;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile CAN_IF_T IF[2];
    
    volatile const  uint32_t RESERVE2[8];
    
    volatile const  uint32_t TXREQ1;                 
    volatile const  uint32_t TXREQ2;                 
    
    volatile const  uint32_t RESERVE3[6];
    
    volatile const  uint32_t NDAT1;                  
    volatile const  uint32_t NDAT2;                  
    
    volatile const  uint32_t RESERVE4[6];
    
    volatile const  uint32_t IPND1;                  
    volatile const  uint32_t IPND2;                  
    
    volatile const  uint32_t RESERVE5[6];
    
    volatile const  uint32_t MVLD1;                  
    volatile const  uint32_t MVLD2;                  
    volatile uint32_t WU_EN;                  
    volatile uint32_t WU_STATUS;              

} CAN_T;




 































































































































































































































   
   
   


#pragma no_anon_unions


#line 240 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\sdh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    





















































































































































































































































































































 

    volatile uint32_t FB[32];                 
    
    volatile const  uint32_t RESERVE0[224];
    
    volatile uint32_t DMACTL;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t DMASA;                  
    volatile const  uint32_t DMABCNT;                
    volatile uint32_t DMAINTEN;               
    volatile uint32_t DMAINTSTS;              
    
    volatile const  uint32_t RESERVE2[250];
    
    volatile uint32_t GCTL;                   
    volatile uint32_t GINTEN;                 
    volatile const  uint32_t GINTSTS;                
    
    volatile const  uint32_t RESERVE3[5];
    
    volatile uint32_t CTL;                    
    volatile uint32_t CMDARG;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t RESP0;                  
    volatile const  uint32_t RESP1;                  
    volatile uint32_t BLEN;                   
    volatile uint32_t TOUT;                   

} SDH_T;





 


























































































































































   
   
   


#pragma no_anon_unions




#line 241 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\ebi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    









































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t TCTL0;                  
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t CTL1;                   
    volatile uint32_t TCTL1;                  
    
    volatile const  uint32_t RESERVE1[2];
    
    volatile uint32_t CTL2;                   
    volatile uint32_t TCTL2;                  

} EBI_T;




 









































































































































































   
   
   


#pragma no_anon_unions


#line 242 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\usbd_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    































































 
    volatile uint32_t BUFSEG;                
    volatile uint32_t MXPLD;                 
    volatile uint32_t CFG;                   
    volatile uint32_t CFGP;                  

} USBD_EP_T;

typedef struct
{


    













































































































































































































































































































 

    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t FADDR;                  
    volatile const  uint32_t EPSTS;                  
    volatile uint32_t ATTR;                   
    volatile const  uint32_t VBUSDET;                
    volatile uint32_t STBUFSEG;               
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile const  uint32_t EPSTS0;                 
    volatile const  uint32_t EPSTS1;                 
    
    volatile const  uint32_t RESERVE1[24];
    
    volatile const  uint32_t LPMATTR;                
    volatile const  uint32_t FN;                     
    volatile uint32_t SE0;                    
    
    volatile const  uint32_t RESERVE2[283];
    
    USBD_EP_T     EP[12];                 

} USBD_T;





 



























































































































































































   
   
   


#pragma no_anon_unions


#line 243 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\hsusbd_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    























































































































































































































































 

    union
    {
        volatile uint32_t EPDAT;
        volatile uint8_t  EPDAT_BYTE;

    };                                   

    volatile uint32_t EPINTSTS;              
    volatile uint32_t EPINTEN;               
    volatile const  uint32_t EPDATCNT;              
    volatile uint32_t EPRSPCTL;              
    volatile uint32_t EPMPS;                 
    volatile uint32_t EPTXCNT;               
    volatile uint32_t EPCFG;                 
    volatile uint32_t EPBUFST;               
    volatile uint32_t EPBUFEND;              

} HSUSBD_EP_T;

typedef struct
{

    




















































































































































































































































































































































































































































































































































































 

    volatile const  uint32_t GINTSTS;                
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t GINTEN;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t BUSINTSTS;              
    volatile uint32_t BUSINTEN;               
    volatile uint32_t OPER;                   
    volatile const  uint32_t FRAMECNT;               
    volatile uint32_t FADDR;                  
    volatile uint32_t TEST;                   

    union
    {
        volatile uint32_t CEPDAT;
        volatile uint8_t  CEPDAT_BYTE;

    };                                    

    volatile uint32_t CEPCTL;                 
    volatile uint32_t CEPINTEN;               
    volatile uint32_t CEPINTSTS;              
    volatile uint32_t CEPTXCNT;               
    volatile const  uint32_t CEPRXCNT;               
    volatile const  uint32_t CEPDATCNT;              
    volatile const  uint32_t SETUP1_0;               
    volatile const  uint32_t SETUP3_2;               
    volatile const  uint32_t SETUP5_4;               
    volatile const  uint32_t SETUP7_6;               
    volatile uint32_t CEPBUFST;               
    volatile uint32_t CEPBUFEND;              
    volatile uint32_t DMACTL;                 
    volatile uint32_t DMACNT;                 

    HSUSBD_EP_T EP[12];

    
    volatile const  uint32_t RESERVE2[303];
    
    volatile uint32_t DMAADDR;                
    volatile uint32_t PHYCTL;                 

} HSUSBD_T;




 






































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 244 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\usbh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    
























































































































































































































































































































































































































































































































 
    volatile const  uint32_t HcRevision;             
    volatile uint32_t HcControl;              
    volatile uint32_t HcCommandStatus;        
    volatile uint32_t HcInterruptStatus;      
    volatile uint32_t HcInterruptEnable;      
    volatile uint32_t HcInterruptDisable;     
    volatile uint32_t HcHCCA;                 
    volatile uint32_t HcPeriodCurrentED;      
    volatile uint32_t HcControlHeadED;        
    volatile uint32_t HcControlCurrentED;     
    volatile uint32_t HcBulkHeadED;           
    volatile uint32_t HcBulkCurrentED;        
    volatile uint32_t HcDoneHead;             
    volatile uint32_t HcFmInterval;           
    volatile const  uint32_t HcFmRemaining;          
    volatile const  uint32_t HcFmNumber;             
    volatile uint32_t HcPeriodicStart;        
    volatile uint32_t HcLSThreshold;          
    volatile uint32_t HcRhDescriptorA;        
    volatile uint32_t HcRhDescriptorB;        
    volatile uint32_t HcRhStatus;             
    volatile uint32_t HcRhPortStatus[2];      
    
    volatile const  uint32_t RESERVE0[105];
    
    volatile uint32_t HcPhyControl;           
    volatile uint32_t HcMiscControl;          

} USBH_T;




 




























































































































































































































   
   
   


#pragma no_anon_unions


#line 245 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\hsusbh_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































 
    volatile const  uint32_t EHCVNR;                 
    volatile const  uint32_t EHCSPR;                 
    volatile const  uint32_t EHCCPR;                 
    
    volatile const  uint32_t RESERVE0[5];
    
    volatile uint32_t UCMDR;                  
    volatile uint32_t USTSR;                  
    volatile uint32_t UIENR;                  
    volatile uint32_t UFINDR;                 
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t UPFLBAR;                
    volatile uint32_t UCALAR;                 
    volatile uint32_t UASSTR;                 
    
    volatile const  uint32_t RESERVE2[8];
    
    volatile uint32_t UCFGR;                  
    volatile uint32_t UPSCR[2];               
    
    volatile const  uint32_t RESERVE3[22];
    
    volatile uint32_t USBPCR0;                
    volatile uint32_t USBPCR1;                

} HSUSBH_T;




 






































































































































































   
   
   


#pragma no_anon_unions


#line 246 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\otg_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
























































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t PHYCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t STATUS;                 

} OTG_T;





 


































































































































   
   
   


#pragma no_anon_unions


#line 247 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\hsotg_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
























































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t PHYCTL;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t STATUS;                 

} HSOTG_T;




 


































































































































   
   
   


#pragma no_anon_unions


#line 248 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\crc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































 
    volatile uint32_t CTL;                    
    volatile uint32_t DAT;                    
    volatile uint32_t SEED;                   
    volatile const  uint32_t CHECKSUM;               

} CRC_T;




 


































   
   
   


#pragma no_anon_unions


#line 249 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\crypto_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    










































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t PRNG_CTL;               
    volatile  uint32_t PRNG_SEED;              
    volatile const  uint32_t PRNG_KEY[8];            
    
    volatile const  uint32_t RESERVE0[8];
    
    volatile const  uint32_t AES_FDBCK[4];           
    volatile const  uint32_t TDES_FDBCKH;            
    volatile const  uint32_t TDES_FDBCKL;            
    
    volatile const  uint32_t RESERVE1[38];
    
    volatile uint32_t AES_CTL;                
    volatile const  uint32_t AES_STS;                
    volatile uint32_t AES_DATIN;              
    volatile const  uint32_t AES_DATOUT;             
    volatile uint32_t AES0_KEY[8];            
    volatile uint32_t AES0_IV[4];             
    volatile uint32_t AES0_SADDR;             
    volatile uint32_t AES0_DADDR;             
    volatile uint32_t AES0_CNT;               
    volatile uint32_t AES1_KEY[8];            
    volatile uint32_t AES1_IV[4];             
    volatile uint32_t AES1_SADDR;             
    volatile uint32_t AES1_DADDR;             
    volatile uint32_t AES1_CNT;               
    volatile uint32_t AES2_KEY[8];            
    volatile uint32_t AES2_IV[4];             
    volatile uint32_t AES2_SADDR;             
    volatile uint32_t AES2_DADDR;             
    volatile uint32_t AES2_CNT;               
    volatile uint32_t AES3_KEY[8];            
    volatile uint32_t AES3_IV[4];             
    volatile uint32_t AES3_SADDR;             
    volatile uint32_t AES3_DADDR;             
    volatile uint32_t AES3_CNT;               
    volatile uint32_t TDES_CTL;               
    volatile const  uint32_t TDES_STS;               
    volatile uint32_t TDES0_KEY1H;            
    volatile uint32_t TDES0_KEY1L;            
    volatile uint32_t TDES0_KEY2H;            
    volatile uint32_t TDES0_KEY2L;            
    volatile uint32_t TDES0_KEY3H;            
    volatile uint32_t TDES0_KEY3L;            
    volatile uint32_t TDES0_IVH;              
    volatile uint32_t TDES0_IVL;              
    volatile uint32_t TDES0_SA;               
    volatile uint32_t TDES0_DA;               
    volatile uint32_t TDES0_CNT;              
    volatile uint32_t TDES_DATIN;             
    volatile const  uint32_t TDES_DATOUT;            
    
    volatile const  uint32_t RESERVE2[3];
    
    volatile uint32_t TDES1_KEY1H;            
    volatile uint32_t TDES1_KEY1L;            
    volatile uint32_t TDES1_KEY2H;            
    volatile uint32_t TDES1_KEY2L;            
    volatile uint32_t TDES1_KEY3H;            
    volatile uint32_t TDES1_KEY3L;            
    volatile uint32_t TDES1_IVH;              
    volatile uint32_t TDES1_IVL;              
    volatile uint32_t TDES1_SA;               
    volatile uint32_t TDES1_DA;               
    volatile uint32_t TDES1_CNT;              
    
    volatile const  uint32_t RESERVE3[5];
    
    volatile uint32_t TDES2_KEY1H;            
    volatile uint32_t TDES2_KEY1L;            
    volatile uint32_t TDES2_KEY2H;            
    volatile uint32_t TDES2_KEY2L;            
    volatile uint32_t TDES2_KEY3H;            
    volatile uint32_t TDES2_KEY3L;            
    volatile uint32_t TDES2_IVH;              
    volatile uint32_t TDES2_IVL;              
    volatile uint32_t TDES2_SA;               
    volatile uint32_t TDES2_DA;               
    volatile uint32_t TDES2_CNT;              
    
    volatile const  uint32_t RESERVE4[5];
    
    volatile uint32_t TDES3_KEY1H;            
    volatile uint32_t TDES3_KEY1L;            
    volatile uint32_t TDES3_KEY2H;            
    volatile uint32_t TDES3_KEY2L;            
    volatile uint32_t TDES3_KEY3H;            
    volatile uint32_t TDES3_KEY3L;            
    volatile uint32_t TDES3_IVH;              
    volatile uint32_t TDES3_IVL;              
    volatile uint32_t TDES3_SA;               
    volatile uint32_t TDES3_DA;               
    volatile uint32_t TDES3_CNT;              
    
    volatile const  uint32_t RESERVE5[3];
    
    volatile uint32_t HMAC_CTL;               
    volatile const  uint32_t HMAC_STS;               
    volatile const  uint32_t HMAC_DGST[16];          
    volatile uint32_t HMAC_KEYCNT;            
    volatile uint32_t HMAC_SADDR;             
    volatile uint32_t HMAC_DMACNT;            
    volatile uint32_t HMAC_DATIN;             
    
    volatile const  uint32_t RESERVE6[298];
    
    volatile uint32_t ECC_CTL;                
    volatile const  uint32_t ECC_STS;                
    volatile uint32_t ECC_X1[18];             
    volatile uint32_t ECC_Y1[18];             
    volatile uint32_t ECC_X2[18];             
    volatile uint32_t ECC_Y2[18];             
    volatile uint32_t ECC_A[18];              
    volatile uint32_t ECC_B[18];              
    volatile uint32_t ECC_N[18];              
    volatile  uint32_t ECC_K[18];              
    volatile uint32_t ECC_SADDR;              
    volatile uint32_t ECC_DADDR;              
    volatile uint32_t ECC_STARTREG;           
    volatile uint32_t ECC_WORDCNT;            

} CRPT_T;




 

























































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 250 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\trng_reg.h"
 





 







 

 



 

typedef struct
{


    






























































 
    volatile uint32_t CTL;                    
    volatile const  uint32_t DATA;                   
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t ACT;                    

} TRNG_T;




 




























   
   
   


#line 251 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\eadc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
































































































































































































































































































































































































































































































































 
    volatile const  uint32_t DAT[19];                
    volatile const  uint32_t CURDAT;                 
    volatile uint32_t CTL;                    
    volatile  uint32_t SWTRG;                  
    volatile uint32_t PENDSTS;                
    volatile uint32_t OVSTS;                  
    
    volatile const  uint32_t RESERVE0[8];
    
    volatile uint32_t SCTL[19];               
    
    volatile const  uint32_t RESERVE1[1];
    
    volatile uint32_t INTSRC[4];              
    volatile uint32_t CMP[4];                 
    volatile const  uint32_t STATUS0;                
    volatile const  uint32_t STATUS1;                
    volatile uint32_t STATUS2;                
    volatile const  uint32_t STATUS3;                
    volatile const  uint32_t DDAT[4];                
    volatile uint32_t PWRM;                   
    volatile uint32_t CALCTL;                 
    volatile uint32_t CALDWRD;                
    
    volatile const  uint32_t RESERVE2[5];
    
    volatile uint32_t PDMACTL;                
} EADC_T;




 









































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 252 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\dac_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t SWTRG;                  
    volatile uint32_t DAT;                    
    volatile const  uint32_t DATOUT;                 
    volatile uint32_t STATUS;                 
    volatile uint32_t TCTL;                   

} DAC_T;




 























































   
   
   


#pragma no_anon_unions


#line 253 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\acmp_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





















































































































 
    volatile uint32_t CTL[2];                 
    volatile uint32_t STATUS;                 
    volatile uint32_t VREF;                   

} ACMP_T;




 









































































   
   
   


#pragma no_anon_unions


#line 254 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\opa_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    




















































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t STATUS;                 
    volatile uint32_t CALCTL;                 
    volatile const  uint32_t CALST;                  

} OPA_T;




 




































































































   
   
   


#pragma no_anon_unions




#line 255 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\Device\\Nuvoton\\M480\\Include\\ccap_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct {


    













































































































































































































































































 
    volatile uint32_t CTL;
    volatile uint32_t PAR;
    volatile uint32_t INT;
    volatile uint32_t POSTERIZE;
    volatile uint32_t MD;
    volatile uint32_t MDADDR;
    volatile uint32_t MDYADDR;
    volatile uint32_t SEPIA;
    volatile uint32_t CWSP;
    volatile uint32_t CWS;
    volatile uint32_t PKTSL;
    volatile uint32_t PLNSL;
    volatile uint32_t FRCTL;
    volatile uint32_t STRIDE;
    
    uint32_t RESERVE0[1];
    
    volatile uint32_t FIFOTH;
    volatile uint32_t CMPADDR;
    volatile uint32_t LUMA_Y1_THD;
    volatile uint32_t PKTSM;
    
    uint32_t RESERVE2[5];
    
    volatile uint32_t PKTBA0;
} CCAP_T;




 




























































































































































   
   
   


#pragma no_anon_unions


#line 256 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"





 
 






 
#line 296 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

 
#line 321 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"


 
#line 349 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 357 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   





 

#line 396 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 404 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 458 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

   




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 



   

 
 
 



 
















 
#line 650 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

 










   


 
 
 
#line 1 "..\\Library\\StdDriver\\inc\\sys.h"
 





 












 



 



 


 
 
 
#line 45 "..\\Library\\StdDriver\\inc\\sys.h"

#line 75 "..\\Library\\StdDriver\\inc\\sys.h"

#line 95 "..\\Library\\StdDriver\\inc\\sys.h"

 
 
 
#line 109 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 120 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 132 "..\\Library\\StdDriver\\inc\\sys.h"

 
 
 




 
 
 






 
 
#line 258 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 358 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 483 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 604 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 712 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 779 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 835 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 887 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 973 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1049 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1108 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1141 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1191 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1246 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1286 "..\\Library\\StdDriver\\inc\\sys.h"
 
#line 1321 "..\\Library\\StdDriver\\inc\\sys.h"

   




 








 









 









 









 









 










 









 









 









 

















 









 









 









 









 









 









 









 









 









 









 









 
















 



 
 
 
 
static __inline void SYS_UnlockReg(void);
static __inline void SYS_LockReg(void);







 
static __inline void SYS_UnlockReg(void)
{
    do
    {
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x59UL;
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x16UL;
        ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0x88UL;
    }
    while(((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL == 0UL);
}







 
static __inline void SYS_LockReg(void)
{
    ((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->REGLCTL = 0UL;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_SetPowerLevel(uint32_t u32PowerLevel);
void SYS_SetVRef(uint32_t u32VRefCTL);

   

   

   








 
#line 669 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\clk.h"
 





 











 



 



 


#line 41 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






#line 57 "..\\Library\\StdDriver\\inc\\clk.h"







#line 71 "..\\Library\\StdDriver\\inc\\clk.h"

















 
 
 




#line 101 "..\\Library\\StdDriver\\inc\\clk.h"

#line 108 "..\\Library\\StdDriver\\inc\\clk.h"

#line 115 "..\\Library\\StdDriver\\inc\\clk.h"

#line 122 "..\\Library\\StdDriver\\inc\\clk.h"




















 
 
 






































 
 
 


























































 
 
 
#line 253 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 




 
 
 



 
 
 





 
 
 
#line 284 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 298 "..\\Library\\StdDriver\\inc\\clk.h"

#line 309 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






















 
 
 

 

#line 351 "..\\Library\\StdDriver\\inc\\clk.h"

#line 360 "..\\Library\\StdDriver\\inc\\clk.h"

#line 426 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 436 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 452 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 472 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






























 
 
 












#line 533 "..\\Library\\StdDriver\\inc\\clk.h"

   



 























 




























 


 
 
 
 
static __inline void CLK_SysTickDelay(uint32_t us);
static __inline void CLK_SysTickLongDelay(uint32_t us);









 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = 0x0UL;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL)
    {
    }

     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
}








 
static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;

     
    delay = 349525UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }

        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;

    }
    while(us > 0UL);

}


void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
void CLK_SetPowerDownMode(uint32_t u32PDMode);
void CLK_EnableDPDWKPin(uint32_t u32TriggerType);
uint32_t CLK_GetPMUWKSrc(void);
void CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx);
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx);

   

   

   







 
#line 670 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"

#line 1 "..\\Library\\StdDriver\\inc\\acmp.h"
 





 












 




 




 



 
 
 
#line 60 "..\\Library\\StdDriver\\inc\\acmp.h"

 
 
 




   




 

 
 
 









 









 














 








 









 













 










 









 









 









 









 









 









 









 









 









 














 









 









 


















 












 











 













 












 









 
















 









 





 
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysSel);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);



   

   

   








 
#line 672 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\dac.h"
 





 











 



 




 

 
 
 



#line 48 "..\\Library\\StdDriver\\inc\\dac.h"





   




 







 








 








 









 








 









 









 








 








 








 








 











 









 










 










 









 









 








 








 







 


void DAC_Open(DAC_T *dac, uint32_t u32Ch, uint32_t u32TrgSrc);
void DAC_Close(DAC_T *dac, uint32_t u32Ch);
uint32_t DAC_SetDelayTime(DAC_T *dac, uint32_t u32Delay);

   

   

   







 
#line 673 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\emac.h"
 





 











 



 



 











   




 







 








 







 








 







 







 







 







 







 







 









 







 






 






 


void EMAC_Open(uint8_t *pu8MacAddr);
void EMAC_Close(void);
void EMAC_SetMacAddr(uint8_t *pu8MacAddr);
void EMAC_EnableCamEntry(uint32_t u32Entry, uint8_t pu8MacAddr[]);
void EMAC_DisableCamEntry(uint32_t u32Entry);

uint32_t EMAC_RecvPkt(uint8_t *pu8Data, uint32_t *pu32Size);
uint32_t EMAC_RecvPktTS(uint8_t *pu8Data, uint32_t *pu32Size, uint32_t *pu32Sec, uint32_t *pu32Nsec);
void EMAC_RecvPktDone(void);

uint32_t EMAC_SendPkt(uint8_t *pu8Data, uint32_t u32Size);
uint32_t EMAC_SendPktDone(void);
uint32_t EMAC_SendPktDoneTS(uint32_t *pu32Sec, uint32_t *pu32Nsec);

void EMAC_EnableTS(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_DisableTS(void);
void EMAC_GetTime(uint32_t *pu32Sec, uint32_t *pu32Nsec);
void EMAC_SetTime(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_UpdateTime(uint32_t u32Neg, uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_EnableAlarm(uint32_t u32Sec, uint32_t u32Nsec);
void EMAC_DisableAlarm(void);

uint32_t EMAC_CheckLinkStatus(void);

   

   

   







 
#line 674 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\uart.h"
 





 












 



 



 

 
 
 

#line 40 "..\\Library\\StdDriver\\inc\\uart.h"

 
 
 











 
 
 
















 
 
 




 
 
 




 
 
 






 
 
 
#line 106 "..\\Library\\StdDriver\\inc\\uart.h"


 
 
 




   




 












 













 













 












 













 













 














 












 













 













 













 













 













 






















 






















 




































 












 













 


 
static __inline void UART_CLEAR_RTS(UART_T* uart);
static __inline void UART_SET_RTS(UART_T* uart);










 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= ~(0x1ul << (1));
}










 
static __inline void UART_SET_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}


void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void UART_SetLineConfig(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);




   

   

   







 
#line 675 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\usci_spi.h"
 





 











 



 



 













 
#line 52 "..\\Library\\StdDriver\\inc\\usci_spi.h"

 
#line 60 "..\\Library\\StdDriver\\inc\\usci_spi.h"

   




 






 







 









 









 









 







 








 








 












 












 







 







 








 
#line 197 "..\\Library\\StdDriver\\inc\\usci_spi.h"








 









 







 







 
















 







 










 












 












 










 










 












 












 









 








 








 








 


uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);


   

   

   







 
#line 676 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\gpio.h"
 





 











 



 



 





 
 
 






 
 
 







 
 
 



 
 
 




 
 
 





 
 
 






#line 98 "..\\Library\\StdDriver\\inc\\gpio.h"















 
#line 233 "..\\Library\\StdDriver\\inc\\gpio.h"


   




 














 















 














 















 















 















 















 
















 
































 











 












 











 


















 















 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);


   

   

   








 
#line 677 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\ccap.h"
 





 










 



 



 

 
 
 
#line 40 "..\\Library\\StdDriver\\inc\\ccap.h"

 
 
 
























#line 74 "..\\Library\\StdDriver\\inc\\ccap.h"

 
 
 






static uint32_t u32EscapeFrame = 0;
 
 
 





   





 








 















 









 


void CCAP_Open(uint32_t u32InFormat, uint32_t u32OutFormet);
void CCAP_SetCroppingWindow(uint32_t u32VStart,uint32_t u32HStart, uint32_t u32Height, uint32_t u32Width);
void CCAP_SetPacketBuf(uint32_t  u32Address );
void CCAP_Close(void);
void CCAP_EnableInt(uint32_t u32IntMask);
void CCAP_DisableInt(uint32_t u32IntMask);
void CCAP_Start(void);
void CCAP_Stop(uint32_t u32FrameComplete);
void CCAP_SetPacketScaling(uint32_t u32VNumerator, uint32_t u32VDenominator, uint32_t u32HNumerator, uint32_t u32HDenominator);
void CCAP_SetPacketStride(uint32_t u32Stride );
void CCAP_EnableMono(uint32_t u32Interface);
void CCAP_DisableMono(void);
void CCAP_EnableLumaYOne(uint32_t u32th);
void CCAP_DisableLumaYOne(void);

   



   

   







 
#line 678 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\ecap.h"
 





 











 



 



 





 
 
 
#line 42 "..\\Library\\StdDriver\\inc\\ecap.h"







 
 
 




#line 64 "..\\Library\\StdDriver\\inc\\ecap.h"






   



 














 








 















 












 












 















 












 












 








 








 








 








 








 








 















 
#line 260 "..\\Library\\StdDriver\\inc\\ecap.h"







 








 








 








 
















 













 

















 













 








 














 














 









 








 












 









 


void ECAP_Open(ECAP_T* ecap, uint32_t u32FuncMask);
void ECAP_Close(ECAP_T* ecap);
void ECAP_EnableINT(ECAP_T* ecap, uint32_t u32Mask);
void ECAP_DisableINT(ECAP_T* ecap, uint32_t u32Mask);
   

   

   







 
#line 679 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\qei.h"
 





 











 



 



 

 
 
 





 
 
 
#line 46 "..\\Library\\StdDriver\\inc\\qei.h"




   




 







 








 








 








 








 








 












 












 












 












 













 













 








 















 








 









 








 








 









 















 














 









 









 














 














 









 













 



void QEI_Close(QEI_T* qei);
void QEI_DisableInt(QEI_T* qei, uint32_t u32IntSel);
void QEI_EnableInt(QEI_T* qei, uint32_t u32IntSel);
void QEI_Open(QEI_T* qei, uint32_t u32Mode, uint32_t u32Value);
void QEI_Start(QEI_T* qei);
void QEI_Stop(QEI_T* qei);


   

   

   







 
#line 680 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\timer.h"
 





 











 



 



 
 
 
 
#line 37 "..\\Library\\StdDriver\\inc\\timer.h"






#line 49 "..\\Library\\StdDriver\\inc\\timer.h"

#line 56 "..\\Library\\StdDriver\\inc\\timer.h"

   




 














 













 












 














 


 
static __inline void TIMER_Start(TIMER_T *timer);
static __inline void TIMER_Stop(TIMER_T *timer);
static __inline void TIMER_EnableWakeup(TIMER_T *timer);
static __inline void TIMER_DisableWakeup(TIMER_T *timer);
static __inline void TIMER_StartCapture(TIMER_T *timer);
static __inline void TIMER_StopCapture(TIMER_T *timer);
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer);
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer);
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer);
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer);
static __inline void TIMER_EnableInt(TIMER_T *timer);
static __inline void TIMER_DisableInt(TIMER_T *timer);
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer);
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer);
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer);
static __inline void TIMER_ClearIntFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer);
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer);
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer);
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer);
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer);
static __inline void TIMER_ResetCounter(TIMER_T *timer);









 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}











 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}









 
static __inline void TIMER_StartCapture(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (3));
}









 
static __inline void TIMER_StopCapture(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (3));
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (6));
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (6));
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return ((timer->INTSTS & (0x1ul << (0))) ? 1UL : 0UL);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1UL : 0UL);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}









 
static __inline void TIMER_ResetCounter(TIMER_T *timer)
{
    timer->CNT = 0UL;
    while((timer->CNT&(0x1ul << (31))) == (0x1ul << (31)))
    {
        ;
    }
}


uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);

   

   

   








#line 681 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\timer_pwm.h"
 





 












 


 



 
 
 
 



 
 
 




 
 
 



 
 
 





 
 
 






 
 
 
#line 74 "..\\Library\\StdDriver\\inc\\timer_pwm.h"


#line 83 "..\\Library\\StdDriver\\inc\\timer_pwm.h"




 
 
 




 
 
 
#line 105 "..\\Library\\StdDriver\\inc\\timer_pwm.h"

 
 
 





 
 
 






 
 
 






   




 











 












 











 











 















 











 











 














 











 













 











 













 











 











 















 
















 
















 


















 




















 













 











 











 












 











 











 











 












 











 











 











 












 











 











 











 












 











 












 











 












 











 












 











 
















 



void TPWM_SetCounterClockSource(TIMER_T *timer, uint32_t u32CntClkSrc);
uint32_t TPWM_ConfigOutputFreqAndDuty(TIMER_T *timer, uint32_t u32Frequency, uint32_t u32DutyCycle);
void TPWM_EnableDeadTime(TIMER_T *timer, uint32_t u32DTCount);
void TPWM_EnableDeadTimeWithPrescale(TIMER_T *timer, uint32_t u32DTCount);
void TPWM_DisableDeadTime(TIMER_T *timer);
void TPWM_EnableCounter(TIMER_T *timer);
void TPWM_DisableCounter(TIMER_T *timer);
void TPWM_EnableTriggerADC(TIMER_T *timer, uint32_t u32Condition);
void TPWM_DisableTriggerADC(TIMER_T *timer);
void TPWM_EnableFaultBrake(TIMER_T *timer, uint32_t u32CH0Level, uint32_t u32CH1Level, uint32_t u32BrakeSource);
void TPWM_EnableFaultBrakeInt(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_DisableFaultBrakeInt(TIMER_T *timer, uint32_t u32IntSource);
uint32_t TPWM_GetFaultBrakeIntFlag(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_ClearFaultBrakeIntFlag(TIMER_T *timer, uint32_t u32IntSource);
void TPWM_SetLoadMode(TIMER_T *timer, uint32_t u32LoadMode);
void TPWM_EnableBrakePinDebounce(TIMER_T *timer, uint32_t u32BrakePinSrc, uint32_t u32DebounceCnt, uint32_t u32ClkSrcSel);
void TPWM_DisableBrakePinDebounce(TIMER_T *timer);
void TPWM_EnableBrakePinInverse(TIMER_T *timer);
void TPWM_DisableBrakePinInverse(TIMER_T *timer);
void TPWM_SetBrakePinSource(TIMER_T *timer, uint32_t u32BrakePinNum);

   

   

   







#line 682 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\pdma.h"
 





 











 



 



 


 
 
 




 
 
 




 
 
 





 
 
 



#line 66 "..\\Library\\StdDriver\\inc\\pdma.h"

 
 
 



 
 
 
#line 145 "..\\Library\\StdDriver\\inc\\pdma.h"
 
 
 





   



 










 











 













 











 













 











 












 












 













 













 













 













 













 













 













 


 
 
 
void PDMA_Open(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_Close(PDMA_T * pdma);
void PDMA_SetTransferCnt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_DisableTimeout(PDMA_T * pdma,uint32_t u32Mask);
void PDMA_SetTimeOut(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(PDMA_T * pdma,uint32_t u32Ch);
void PDMA_EnableInt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32Mask);
void PDMA_SetStride(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32DestLen, uint32_t u32SrcLen, uint32_t u32TransCount);
void PDMA_SetRepeat(PDMA_T * pdma,uint32_t u32Ch, uint32_t u32DestInterval, uint32_t u32SrcInterval, uint32_t u32RepeatCount);


   

   

   







 
#line 683 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\crypto.h"
 





 










 



 




 













#line 49 "..\\Library\\StdDriver\\inc\\crypto.h"






#line 65 "..\\Library\\StdDriver\\inc\\crypto.h"

#line 74 "..\\Library\\StdDriver\\inc\\crypto.h"

















typedef enum
{
     
    CURVE_P_192,                         
    CURVE_P_224,                         
    CURVE_P_256,                         
    CURVE_P_384,                         
    CURVE_P_521,                         
    CURVE_K_163,                         
    CURVE_K_233,                         
    CURVE_K_283,                         
    CURVE_K_409,                         
    CURVE_K_571,                         
    CURVE_B_163,                         
    CURVE_B_233,                         
    CURVE_B_283,                         
    CURVE_B_409,                         
    CURVE_B_571,                         
    CURVE_KO_192,                        
    CURVE_KO_224,                        
    CURVE_KO_256,                        
    CURVE_BP_256,                        
    CURVE_BP_384,                        
    CURVE_BP_512,                        
    CURVE_UNDEF,                         
}
E_ECC_CURVE;                             


   




 

 
 
 






 







 







 







 







 







 







 







 







 







 








 







 







 







 







 







 








 







 







 







 







 







 







 







 



   




 


 
 
 

void PRNG_Open(CRPT_T *crpt, uint32_t u32KeySize, uint32_t u32SeedReload, uint32_t u32Seed);
void PRNG_Start(CRPT_T *crpt);
void PRNG_Read(CRPT_T *crpt, uint32_t u32RandKey[]);
void AES_Open(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32EncDec, uint32_t u32OpMode, uint32_t u32KeySize, uint32_t u32SwapType);
void AES_Start(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32DMAMode);
void AES_SetKey(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32Keys[], uint32_t u32KeySize);
void AES_SetInitVect(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32IV[]);
void AES_SetDMATransfer(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32TransCnt);
void TDES_Open(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32EncDec, int32_t Is3DES, int32_t Is3Key, uint32_t u32OpMode, uint32_t u32SwapType);
void TDES_Start(CRPT_T *crpt, int32_t u32Channel, uint32_t u32DMAMode);
void TDES_SetKey(CRPT_T *crpt, uint32_t u32Channel, uint32_t au32Keys[3][2]);
void TDES_SetInitVect(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32IVH, uint32_t u32IVL);
void TDES_SetDMATransfer(CRPT_T *crpt, uint32_t u32Channel, uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32TransCnt);
void SHA_Open(CRPT_T *crpt, uint32_t u32OpMode, uint32_t u32SwapType, uint32_t hmac_key_len);
void SHA_Start(CRPT_T *crpt, uint32_t u32DMAMode);
void SHA_SetDMATransfer(CRPT_T *crpt, uint32_t u32SrcAddr, uint32_t u32TransCnt);
void SHA_Read(CRPT_T *crpt, uint32_t u32Digest[]);
void ECC_Complete(CRPT_T *crpt);
int  ECC_IsPrivateKeyValid(CRPT_T *crpt, E_ECC_CURVE ecc_curve,  char private_k[]);
int32_t  ECC_GeneratePublicKey(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[]);
int32_t  ECC_Mutiply(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char x1[], char y1[], char *k, char x2[], char y2[]);
int32_t  ECC_GenerateSecretZ(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[], char secret_z[]);
int32_t  ECC_GenerateSignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message, char *d, char *k, char *R, char *S);
int32_t  ECC_VerifySignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message, char *public_k1, char *public_k2, char *R, char *S);


   

   

   







 

#line 684 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\trng.h"
 





 










 



 




 

 
 
 


















 



   




 


 
 
 

void TRNG_Open(void);
int32_t TRNG_GenWord(uint32_t *u32RndNum);
int32_t TRNG_GenBignum(uint8_t u8BigNum[], int32_t i32Len);
int32_t TRNG_GenBignumHex(char cBigNumHex[], int32_t i32Len);


   

   

   







 

#line 685 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\fmc.h"
 





 










 



 




 


 
 
 
#line 51 "..\\Library\\StdDriver\\inc\\fmc.h"











 
 
 





 
 
 



 
 
 
#line 95 "..\\Library\\StdDriver\\inc\\fmc.h"






   




 


 
 
 

#line 127 "..\\Library\\StdDriver\\inc\\fmc.h"

   




 

 
 
 

static __inline uint32_t FMC_ReadCID(void);
static __inline uint32_t FMC_ReadPID(void);
static __inline uint32_t FMC_ReadUID(uint8_t u8Index);
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index);
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
static __inline uint32_t FMC_GetVECMAP(void);








 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPSTS & (0x7ffful << (9)));
}






 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x0BUL;            
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = 0x0u;                          
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));           



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x0CUL;           
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = 0x04u;                        
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));          



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUID(uint8_t u8Index)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x04UL;
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT = 0u;
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = 0x1u;



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG) {}

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x04UL;             
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = (0x04u * u32Index) + 0x10u;     
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = (0x1ul << (0));            



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG & (0x1ul << (0))) {}   

    return ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPDAT;
}








 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPCMD = 0x2EUL;   
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPADDR = u32PageAddr;        
    ((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG = 0x1u;                



    while(((FMC_T *) (((uint32_t)0x40000000) + 0x0C000UL))->ISPTRG) {}              
}


 
 
 

extern void     FMC_Close(void);
extern int32_t  FMC_ConfigXOM(uint32_t xom_num, uint32_t xom_base, uint8_t xom_page);
extern int32_t  FMC_Erase(uint32_t u32PageAddr);
extern int32_t  FMC_Erase_SPROM(void);
extern int32_t  FMC_Erase_Block(uint32_t u32BlockAddr);
extern int32_t  FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t  FMC_EraseXOM(uint32_t xom_num);
extern int32_t  FMC_GetXOMState(uint32_t xom_num);
extern int32_t  FMC_GetBootSource(void);
extern void     FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern int32_t  FMC_Read_64(uint32_t u32addr, uint32_t * u32data0, uint32_t * u32data1);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void     FMC_SetBootSource(int32_t i32BootSrc);
extern void     FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t  FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t  FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len);
extern int32_t  FMC_Write_OTP(uint32_t otp_num, uint32_t low_word, uint32_t high_word);
extern int32_t  FMC_Read_OTP(uint32_t otp_num, uint32_t *low_word, uint32_t *high_word);
extern int32_t  FMC_Lock_OTP(uint32_t otp_num);
extern int32_t  FMC_Is_OTP_Locked(uint32_t otp_num);
extern int32_t  FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t  FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t  FMC_SetSPKey(uint32_t key[3], uint32_t kpmax, uint32_t kemax, const int32_t lock_CONFIG, const int32_t lock_SPROM);
extern int32_t  FMC_CompareSPKey(uint32_t key[3]);


   

   

   







 
#line 686 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\spim.h"
 





 



 
 
 









 



 




 






 
 
 
#line 50 "..\\Library\\StdDriver\\inc\\spim.h"







#line 65 "..\\Library\\StdDriver\\inc\\spim.h"

 

typedef enum
{
    MFGID_UNKNOW    = 0x00U,
    MFGID_SPANSION  = 0x01U,
    MFGID_EON       = 0x1CU,
    MFGID_ISSI      = 0x7FU,
    MFGID_MXIC      = 0xC2U,
    MFGID_WINBOND   = 0xEFU
}
E_MFGID;

 
#line 100 "..\\Library\\StdDriver\\inc\\spim.h"

 




 












 



 

 








 

   




 


 
 
 




 





 





 





 





 








 





 





 





 








 








 






 








 






 








 








 








 








 








 








 








 






 








 





 








 





 





 





 








 





 





 





 





 





 





 





 








 






 








 








 






 








 






 








 






 








 






 






 






 








 






 








 






 





 





 





 





 








 





 








 




 
 
 


int  SPIM_InitFlash(int clrWP);
uint32_t SPIM_GetSClkFreq(void);
void SPIM_ReadJedecId(uint8_t idBuf[], uint32_t u32NRx, uint32_t u32NBit);
int  SPIM_Enable_4Bytes_Mode(int isEn, uint32_t u32NBit);
int  SPIM_Is4ByteModeEnable(uint32_t u32NBit);

void SPIM_ChipErase(uint32_t u32NBit, int isSync);
void SPIM_EraseBlock(uint32_t u32Addr, int is4ByteAddr, uint8_t u8ErsCmd, uint32_t u32NBit, int isSync);

void SPIM_IO_Write(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t wrCmd, uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat);
void SPIM_IO_Read(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint8_t rdCmd, uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, int u32NDummy);

void SPIM_DMA_Write(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint32_t wrCmd);
void SPIM_DMA_Read(uint32_t u32Addr, int is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint32_t u32RdCmd, int isSync);

void SPIM_EnterDirectMapMode(int is4ByteAddr, uint32_t u32RdCmd, uint32_t u32IdleIntvl);
void SPIM_ExitDirectMapMode(void);

void SPIM_SetQuadEnable(int isEn, uint32_t u32NBit);

   

   

   







 
#line 687 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2c.h"
 





 











 



 



 

 
 
 
#line 41 "..\\Library\\StdDriver\\inc\\i2c.h"

 
 
 



 
 
 





   



 










 











 











 











 












 











 












 












 











 












 











 












 












 












 












 












 











 












 











 











 











 











 











 











 








 








 








 








 








 








 








 


 
 
 

 
static __inline void I2C_STOP(I2C_T *i2c);









 
static __inline void I2C_STOP(I2C_T *i2c)
{

    (i2c)->CTL0 |= ((0x1ul << (3)) | (0x1ul << (4)));
    while(i2c->CTL0 & (0x1ul << (4)))
    {
    }
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SMBusClearInterruptFlag(I2C_T *i2c, uint8_t u8SMBusIntFlag);
uint8_t I2C_WriteByte(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t I2C_WriteMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_ReadByte(I2C_T *i2c, uint8_t u8SlaveAddr);
uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t I2C_ReadMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_SMBusGetStatus(I2C_T *i2c);
void I2C_SMBusSetPacketByteCount(I2C_T *i2c, uint32_t u32PktSize);
void I2C_SMBusOpen(I2C_T *i2c, uint8_t u8HostDevice);
void I2C_SMBusClose(I2C_T *i2c);
void I2C_SMBusPECTxEnable(I2C_T *i2c, uint8_t u8PECTxEn);
uint8_t I2C_SMBusGetPECValue(I2C_T *i2c);
void I2C_SMBusIdleTimeout(I2C_T *i2c, uint32_t us, uint32_t u32Hclk);
void I2C_SMBusTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);
void I2C_SMBusClockLoTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);

   

   

   







 
#line 688 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2s.h"
 





 










 



 



 





 



 
#line 43 "..\\Library\\StdDriver\\inc\\i2s.h"

 



 





 





 



 



 
#line 85 "..\\Library\\StdDriver\\inc\\i2s.h"

#line 102 "..\\Library\\StdDriver\\inc\\i2s.h"

 



 



   



 
 
 
 






 
static __inline void I2S_ENABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if((u32ChMask > 0U) && (u32ChMask < 9U))
    {
        i2s->CTL1 |= ((uint32_t)1U << (u32ChMask-1U));
    }
}







 
static __inline void I2S_DISABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if((u32ChMask > 0U) && (u32ChMask < 9U))
    {
        i2s->CTL1 &= ~((uint32_t)1U << (u32ChMask-1U));
    }
}






 







 







 







 







 







 







 







 







 







 







 







 










 
static __inline void I2S_SET_MONO_RX_CHANNEL(I2S_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->CTL0 |= (0x1ul << (23))) :
    (i2s->CTL0 &= ~(0x1ul << (23)));
}







 







 








 








 








 








 







 







 


void I2S_Close(I2S_T *i2s);
void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask);
void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);
void I2S_SetFIFO(I2S_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void I2S_ConfigureTDM(I2S_T *i2s, uint32_t u32ChannelWidth, uint32_t u32ChannelNum, uint32_t u32SyncWidth);
uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32MonoData, uint32_t u32DataFormat);

   


   

   






 

#line 689 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\epwm.h"
 





 











 



 



 
#line 35 "..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 




 
 
 



 
 
 





 
 
 





 
 
 
#line 84 "..\\Library\\StdDriver\\inc\\epwm.h"






 
 
 
#line 102 "..\\Library\\StdDriver\\inc\\epwm.h"

#line 112 "..\\Library\\StdDriver\\inc\\epwm.h"




 
 
 
#line 129 "..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 







 
 
 



 
 
 





 
 
 




 
 
 
#line 170 "..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 
#line 182 "..\\Library\\StdDriver\\inc\\epwm.h"

 
 
 







   




 







 








 








 








 















 










 
#line 269 "..\\Library\\StdDriver\\inc\\epwm.h"








 










 









 









 












 
















 











 











 









 











 












 









 













 
#line 430 "..\\Library\\StdDriver\\inc\\epwm.h"









 










 










 






























 
#line 505 "..\\Library\\StdDriver\\inc\\epwm.h"












 











 




 
 
 
uint32_t EPWM_ConfigCaptureChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableADCTrigger(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void EPWM_DisableADCTrigger(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearADCTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t EPWM_GetADCTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDACTrigger(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void EPWM_DisableDACTrigger(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDACTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t EPWM_GetDACTriggerFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableCapture(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableCapture(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnablePDMA(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode);
void EPWM_DisablePDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableCaptureInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void EPWM_DisableCaptureInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void EPWM_ClearCaptureIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t EPWM_GetCaptureIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAcc(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntFlagCnt, uint32_t u32IntAccSrc);
void EPWM_DisableAcc(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetAccInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccPDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccPDMA(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableAccStopMode(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableAccStopMode(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearFTDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetFTDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_DisableLoadMode(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void EPWM_ConfigSyncPhase(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32SyncSrc, uint32_t u32Direction, uint32_t u32StartPhase);
void EPWM_EnableSyncPhase(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableSyncPhase(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableSyncNoiseFilter(EPWM_T *epwm, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void EPWM_DisableSyncNoiseFilter(EPWM_T *epwm);
void EPWM_EnableSyncPinInverse(EPWM_T *epwm);
void EPWM_DisableSyncPinInverse(EPWM_T *epwm);
void EPWM_SetClockSource(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
void EPWM_EnableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void EPWM_DisableBrakeNoiseFilter(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_EnableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_DisableBrakePinInverse(EPWM_T *epwm, uint32_t u32BrakePinNum);
void EPWM_SetBrakePinSource(EPWM_T *epwm, uint32_t u32BrakePinNum, uint32_t u32SelAnotherModule);
void EPWM_SetLeadingEdgeBlanking(EPWM_T *epwm, uint32_t u32TrigSrcSel, uint32_t u32TrigType, uint32_t u32BlankingCnt, uint32_t u32BlankingEnable);
uint32_t EPWM_GetWrapAroundFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearWrapAroundFlag(EPWM_T *epwm, uint32_t u32ChannelNum);

   

   

   







 
#line 690 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\eadc.h"
 





 











 



 



 

 
 
 






 
 
 




#line 67 "..\\Library\\StdDriver\\inc\\eadc.h"







 
 
 
#line 83 "..\\Library\\StdDriver\\inc\\eadc.h"

   



 
 
 
 







 










 








 











 










 










 









 









 









 











 










 











 











 











 











 











 









 









 









 









 









 











 









 











 










 









 









 









 









 









 
















 
#line 438 "..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 463 "..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 488 "..\\Library\\StdDriver\\inc\\eadc.h"















 
#line 513 "..\\Library\\StdDriver\\inc\\eadc.h"








 









 











 









 








 








 








 








 


 
 
 
void EADC_Open(EADC_T *eadc, uint32_t u32InputMode);
void EADC_Close(EADC_T *eadc);
void EADC_ConfigSampleModule(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerSrc, uint32_t u32Channel);
void EADC_SetTriggerDelayTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerDelayTime, uint32_t u32DelayClockDivider);
void EADC_SetExtendSampleTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime);

   

   

   







 
#line 691 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\bpwm.h"
 





 











 



 



 
#line 35 "..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 




 
 
 



 
 
 





 
 
 





 
 
 
#line 75 "..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 



 
 
 



 
 
 



 
 
 






   




 














 










 









 









 








 








 












 













 










 








 











 








 












 









 






























 
#line 307 "..\\Library\\StdDriver\\inc\\bpwm.h"


 
 
 
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void BPWM_DisableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t BPWM_GetADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_DisableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_DisableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_SetClockSource(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
uint32_t BPWM_GetWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);


   

   

   







 
#line 692 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\wdt.h"
 





 











 



 



 
 
 
 
#line 39 "..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





 
 
 


   




 










 











 











 












 












 












 














 


 
static __inline void WDT_Close(void);
static __inline void WDT_EnableInt(void);
static __inline void WDT_DisableInt(void);









 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL = 0UL;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL |= (0x1ul << (6));
    return;
}









 
static __inline void WDT_DisableInt(void)
{
     
    ((WDT_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x00000UL))->CTL &= ~((0x1ul << (6)) | (0x1ul << (2)) | (0x1ul << (3)) | (0x1ul << (5)));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 693 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\wwdt.h"
 





 











 



 



 
 
 
 
#line 47 "..\\Library\\StdDriver\\inc\\wwdt.h"

 
 
 


   




 










 











 












 












 











 














 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 694 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\opa.h"
 





 











 



 



 




   



 

 
 
 
static __inline int32_t OPA_Calibration(OPA_T *opa, uint32_t u32OpaNum, uint32_t u32ClockSel, uint32_t u32LevelSel);









 









 









 









 









 









 









 










 









 
















 
static __inline int32_t OPA_Calibration(OPA_T *opa,
                                        uint32_t u32OpaNum,
                                        uint32_t u32ClockSel,
                                        uint32_t u32RefVol)
{
    uint32_t u32CALResult;
    int32_t i32Ret = 0L;

    (opa)->CALCTL = (((opa)->CALCTL) & ~((0x3ul << (4)) << (u32OpaNum << 1)));
    (opa)->CALCTL = (((opa)->CALCTL) & ~((0x1ul << (16)) << (u32OpaNum))) | (((u32RefVol) << (16)) << (u32OpaNum));
    (opa)->CALCTL |= ((0x1ul << (0)) << (u32OpaNum));
    while((opa)->CALCTL & ((0x1ul << (0)) << (u32OpaNum))) {}

    u32CALResult = ((opa)->CALST >> ((u32OpaNum)*4U)) & ((0x1ul << (1))|(0x1ul << (2)));
    if (u32CALResult == 0U)
    {
        i32Ret = 0L;
    }
    else if (u32CALResult == (0x1ul << (1)))
    {
        i32Ret = -2L;
    }
    else if (u32CALResult == (0x1ul << (2)))
    {
        i32Ret = -1L;
    }
    else if (u32CALResult == ((0x1ul << (1))|(0x1ul << (2))))
    {
        i32Ret = -3L;
    }

    return i32Ret;
}






 






   

   

   







 
#line 695 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\crc.h"
 





 











 



 



 
 
 
 





 
 
 





 
 
 




   




 













 











 











 


void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
uint32_t CRC_GetChecksum(void);

   

   

   







 
#line 696 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\ebi.h"
 





 











 



 



 
 
 
 





 
 
 




 
 
 



 
 
 



 
 
 
#line 66 "..\\Library\\StdDriver\\inc\\ebi.h"

#line 74 "..\\Library\\StdDriver\\inc\\ebi.h"





   




 










 












 











 












 











 












 











 












 











 












 











 












 











 












 











 












 











 












 











 











 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 697 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\usci_i2c.h"
 





 











 



 



 

 
 
 
enum UI2C_MASTER_EVENT
{
    MASTER_SEND_ADDRESS = 10,     
    MASTER_SEND_H_WR_ADDRESS,     
    MASTER_SEND_H_RD_ADDRESS,     
    MASTER_SEND_L_ADDRESS,        
    MASTER_SEND_DATA,             
    MASTER_SEND_REPEAT_START,     
    MASTER_READ_DATA,             
    MASTER_STOP,                  
    MASTER_SEND_START             
};

 
 
 
enum UI2C_SLAVE_EVENT
{
    SLAVE_ADDRESS_ACK = 100,       
    SLAVE_H_WR_ADDRESS_ACK,        
    SLAVE_L_WR_ADDRESS_ACK,        
    SLAVE_GET_DATA,                
    SLAVE_SEND_DATA,               
    SLAVE_H_RD_ADDRESS_ACK,        
    SLAVE_L_RD_ADDRESS_ACK         
};

 
 
 





 
 
 



 
 
 



 
 
 
#line 89 "..\\Library\\StdDriver\\inc\\usci_i2c.h"

   




 











 











 











 











 












 












 












 











 











 











 











 

















 

















 

















 



uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T* ui2c, uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
uint8_t UI2C_WriteByte(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_ReadByte(UI2C_T *ui2c, uint8_t u8SlaveAddr);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t UI2C_ReadMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen);

   

   

   







 
#line 698 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\scuart.h"
 





 











 



 



 













   




 

 






 









 









 








 









 









 









 


 






 









 










 










 









 


 











 












 














 











 










 











 


void SCUART_Close(SC_T* sc);
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate);
uint32_t SCUART_Read(SC_T* sc, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t SCUART_SetLineConfig(SC_T* sc, uint32_t u32Baudrate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t  u32StopBits);
void SCUART_SetTimeoutCnt(SC_T* sc, uint32_t u32TOC);
void SCUART_Write(SC_T* sc,uint8_t pu8TxBuf[], uint32_t u32WriteBytes);

   

   

   







 
#line 699 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\sc.h"
 





 











 



 



 
#line 34 "..\\Library\\StdDriver\\inc\\sc.h"

#line 45 "..\\Library\\StdDriver\\inc\\sc.h"


   




 


















 



















 








 
#line 109 "..\\Library\\StdDriver\\inc\\sc.h"








 
#line 126 "..\\Library\\StdDriver\\inc\\sc.h"







 
#line 142 "..\\Library\\StdDriver\\inc\\sc.h"







 
#line 158 "..\\Library\\StdDriver\\inc\\sc.h"






 








 









 


 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count);
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count);






 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (0x1ul << (30)))
    {
        ;
    }
     
    (sc)->CTL &= ~((0x7ul << (20)) | (0x1ul << (23)));

    if((u32Count) != 0UL)
    {
        while((sc)->CTL & (0x1ul << (30)))
        {
            ;
        }
        (sc)->CTL |= (((u32Count) - 1UL) << (20)) | (0x1ul << (23));
    }
}






 
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (0x1ul << (30)))
    {
        ;
    }
     
    (sc)->CTL &= ~((0x7ul << (16)) | (0x1ul << (19)));

    if((u32Count) != 0UL)
    {
        while((sc)->CTL & (0x1ul << (30)))
        {
            ;
        }
        (sc)->CTL |= (((u32Count) - 1UL) << (16)) | (0x1ul << (19));
    }

}


uint32_t SC_IsCardInserted(SC_T *sc);
void SC_ClearFIFO(SC_T *sc);
void SC_Close(SC_T *sc);
void SC_Open(SC_T *sc, uint32_t u32CardDet, uint32_t u32PWR);
void SC_ResetReader(SC_T *sc);
void SC_SetBlockGuardTime(SC_T *sc, uint32_t u32BGT);
void SC_SetCharGuardTime(SC_T *sc, uint32_t u32CGT);
void SC_StopAllTimer(SC_T *sc);
void SC_StartTimer(SC_T *sc, uint32_t u32TimerNum, uint32_t u32Mode, uint32_t u32ETUCount);
void SC_StopTimer(SC_T *sc, uint32_t u32TimerNum);
uint32_t SC_GetInterfaceClock(SC_T *sc);


   

   

   







 
#line 700 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\spi.h"
 





 











 



 



 













 
#line 52 "..\\Library\\StdDriver\\inc\\spi.h"

 
#line 62 "..\\Library\\StdDriver\\inc\\spi.h"


 





 



 





 



 




 





 



 



 
#line 111 "..\\Library\\StdDriver\\inc\\spi.h"

   




 







 








 








 








 








 








 









 









 









 








 









 








 








 








 








 










 








 








 









 









 








 








 


 
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask);
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask);
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch);









 
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0U))
    {
        i2s->I2SCTL |= (0x1ul << (16));
    }
    else
    {
        i2s->I2SCTL |= (0x1ul << (17));
    }
}









 
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0U))
    {
        i2s->I2SCTL &= ~(0x1ul << (16));
    }
    else
    {
        i2s->I2SCTL &= ~(0x1ul << (17));
    }
}







 








 








 








 








 








 








 








 








 








 








 








 











 
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->I2SCTL |= (0x1ul << (23))) :
    (i2s->I2SCTL &= ~(0x1ul << (23)));
}








 








 









 










 








 








 




 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void SPII2S_Close(SPI_T *i2s);
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask);
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask);
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock);
void SPII2S_DisableMCLK(SPI_T *i2s);
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);


   

   

   







 
#line 701 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\qspi.h"
 





 











 



 



 













 
#line 52 "..\\Library\\StdDriver\\inc\\qspi.h"

 
#line 62 "..\\Library\\StdDriver\\inc\\qspi.h"

   




 







 








 








 








 








 








 









 









 









 








 









 








 








 








 








 










 








 








 









 









 








 








 







 







 







 







 







 







 





 
uint32_t QSPI_Open(QSPI_T *qspi, uint32_t u32MasterSlave, uint32_t u32QSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void QSPI_Close(QSPI_T *qspi);
void QSPI_ClearRxFIFO(QSPI_T *qspi);
void QSPI_ClearTxFIFO(QSPI_T *qspi);
void QSPI_DisableAutoSS(QSPI_T *qspi);
void QSPI_EnableAutoSS(QSPI_T *qspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t QSPI_SetBusClock(QSPI_T *qspi, uint32_t u32BusClock);
void QSPI_SetFIFO(QSPI_T *qspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t QSPI_GetBusClock(QSPI_T *qspi);
void QSPI_EnableInt(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_DisableInt(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetIntFlag(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_ClearIntFlag(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetStatus(QSPI_T *qspi, uint32_t u32Mask);


   

   

   







 
#line 702 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\can.h"
 





 











 



 



 
 
 
 



 
 
 



 
 
 



   




 


 
typedef struct
{
    uint32_t  IdType;        
    uint32_t  FrameType;     
    uint32_t  Id;            
    uint8_t   DLC;           
    uint8_t   Data[8];       
} STR_CANMSG_T;



 
typedef struct
{
    uint8_t   u8Xtd;       
    uint8_t   u8Dir;       
    uint32_t  u32Id;       
    uint8_t   u8IdType;    
} STR_CANMASK_T;

   

 

 



 











 












 











 











 













 



 
 
 
uint32_t CAN_SetBaudRate(CAN_T *tCAN, uint32_t u32BaudRate);
uint32_t CAN_Open(CAN_T *tCAN, uint32_t u32BaudRate, uint32_t u32Mode);
void CAN_Close(CAN_T *tCAN);
void CAN_CLR_INT_PENDING_BIT(CAN_T *tCAN, uint8_t u32MsgNum);
void CAN_EnableInt(CAN_T *tCAN, uint32_t u32Mask);
void CAN_DisableInt(CAN_T *tCAN, uint32_t u32Mask);
int32_t CAN_Transmit(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_Receive(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_SetMultiRxMsg(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32MsgCount, uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsg(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsgAndMsk(CAN_T *tCAN, uint32_t u32MsgNum, uint32_t u32IDType, uint32_t u32ID, uint32_t u32IDMask);
int32_t CAN_SetTxMsg(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
int32_t CAN_TriggerTxMsg(CAN_T  *tCAN, uint32_t u32MsgNum);
int32_t CAN_BasicSendMsg(CAN_T *tCAN, STR_CANMSG_T* pCanMsg);
int32_t CAN_BasicReceiveMsg(CAN_T *tCAN, STR_CANMSG_T* pCanMsg);
void CAN_EnterInitMode(CAN_T *tCAN, uint8_t u8Mask);
void CAN_EnterTestMode(CAN_T *tCAN, uint8_t u8TestMask);
void CAN_LeaveTestMode(CAN_T *tCAN);
uint32_t CAN_GetCANBitRate(CAN_T *tCAN);
uint32_t CAN_IsNewDataReceived(CAN_T *tCAN, uint8_t u8MsgObj);
void CAN_LeaveInitMode(CAN_T *tCAN);
int32_t CAN_SetRxMsgObjAndMsk(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint32_t u32idmask, uint8_t u8singleOrFifoLast);
int32_t CAN_SetRxMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast);
void CAN_WaitMsg(CAN_T *tCAN);
int32_t CAN_ReadMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8Release, STR_CANMSG_T* pCanMsg);

   

   

   







 
#line 703 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\rtc.h"
 





 











 



 



 
 
 
 



 
 
 





 
 
 
#line 53 "..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 
#line 64 "..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 





#line 80 "..\\Library\\StdDriver\\inc\\rtc.h"












#line 100 "..\\Library\\StdDriver\\inc\\rtc.h"





   




 


 
typedef struct
{
    uint32_t u32Year;            
    uint32_t u32Month;           
    uint32_t u32Day;             
    uint32_t u32DayOfWeek;       
    uint32_t u32Hour;            
    uint32_t u32Minute;          
    uint32_t u32Second;          
    uint32_t u32TimeScale;       
    uint32_t u32AmPm;            
} S_RTC_TIME_DATA_T;

   




 











 











 











 

















 












 












 












 
















 













 














 


 
static __inline void RTC_WaitAccessEnable(void);









 
static __inline void RTC_WaitAccessEnable(void)
{
    while((((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN & (0x1ul << (24))) == (0x1ul << (24)))
    {
    }

    if(!(((SYS_T *) (((uint32_t)0x40000000) + 0x00000UL))->CSERVER & 0x1))
    {
         
        ((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN = 0x0000A965UL;
    }

     
    while((((RTC_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x01000UL))->RWEN & (0x1ul << (16))) == (uint32_t)0x0)
    {
    }
}

void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX10000);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
void RTC_EnableSpareAccess(void);
void RTC_DisableSpareRegister(void);
void RTC_StaticTamperEnable(uint32_t u32TamperSelect, uint32_t u32DetecLevel, uint32_t u32DebounceEn);
void RTC_StaticTamperDisable(uint32_t u32TamperSelect);
void RTC_DynamicTamperEnable(uint32_t u32PairSel, uint32_t u32DebounceEn, uint32_t u32Pair1Source, uint32_t u32Pair2Source);
void RTC_DynamicTamperDisable(uint32_t u32PairSel);
void RTC_DynamicTamperConfig(uint32_t u32ChangeRate, uint32_t u32SeedReload, uint32_t u32RefPattern, uint32_t u32Seed);

   

   

   







 
#line 704 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\usci_uart.h"
 





 












 



 



 

 
 
 





 
 
 







 
 
 
#line 58 "..\\Library\\StdDriver\\inc\\usci_uart.h"


   




 












 












 













 













 














 














 












 













 













 













 













 















 















 














 














 

















 

















 












 






















 












 














 













 












 



void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T*  uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T*  uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
uint32_t UUART_Write(UUART_T* uuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
void UUART_EnableFlowCtrl(UUART_T* uuart);
void UUART_DisableFlowCtrl(UUART_T* uuart);


   

   

   







 
#line 705 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\sdh.h"
 





 
#line 9 "..\\Library\\StdDriver\\inc\\sdh.h"












 



 




 






 



 






 
#line 59 "..\\Library\\StdDriver\\inc\\sdh.h"










   



 
typedef struct SDH_info_t
{
    unsigned int    CardType;        
    unsigned int    RCA;             
    unsigned char   IsCardInsert;    
    unsigned int    totalSectorN;    
    unsigned int    diskSize;        
    int             sectorSize;      
} SDH_INFO_T;                        

   

 
extern uint8_t g_u8R3Flag;
extern uint8_t volatile g_u8SDDataReadyFlag;
extern SDH_INFO_T SD0, SD1;
 



 












 












 

















 














 











 









 



void SDH_Open(SDH_T *sdh, uint32_t u32CardDetSrc);
uint32_t SDH_Probe(SDH_T *sdh);
uint32_t SDH_Read(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount);
uint32_t SDH_Write(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount);

uint32_t SDH_CardDetection(SDH_T *sdh);
void SDH_Open_Disk(SDH_T *sdh, uint32_t u32CardDetSrc);
void SDH_Close_Disk(SDH_T *sdh);


   

   

   






 
#line 706 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\usbd.h"
 





 











 



 



 
typedef struct s_usbd_info
{
    uint8_t *gu8DevDesc;             
    uint8_t *gu8ConfigDesc;          
    uint8_t **gu8StringDesc;         
    uint8_t **gu8HidReportDesc;      
    uint8_t *gu8BosDesc;             
    uint32_t *gu32HidReportSize;     
    uint32_t *gu32ConfigHidDescIdx;  

} S_USBD_INFO_T;   

extern const S_USBD_INFO_T gsInfo;

   






 



#line 65 "..\\Library\\StdDriver\\inc\\usbd.h"

 
 




 
#line 84 "..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 97 "..\\Library\\StdDriver\\inc\\usbd.h"

 



 



 
#line 117 "..\\Library\\StdDriver\\inc\\usbd.h"

 







 


 

 
 
 














#line 165 "..\\Library\\StdDriver\\inc\\usbd.h"
















   




 










 













 












 











 











 











 











 











 











 











 














 











 














 











 















 












 











 












 












 













 











 













 













 











 











 











 












 















 
static __inline void USBD_MemCopy(uint8_t dest[], uint8_t src[], uint32_t size)
{
    uint32_t volatile i=0ul;

    while(size--)
    {
        dest[i] = src[i];
        i++;
    }
}










 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (0x1ul << (1)));
            break;
        }
    }
}









 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(0x1ul << (1)));
            break;
        }
    }
}











 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 12ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((((uint32_t)0x40000000) + (uint32_t)0x00040000) + 0x80000UL))->EP[0].CFGP;  
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (0x1ul << (1)));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;


typedef void (*VENDOR_REQ)(void);            
typedef void (*CLASS_REQ)(void);             
typedef void (*SET_INTERFACE_REQ)(uint32_t u32AltInterface);     
typedef void (*SET_CONFIG_CB)(void);        


 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);

   

   

   







 
#line 707 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\hsusbd.h"
 





 











 



 



 
 






#line 48 "..\\Library\\StdDriver\\inc\\hsusbd.h"

 
 





 
#line 67 "..\\Library\\StdDriver\\inc\\hsusbd.h"

 
#line 76 "..\\Library\\StdDriver\\inc\\hsusbd.h"


   



 


typedef struct HSUSBD_CMD_STRUCT
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;

} S_HSUSBD_CMD_T;  




typedef struct s_hsusbd_info
{
    uint8_t *gu8DevDesc;             
    uint8_t *gu8ConfigDesc;          
    uint8_t **gu8StringDesc;         
    uint8_t *gu8QualDesc;            
    uint8_t *gu8FullConfigDesc;      
    uint8_t *gu8HSOtherConfigDesc;   
    uint8_t *gu8FSOtherConfigDesc;   
    uint8_t **gu8HidReportDesc;      
    uint32_t *gu32HidReportSize;     
    uint8_t **gu8FSHidReportDesc;    
    uint32_t *gu32FSHidReportSize;   

} S_HSUSBD_INFO_T;  


   

 
extern uint32_t g_u32HsEpStallLock;
extern uint8_t g_hsusbd_Configured;
extern uint8_t g_hsusbd_ShortPacket;
extern uint8_t g_hsusbd_CtrlZero;
extern uint8_t g_hsusbd_UsbAddr;
extern uint32_t volatile g_hsusbd_DmaDone;
extern uint32_t g_hsusbd_CtrlInSize;
extern S_HSUSBD_INFO_T gsHSInfo;
extern S_HSUSBD_CMD_T gUsbCmd;
 



 

#line 159 "..\\Library\\StdDriver\\inc\\hsusbd.h"







 
static __inline void HSUSBD_MemCopy(uint8_t u8Dst[], uint8_t u8Src[], uint32_t u32Size)
{
    uint32_t i = 0ul;

    while (u32Size--)
    {
        u8Dst[i] = u8Src[i];
        i++;
    }
}





 
static __inline void HSUSBD_ResetDMA(void)
{
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACNT = 0ul;
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACTL = 0x80ul;
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->DMACTL = 0x00ul;
}






 
static __inline void HSUSBD_SetEpBufAddr(uint32_t u32Ep, uint32_t u32Base, uint32_t u32Len)
{
    if (u32Ep == 0xfful)
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPBUFST = u32Base;
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPBUFEND   = u32Base + u32Len - 1ul;
    }
    else
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPBUFST = u32Base;
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPBUFEND = u32Base + u32Len - 1ul;
    }
}








 
static __inline void HSUSBD_ConfigEp(uint32_t u32Ep, uint32_t u32EpNum, uint32_t u32EpType, uint32_t u32EpDir)
{
    if (u32EpType == ((uint32_t)0x00000002ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000000ul));
    }
    else if (u32EpType == ((uint32_t)0x00000004ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000002ul));
    }
    else if (u32EpType == ((uint32_t)0x00000006ul))
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((uint32_t)0x00000001ul)|((uint32_t)0x00000004ul));
    }

    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPCFG = (u32EpType|u32EpDir|((uint32_t)0x00000001ul)|(u32EpNum << 4));
}






 
static __inline void HSUSBD_SetEpStall(uint32_t u32Ep)
{
    if (u32Ep == 0xfful)
    {
        (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPCTL = (((uint32_t)0x00000002ul)));
    }
    else
    {
        ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL & 0xf7ul) | ((uint32_t)0x00000010ul);
    }
}








 
static __inline void HSUSBD_SetStall(uint32_t u32EpNum)
{
    uint32_t i;

    if (u32EpNum == 0ul)
    {
        (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->CEPCTL = (((uint32_t)0x00000002ul)));
    }
    else
    {
        for (i=0ul; i<12ul; i++)
        {
            if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
            {
                ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL & 0xf7ul) | ((uint32_t)0x00000010ul);
            }
        }
    }
}






 
static __inline void  HSUSBD_ClearEpStall(uint32_t u32Ep)
{
    ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL = ((uint32_t)0x00000008ul);
}








 
static __inline void HSUSBD_ClearStall(uint32_t u32EpNum)
{
    uint32_t i;

    for (i=0ul; i<12ul; i++)
    {
        if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
        {
            ((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL = ((uint32_t)0x00000008ul);
        }
    }
}







 
static __inline uint32_t HSUSBD_GetEpStall(uint32_t u32Ep)
{
    return (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[u32Ep].EPRSPCTL & ((uint32_t)0x00000010ul));
}









 
static __inline uint32_t HSUSBD_GetStall(uint32_t u32EpNum)
{
    uint32_t i;
    uint32_t val = 0ul;

    for (i=0ul; i<12ul; i++)
    {
        if (((((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPCFG & 0xf0ul) >> 4) == u32EpNum)
        {
            val = (((HSUSBD_T *)(((uint32_t)0x40000000) + 0x19000UL))->EP[i].EPRSPCTL & ((uint32_t)0x00000010ul));
            break;
        }
    }
    return val;
}


 
typedef void (*HSUSBD_VENDOR_REQ)(void);  
typedef void (*HSUSBD_CLASS_REQ)(void);  
typedef void (*HSUSBD_SET_INTERFACE_REQ)(uint32_t u32AltInterface);  

void HSUSBD_Open(S_HSUSBD_INFO_T *param, HSUSBD_CLASS_REQ pfnClassReq, HSUSBD_SET_INTERFACE_REQ pfnSetInterface);
void HSUSBD_Start(void);
void HSUSBD_ProcessSetupPacket(void);
void HSUSBD_StandardRequest(void);
void HSUSBD_UpdateDeviceState(void);
void HSUSBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void HSUSBD_CtrlIn(void);
void HSUSBD_CtrlOut(uint8_t pu8Buf[], uint32_t u32Size);
void HSUSBD_SwReset(void);
void HSUSBD_SetVendorRequest(HSUSBD_VENDOR_REQ pfnVendorReq);



   

   

   







 
#line 708 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\otg.h"
 






 











 



 




 



 
 
 






   




 

 
 
 








 








 









 








 








 








 








 








 










 










 





















 





















 





















 





















 














 




   

   

   









 
#line 709 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"
#line 1 "..\\Library\\StdDriver\\inc\\hsotg.h"
 






 











 



 




 



 
 
 






   




 

 
 
 








 








 









 








 








 








 








 








 










 










 





















 





















 





















 





















 














 




   

   

   









 
#line 710 "..\\Library\\Device\\Nuvoton\\M480\\Include\\M480.h"








#line 12 "..\\Library\\Device\\Nuvoton\\M480\\Include\\NuMicro.h"




#line 49 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
#line 1 "..\\For_SSC_Tool\\Src\\esc.h"



 




 














 





 
#line 32 "..\\For_SSC_Tool\\Src\\esc.h"

#line 180 "..\\For_SSC_Tool\\Src\\esc.h"
 
#line 50 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 
	 


#line 1 "..\\Source\\Debug\\printd.h"




























 




 
#line 1 "..\\Source\\Debug\\ax_uart.h"




























 




 
#line 36 "..\\Source\\Debug\\ax_uart.h"

 
#line 44 "..\\Source\\Debug\\ax_uart.h"

#line 52 "..\\Source\\Debug\\ax_uart.h"

 
typedef struct{
	UART_T*                  Instance;
	
	unsigned char            RxBuf[512];
	volatile unsigned long   RxWrPtr;
	volatile unsigned long   RxRdPtr;
	volatile unsigned long   RxBufFree;
	
	unsigned char            TxBuf[1024];
	volatile unsigned long   TxWrPtr;
	volatile unsigned long   TxRdPtr;
	volatile unsigned long   TxBufFree;	
} AX_UART_OBJECT;
																								
 
  
int AX_UART_Init(void);
int AX_UART_DeInit(void);
char AX_UART_GetChar(void);
unsigned char AX_UART_PutChar(unsigned char);
int AX_UART_CheckTxBufEmpty(void);


 
#line 36 "..\\Source\\Debug\\printd.h"

 



 
																								
 

  
short printd(const char *fmt, ...);
void printdCtrl(char enb);



 
#line 57 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"




	 


	 


	 


	 
#line 77 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"
	 


	 






 
#line 103 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

	 






	 





	 





	 
#line 138 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

#line 145 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

	 
#line 153 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 
typedef union
{
	unsigned char	Byte[2];
	unsigned short	Word;
} UALEVENT;

typedef union
{
	unsigned long	d32[2];
	struct{
		unsigned long vendor_id:      32;  
		unsigned long reserved_63_32: 32;  
	} b;
} oIC_VENDOR_ID;

typedef union
{
	unsigned long	d32[2];
	struct{
		unsigned long chip_revision:	 8;   
		unsigned long package_type:	 4;	  
		unsigned long product_id:		 20;  
		unsigned long reserved_63_32: 32;  
	} b;
} oIC_PRODUCT_ID;

#line 195 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

typedef struct{
	unsigned long	EscIsrCnt;
	unsigned long	FunIsrCnt;
	unsigned long	Sync0IsrCnt;
	unsigned long	Sync1IsrCnt;
	unsigned long	AdcTrgCnt;
	unsigned long	TIM1IsrCnt;
	unsigned long	EncIsrCnt;
	unsigned long	TmrTskIsrCnt;
} HW_DEBUG;

typedef struct{
	SPI_T*		pInst;
	GPIO_T*		pCsPort;
	unsigned long		CsPin;
	volatile unsigned char	Lock;
	unsigned long		ReentryCnt;
	unsigned char		ThreeByteAddrMode;
	unsigned char		TxBuf[(4 + 512)];
	unsigned char		RxBuf[(4 + 512)];
} HW_SPI_OBJECT;

 

 


 






 






 






 






 






 




 




 






 





 




 




 




#line 321 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"



 




 




 






 





 




 




 




#line 386 "..\\For_SSC_Tool\\Src\\AX58200_Hw.h"

 

 unsigned char HW_Init(void);
 void HW_Release(void);
 unsigned short HW_GetALEventRegister(void);


 unsigned short HW_GetALEventRegister_Isr(void);


 void HW_EscRead( unsigned long * pData, unsigned short Address, unsigned short Len );

 void HW_EscReadIsr( unsigned long *pData, unsigned short Address, unsigned short Len );


 void HW_EscWrite( unsigned long *pData, unsigned short Address, unsigned short Len );


 void HW_EscWriteIsr( unsigned long *pData, unsigned short Address, unsigned short Len );



 void HW_RestartTarget(void);






 unsigned long HW_GPIO_PinToIntrNum(unsigned long Pin);
 long HW_MultiFuncPins(GPIO_T* Instance, unsigned long Pins, unsigned char MultiFuncValue);
 void HW_TMR_ClkSource(TIMER_T* Instance, unsigned long NewState);
 void HW_SPI_ClkSource(SPI_T* Instance, unsigned char NewState);
 void HW_UART_ClkSource(UART_T* Instance, unsigned char NewState);
 void HW_GPIO_WritePin(GPIO_T* Instance, unsigned long Pin, unsigned char NewState);
 void HW_GPIO_TogglePin(GPIO_T* Instance, unsigned long Pin);
 unsigned char HW_GPIO_ReadPin(GPIO_T* Instance, unsigned long Pin);


unsigned char HW_CheckTimeout(unsigned short StartTime, unsigned short Timeout);
HW_DEBUG* HW_GetDebugCounter(void);



 
 
#line 46 "..\\For_SSC_Tool\\Src\\ecatslv.h"









 


     
     


































































































 













































 

#line 221 "..\\For_SSC_Tool\\Src\\ecatslv.h"

#line 229 "..\\For_SSC_Tool\\Src\\ecatslv.h"










 
#line 299 "..\\For_SSC_Tool\\Src\\ecatslv.h"
                                                              

                                                              


 









 
#line 324 "..\\For_SSC_Tool\\Src\\ecatslv.h"














#line 344 "..\\For_SSC_Tool\\Src\\ecatslv.h"













 
    unsigned char                           bBootMode;  
    unsigned char                           bEcatOutputUpdateRunning;  
 

    unsigned char                            bEcatInputUpdateRunning;  
 

    unsigned char                           bEcatFirstOutputsReceived; 

 

    unsigned char                           bWdTrigger;  

    unsigned char                           bDcSyncActive;  

    short                          EsmTimeoutCounter; 
 



 unsigned char                              bDcRunning;  

 unsigned short                            u16SmSync0Counter;  
 unsigned short                            u16SmSync0Value;  

 unsigned char                              bSmSyncSequenceValid;  

 short                             i16WaitForPllRunningTimeout;  

 short                             i16WaitForPllRunningCnt;  

 unsigned short                            Sync0WdCounter;  
 unsigned short                            Sync0WdValue;  

 unsigned short                            Sync1WdCounter;  
 unsigned short                            Sync1WdValue;  

 unsigned short                            LatchInputSync0Value;  
 unsigned short                            LatchInputSync0Counter;  

 
 unsigned char b32BitDc;
 



 unsigned char                              bEscIntEnabled; 
 

 unsigned char                              b3BufferMode;  

 unsigned char                              bLocalErrorFlag;  
 unsigned short                            u16LocalErrorCode;  
 unsigned char                              bApplEsmPending;  
 unsigned char                              bEcatWaitForAlControlRes; 
 

 unsigned short                            nEcatStateTrans;  


 unsigned short                            nPdInputSize;  

 unsigned short                            nPdOutputSize;  

 unsigned char                             nMaxSyncMan;  

 unsigned short                            nMaxEscAddress;  

 unsigned char                             nAlStatus;  


 unsigned short                            EcatWdValue; 
 
    unsigned short                         nEscAddrOutputData;  
    unsigned short                         nEscAddrInputData;  






 
 void EnableSyncManChannel(unsigned char channel);
 void DisableSyncManChannel(unsigned char channel);
 TSYNCMAN  *GetSyncMan(unsigned char channel);
 void SetALStatus(unsigned char alStatus, unsigned short alStatusCode);
 void AL_ControlInd(unsigned char alControl, unsigned short alStatusCode);
 void DC_CheckWatchdog(void);
    void CheckIfEcatError(void);
 void ECAT_Init(void);

 void ECAT_Main(void);


 
#line 191 "..\\For_SSC_Tool\\Src\\ecatslv.c"

 

#line 1 "..\\For_SSC_Tool\\Src\\ecatappl.h"



 




 



















 








 





 

#line 47 "..\\For_SSC_Tool\\Src\\ecatappl.h"









 

 
#line 85 "..\\For_SSC_Tool\\Src\\ecatappl.h"












 



extern unsigned char bEcatWaitForInputUpdate;  
extern unsigned char bEtherCATRunLed;  
extern unsigned char bEtherCATErrorLed;  
extern unsigned char bRunApplication;  







 




extern    void       ECAT_CheckTimer(void);
extern    void       PDI_Isr(void);
extern    void       Sync0_Isr(void);
extern    void       Sync1_Isr(void);
extern    void       ECAT_Application(void);
extern    void       PDO_ResetOutputs(void);
extern    void       PDO_ReadInputs(void);
extern    void       PDO_InputMapping(void);

extern    void       CalcSMCycleTime(void);
extern    unsigned short     ESC_EepromAccess(unsigned long wordaddress, unsigned short wordsize, unsigned short  *pData, unsigned char access);
extern    unsigned short     ESC_EepromWriteCRC(void);





 
#line 195 "..\\For_SSC_Tool\\Src\\ecatslv.c"


#line 1 "..\\For_SSC_Tool\\Src\\bootmode.h"



 




 












 





 
#line 30 "..\\For_SSC_Tool\\Src\\bootmode.h"









 








typedef enum{
	BL_STATE_IDLE = 0,
	BL_STATE_START,
	BL_STATE_START_DOWNLOAD,
	BL_STATE_DATA,
	BL_STATE_FW_TRANSMIT_DONE,
   BL_STATE_PARAMETER_TRANSMIT_DONE,
} BL_STATE;
 



 






 
extern    void BL_Start( unsigned char State);
extern   void BL_Stop(void);
extern    void BL_StartDownload(unsigned long password);
extern	unsigned short BL_Data(unsigned short *pData,unsigned short Size);
extern	void BL_Init(void);
extern	void BL_JumpToLdrom(void);




 
#line 198 "..\\For_SSC_Tool\\Src\\ecatslv.c"



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


 
#line 202 "..\\For_SSC_Tool\\Src\\ecatslv.c"

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


 
#line 204 "..\\For_SSC_Tool\\Src\\ecatslv.c"
#line 1 "..\\For_SSC_Tool\\Src\\objdef.h"



 




 






















 





 
#line 1 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 




 



















 






 
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
 



 
#line 40 "..\\For_SSC_Tool\\Src\\objdef.h"










 



 
#line 119 "..\\For_SSC_Tool\\Src\\objdef.h"




 

#line 135 "..\\For_SSC_Tool\\Src\\objdef.h"









 
typedef struct OBJ_ENTRY
{
    struct OBJ_ENTRY                      *pPrev;  
    struct OBJ_ENTRY                      *pNext;  

    unsigned short                                Index;  
    TSDOINFOOBJDESC                       ObjDesc;  
    const TSDOINFOENTRYDESC      *pEntryDesc;  
    const unsigned char                  *pName;  
    void                            *pVarPtr;  
    unsigned char (* Read)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
    unsigned char (* Write)( unsigned short Index, unsigned char Subindex, unsigned long Size, unsigned short  * pData, unsigned char bCompleteAccess );  
    unsigned short                                 NonVolatileOffset;  
}
TOBJECT;




 
typedef struct 
{
    unsigned short    subindex0; 
    unsigned short    u16SyncType;  
    unsigned long    u32CycleTime; 
    unsigned long    u32Subindex003; 
    unsigned short    u16SyncTypesSupported; 
    unsigned long    u32MinCycleTime; 
    unsigned long    u32CalcAndCopyTime; 
    unsigned long    u32Si7Reserved; 
    unsigned short    u16GetCycleTime; 
    unsigned long    u32DelayTime;  
    unsigned long    u32Sync0CycleTime;  
    unsigned short    u16SmEventMissedCounter;  
    unsigned short    u16CycleExceededCounter;  
    unsigned short    u16Si13Reserved;  
    unsigned short    u16Si14Reserved;  
    unsigned long    u32Si15Reserved;  
    unsigned long    u32Si16Reserved;  
    unsigned long    u32Si17Reserved;  
    unsigned long    u32Si18Reserved;  
    unsigned char    u8SyncError;  
}
TSYNCMANPAR;



 
typedef struct 
{
    unsigned short    syncFailedCounter;  
}
TCYCLEDIAG;




 
typedef struct  {
   unsigned short   u16SubIndex0;  
   unsigned long   u32LocalErrorReaction;  
   unsigned short   u16SyncErrorCounterLimit;  
} 
TOBJ10F1;








 









 
extern unsigned char bSyncSetByUser;



 
extern TCYCLEDIAG sCycleDiag;




 
extern TSYNCMANPAR  sSyncManOutPar;




 
extern TSYNCMANPAR  sSyncManInPar;








 
extern TOBJ10F1 sErrorSettings



;




 
extern    char          aSubindexDesc[13]



;






 

extern    const TOBJECT  *  OBJ_GetObjectHandle( unsigned short index );
extern    unsigned long  OBJ_GetObjectLength( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned char bCompleteAccess);
extern    unsigned short  OBJ_GetNoOfObjects(unsigned char listType);
extern    unsigned short  OBJ_GetObjectList(unsigned short listType, unsigned short *pIndex, unsigned short size, unsigned short  *pData,unsigned char *pAbort);
extern    unsigned short  OBJ_GetDesc( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned short  * pData );
extern    const TSDOINFOENTRYDESC  * OBJ_GetEntryDesc(const TOBJECT  * pObjEntry, unsigned char Subindex);
extern    const TSDOINFOOBJDESC  * OBJ_GetObjDesc(const TOBJECT  * pObjEntry);
extern    unsigned short  OBJ_GetEntryOffset(unsigned char subindex, const TOBJECT  * pObjEntry);
extern    unsigned char   CheckSyncTypeValue(unsigned short index, unsigned short NewSyncType);
extern    unsigned char   OBJ_Read(unsigned short index, unsigned char subindex, unsigned long objSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    unsigned char   OBJ_Write(unsigned short index, unsigned char subindex, unsigned long dataSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    void    COE_WriteBackupEntry(unsigned char subindex, const TOBJECT  * pObjEntry);


 
#line 205 "..\\For_SSC_Tool\\Src\\ecatslv.c"



#line 1 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




























 
 



 






 

 








 
#line 54 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\For_SSC_Tool\\Src\\objdef.h"



 




 






















 





 
#line 1 "..\\For_SSC_Tool\\Src\\sdoserv.h"



 




 



















 






 
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











 
extern    TMBX  *  pCoeSendStored;                



 





 

extern   void     COE_Init(void);
extern   unsigned char    COE_ServiceInd(TCOEMBX  *pCoeMbx);
extern   unsigned char     COE_ContinueInd(TMBX  * pMbx);


 
#line 38 "..\\For_SSC_Tool\\Src\\sdoserv.h"


#line 599 "..\\For_SSC_Tool\\Src\\sdoserv.h"





 









 
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
 



 
#line 40 "..\\For_SSC_Tool\\Src\\objdef.h"


#line 211 "..\\For_SSC_Tool\\Src\\objdef.h"






 









 
extern unsigned char bSyncSetByUser;



 
extern TCYCLEDIAG sCycleDiag;




 
extern TSYNCMANPAR  sSyncManOutPar;




 
extern TSYNCMANPAR  sSyncManInPar;








 
extern TOBJ10F1 sErrorSettings



;




 
extern    char          aSubindexDesc[13]



;






 

extern    const TOBJECT  *  OBJ_GetObjectHandle( unsigned short index );
extern    unsigned long  OBJ_GetObjectLength( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned char bCompleteAccess);
extern    unsigned short  OBJ_GetNoOfObjects(unsigned char listType);
extern    unsigned short  OBJ_GetObjectList(unsigned short listType, unsigned short *pIndex, unsigned short size, unsigned short  *pData,unsigned char *pAbort);
extern    unsigned short  OBJ_GetDesc( unsigned short index, unsigned char subindex, const TOBJECT  * pObjEntry, unsigned short  * pData );
extern    const TSDOINFOENTRYDESC  * OBJ_GetEntryDesc(const TOBJECT  * pObjEntry, unsigned char Subindex);
extern    const TSDOINFOOBJDESC  * OBJ_GetObjDesc(const TOBJECT  * pObjEntry);
extern    unsigned short  OBJ_GetEntryOffset(unsigned char subindex, const TOBJECT  * pObjEntry);
extern    unsigned char   CheckSyncTypeValue(unsigned short index, unsigned short NewSyncType);
extern    unsigned char   OBJ_Read(unsigned short index, unsigned char subindex, unsigned long objSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    unsigned char   OBJ_Write(unsigned short index, unsigned char subindex, unsigned long dataSize, const TOBJECT  * pObjEntry, unsigned short  * pData, unsigned char bCompleteAccess);
extern    void    COE_WriteBackupEntry(unsigned char subindex, const TOBJECT  * pObjEntry);


 
#line 55 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\For_SSC_Tool\\Src\\ecatappl.h"



 




 



















 








 





 

#line 47 "..\\For_SSC_Tool\\Src\\ecatappl.h"


#line 85 "..\\For_SSC_Tool\\Src\\ecatappl.h"












 



extern unsigned char bEcatWaitForInputUpdate;  
extern unsigned char bEtherCATRunLed;  
extern unsigned char bEtherCATErrorLed;  
extern unsigned char bRunApplication;  







 




extern    void       ECAT_CheckTimer(void);
extern    void       PDI_Isr(void);
extern    void       Sync0_Isr(void);
extern    void       Sync1_Isr(void);
extern    void       ECAT_Application(void);
extern    void       PDO_ResetOutputs(void);
extern    void       PDO_ReadInputs(void);
extern    void       PDO_InputMapping(void);

extern    void       CalcSMCycleTime(void);
extern    unsigned short     ESC_EepromAccess(unsigned long wordaddress, unsigned short wordsize, unsigned short  *pData, unsigned char access);
extern    unsigned short     ESC_EepromWriteCRC(void);





 
#line 56 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 57 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\Source\\McInterface\\mc.h"




























 




 
#line 1 "..\\Source\\McInterface\\mc_cfg.h"




























 




 
#line 1 "..\\Source\\McInterface\\mc_types.h"




























 




 
#line 36 "..\\Source\\McInterface\\mc_types.h"

 


 
typedef enum
{
	MCSTS_BUSY      = 1,
	MCSTS_OK				= 0,
	MCSTS_ERR       = -1,
} MC_STATUS_E;

typedef enum
{
	MCFS_NO_ERR     	= 0x0000,
	MCFS_OVER_VOLT  	= 0x0001,
	MCFS_UNDER_VOLT 	= 0x0002,
	MCFS_OVER_TEMP  	= 0x0004,
	MCFS_PHASE_ERR  	= 0x0008,
	MCFS_OVER_CURR  	= 0x0010,
	MCFS_SW_ERR     	= 0x0020,
	MCFS_VEC_ALI_ERR	= 0x0040,
	MCFS_ENC_ALI_ERR	= 0x0080,
} MC_FAULT_E;

typedef enum
{
	MC_STOP = 0,
	MC_START,
} MC_STSP_E;

typedef enum
{
	MC_DISABLE = 0,
	MC_ENABLE,
} MC_EADA_E;

typedef enum
{
	MC_FORWARD = 0,
	MC_BACKWARD,
} MC_DIR_E;

 



 

 




 
#line 36 "..\\Source\\McInterface\\mc_cfg.h"

 








#line 1 "..\\Source\\McStack\\param_def.h"




























 




 
extern int mapMemory[];

#line 50 "..\\Source\\McStack\\param_def.h"





 
#line 62 "..\\Source\\McStack\\param_def.h"

#line 73 "..\\Source\\McStack\\param_def.h"

#line 90 "..\\Source\\McStack\\param_def.h"

 
#line 102 "..\\Source\\McStack\\param_def.h"














 
#line 125 "..\\Source\\McStack\\param_def.h"


typedef enum
{
	LC_ALL_OFF	    = 0,
	LC_RESET        = 1,
	LC_PWM_CTRL		= 2,
	LC_AMP_CTRL	    = 3,
	LC_SPD_CTRL	    = 4,
	LC_POS_CTRL	    = 5,
	LC_PP_CTRL	    = 6,
} LOOP_CTRL_E;

#line 145 "..\\Source\\McStack\\param_def.h"

 
#line 154 "..\\Source\\McStack\\param_def.h"






#line 167 "..\\Source\\McStack\\param_def.h"






 
#line 184 "..\\Source\\McStack\\param_def.h"

#line 195 "..\\Source\\McStack\\param_def.h"

 
#line 208 "..\\Source\\McStack\\param_def.h"

#line 220 "..\\Source\\McStack\\param_def.h"

 
#line 232 "..\\Source\\McStack\\param_def.h"

#line 240 "..\\Source\\McStack\\param_def.h"

#line 251 "..\\Source\\McStack\\param_def.h"

#line 259 "..\\Source\\McStack\\param_def.h"

 
#line 271 "..\\Source\\McStack\\param_def.h"

#line 280 "..\\Source\\McStack\\param_def.h"

#line 291 "..\\Source\\McStack\\param_def.h"

#line 300 "..\\Source\\McStack\\param_def.h"

 
#line 315 "..\\Source\\McStack\\param_def.h"

#line 329 "..\\Source\\McStack\\param_def.h"

 
#line 338 "..\\Source\\McStack\\param_def.h"

#line 346 "..\\Source\\McStack\\param_def.h"

 
#line 356 "..\\Source\\McStack\\param_def.h"







#line 371 "..\\Source\\McStack\\param_def.h"







 
#line 389 "..\\Source\\McStack\\param_def.h"

#line 400 "..\\Source\\McStack\\param_def.h"

 
#line 409 "..\\Source\\McStack\\param_def.h"

#line 417 "..\\Source\\McStack\\param_def.h"

 
#line 430 "..\\Source\\McStack\\param_def.h"

#line 442 "..\\Source\\McStack\\param_def.h"


 






#line 457 "..\\Source\\McStack\\param_def.h"

 
#line 466 "..\\Source\\McStack\\param_def.h"

 
#line 474 "..\\Source\\McStack\\param_def.h"






 

 




 

#line 47 "..\\Source\\McInterface\\mc_cfg.h"

#line 50 "..\\Source\\McInterface\\mc_cfg.h"





 






 


 
#line 74 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 84 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 92 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 108 "..\\Source\\McInterface\\mc_cfg.h"

 
#line 120 "..\\Source\\McInterface\\mc_cfg.h"

 






 








 

 

 



 
#line 36 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McInterface\\mc_math.h"




























 




 
#line 36 "..\\Source\\McInterface\\mc_math.h"

 

 
typedef struct
{
	int32_t Alpha;
	int32_t Beta;
} MCMATH_AB_PHASE_T;

typedef struct
{
	int32_t Direct;
	int32_t Quadrature;
} MCMATH_DQ_PHASE_T;

 

 

 
static int MI_GetSIN(int d);
MCMATH_AB_PHASE_T MI_Clake(void);
void MI_Park(MCMATH_AB_PHASE_T *pAB);
MCMATH_AB_PHASE_T MI_RevPark(void);

void MI_MapUvwBlkInit(void);
void MI_CalcRotorVector(void);
void MI_MapAngBlkInit(void);
void MI_GetEncAngle_AbzMode(void);
void MI_CalcEncAngle(uint32_t ResetAngle);

void MI_FocCurrController(void);


 
#line 37 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\bus_vol_sensor.h"




























 




 
#line 36 "..\\Source\\McStack\\bus_vol_sensor.h"
#line 1 "..\\Source\\Mchal\\McHal_eadc.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_eadc.h"

 

 
typedef struct
{
	EADC_T		*pInst;
	GPIO_T		*pPort[5];
	IRQn_Type	IrqType;
	uint32_t	Channel[5];
	uint8_t		MultiFuncValue;
	uint32_t	SampleModuleMask[5];
	int32_t		ConvBuffer[5];
} MH_AdcRegConfig_t;

 

 

 
int32_t MH_AdcRegInit(MH_AdcRegConfig_t *pHandle, uint16_t v);
void 		MH_AdcRegDeInit(MH_AdcRegConfig_t *pHandle);
uint16_t 	MH_AdcConv(MH_AdcRegConfig_t *pHandle, uint16_t v);
void 		MH_AdcStop(MH_AdcRegConfig_t *pHandle, uint16_t v);




 

#line 37 "..\\Source\\McStack\\bus_vol_sensor.h"

 

 
typedef struct
{
    MH_AdcRegConfig_t	    VbusRegister;
    uint16_t			    ConversionFactor;
    uint16_t			    FaultState;
    uint8_t				    ChannelNum;
} MS_Vbus_Handle_t;

 

 

 
void MS_MapAdcBlkInit(void);
int32_t MS_GetBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetAvBusVoltage_d(MS_Vbus_Handle_t *pHandle);
uint16_t MS_GetBusVoltage_V(MS_Vbus_Handle_t *pHandle);
uint16_t MS_CheckVbus(MS_Vbus_Handle_t *pHandle);



 
#line 38 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\temp_sensor.h"




























 




 
#line 36 "..\\Source\\McStack\\temp_sensor.h"
#line 37 "..\\Source\\McStack\\temp_sensor.h"

 

 
typedef struct{
	MH_AdcRegConfig_t   TempRegister;
	int32_t             MCP9700V0C;         
	int32_t             MCP9700TC;          
	uint16_t	        FaultState;
	uint8_t             ChannelNum;
} MS_Temp_Handle_t;

 

 

 
uint16_t MS_GetTemp_d(MS_Temp_Handle_t *pHandle);
uint16_t MS_GetTemp_V( MS_Temp_Handle_t * pHandle );
uint16_t MS_CheckTemp( MS_Temp_Handle_t * pHandle );


 
#line 39 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\curr_fdbk.h"




























 




 
#line 36 "..\\Source\\McStack\\curr_fdbk.h"
#line 37 "..\\Source\\McStack\\curr_fdbk.h"

 

 
typedef struct{
	MH_AdcRegConfig_t       CurrentRegister;
	uint16_t			    FaultState;
	uint8_t                 ChannelNum;
}MS_Curr_Handle_t;

 

 

 
MS_Curr_Handle_t	MS_GetCurr_d(MS_Curr_Handle_t *pHandle);
MS_Curr_Handle_t	MS_GetSignedCurr_d(MS_Curr_Handle_t *pHandle);



 
#line 40 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\svpwm.h"




























 




 
#line 36 "..\\Source\\McStack\\svpwm.h"
#line 37 "..\\Source\\McStack\\svpwm.h"
#line 1 "..\\Source\\Mchal\\McHal_epwm.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_epwm.h"

 



 
typedef struct {
	EPWM_T    *pInst;
	IRQn_Type  IrqType;
	uint32_t   PreemptPrio;
	uint32_t   SubPrio;
	GPIO_T    *pPortPwm;
	uint32_t   PinPwmU;
	uint32_t   PinPwmV;
	uint32_t   PinPwmW;
	uint8_t    MultiFuncValue;
	uint32_t   EpwmChU;
	uint32_t   EpwmChV;
	uint32_t   EpwmChW;
} MH_EpwmRegConfig_t;

 










 

 
int32_t MH_EpwmRegInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmRegDeInit(MH_EpwmRegConfig_t *pHandle);
void MH_EpwmPutU(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutV(MH_EpwmRegConfig_t *pHandle , uint32_t value);
void MH_EpwmPutW(MH_EpwmRegConfig_t *pHandle , uint32_t value);

 

#line 38 "..\\Source\\McStack\\svpwm.h"

 

 
typedef enum
{
	SCM_DISABLE 			= 0,
	SCM_VECTOR_ALIGN	= 1,
	SCM_MANUAL_CTRL 	= 2,
	SCM_AUTO_CTRL 		= 9,
} SVP_CTRL_MODE_E;

typedef struct
{
	MH_EpwmRegConfig_t EpwmRegister;
	uint8_t Sector;                     
} MS_Epwm_Handle_t;

 

 

 
void MS_MapPwmBlkInit(void);
void MS_SetPhaseVoltage(MS_Epwm_Handle_t *pHandle, MCMATH_AB_PHASE_T Vab);




 
#line 41 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\incremental_fdbk.h"




























 




 
#line 36 "..\\Source\\McStack\\incremental_fdbk.h"
#line 1 "..\\Source\\Mchal\\McHal_qei.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_qei.h"

 

 
typedef struct
{
	QEI_T	    *pInst;
	GPIO_T	    *pPortIncEnc;
	uint32_t	PinA;
	uint32_t	PinB;
	uint32_t	PinZ;
	uint8_t		MultiFuncValue;
	GPIO_T	    *pPortWireCtrl;
	uint32_t	PinWireCtrl;
} MH_QeiRegConfig_t;

 



 

 
int32_t MH_QeiRegInit(MH_QeiRegConfig_t *pHandle);
void MH_QeiRegDeInit(MH_QeiRegConfig_t *pHandle);
int32_t MH_QeiGetCount(MH_QeiRegConfig_t *pHandle);
int32_t MH_QeiGetCntLatch(MH_QeiRegConfig_t *pHandle);



 
#line 37 "..\\Source\\McStack\\incremental_fdbk.h"

 

 

typedef struct{
	MH_QeiRegConfig_t   QeiRegister;
	uint32_t            AlignRequest;
} MS_IncremEnc_Handle_t;
 

 

 
void MS_MapAbzBlkInit(void);
void MS_GetAbzCount(MS_IncremEnc_Handle_t *pHandle);


 
#line 42 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\stepper_mode.h"




























 




 
#line 36 "..\\Source\\McStack\\stepper_mode.h"

 




 
typedef enum
{
	STPM_IDLE		        = 0,
	STPM_START			    = 1,
	STPM_FORWARD		    = 10,
	STPM_BACKWARD		    = 20,
	STPM_ZERO_VECTOR	    = 30,
	STPM_DONE			    = 99,
} STPM_STATE_E;

typedef enum
{
	STPMVA_IDLE = 0,
	STPMVA_SETUP,
	STPMVA_START,
	STPMVA_WAIT_ALIGN,
	STPMVA_DONE,
	STPMVA_FAIL,
} STPM_VECALI_STATE_E;

typedef struct
{
	 
	STPM_STATE_E   			State;
	volatile uint32_t       TickMs[3];
	int32_t				    IsSingleTurn;
	int32_t				    StpAng;

	 
	STPM_VECALI_STATE_E	    VecAliState;
	int32_t 			    RotorAngT[2];
	int32_t 			    RotorStableCnt;
} STPM_CTRL_T;

 

 

 
MC_STATUS_E STPM_Init(STPM_CTRL_T *pStpMod);
void STPM_TickRun(STPM_CTRL_T *pStpMod);
STPM_STATE_E STPM_RunStepping(STPM_CTRL_T *pStpMod);
STPM_VECALI_STATE_E STPM_VectorAlignment(STPM_CTRL_T *pStpMod);



 
#line 43 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\flash.h"




























 




 
#line 36 "..\\Source\\McStack\\flash.h"

 



 
typedef struct
{
	uint8_t	  Signature[4];
	uint16_t	ImageType;
	uint16_t	Compress;
	uint32_t	ContentOffset;
	uint32_t	FileLen;
	uint32_t	Reserved0;
	uint32_t	Checksum32;	
	uint32_t	Reserved1[2];
} MS_Firmware_Header_t;

typedef struct{
    MS_Firmware_Header_t FwHeader;
    int32_t FileSize;
    int32_t FileSize_div_4;
    int32_t BaseAddr;
    int32_t BufferLen;
    uint8_t Buffer[256];
    uint32_t CalcChecksum32;
} MS_Flash_Handle_t;
 

 

 
void MS_GetFlashParamsAreaData(void);
static void doCommand(int last);
static int MS_GetLineData(void);
static int MS_CalcCheckSum(int base);
static int MS_CheckSignature(uint8_t *p);
static uint32_t MS_Swap(uint32_t d);
static void MS_ReadFlashRom(int addr,int n,uint8_t *buf);


 
#line 44 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\loop_control.h"




























 




 
#line 36 "..\\Source\\McStack\\loop_control.h"

 
void MS_MapCmdBlkInit(void);

#line 45 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\pid.h"




























 




 
#line 36 "..\\Source\\McStack\\pid.h"

 

 
typedef struct{
    int32_t *Cmd;
    int32_t *Feedback;
    int32_t *Kp;
    int32_t *Ki;
    int32_t KpDiv;
    int32_t KiDiv;
    int32_t *Error;
    int32_t *Integraler;
    int32_t *ErrorLimit;
    int32_t *IntegralLimit;
    int32_t *OutputLimit;
    int32_t *Division;
    int32_t *Output;
} MS_PI_Handle_t;

typedef enum{
    Pos = 0,
    Spd,
    Amp,
} Control_Loop_e;

 

 

 
void MS_MapPosBlkInit(void) ;
void MS_MapSpdBlkInit(void);
void MS_MapAmpBlkInit(void) ;
void MS_PiController(MS_PI_Handle_t *pHandle, int32_t type) ;



 

#line 46 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\servo_commu.h"




























 




 
#line 36 "..\\Source\\McStack\\servo_commu.h"
#line 1 "..\\Source\\Mchal\\McHal_husb.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_husb.h"

 





 
#line 50 "..\\Source\\Mchal\\McHal_husb.h"

 
#line 60 "..\\Source\\Mchal\\McHal_husb.h"

 









#line 78 "..\\Source\\Mchal\\McHal_husb.h"

#line 87 "..\\Source\\Mchal\\McHal_husb.h"





 
typedef enum
{
	VCPSTE_NOT_INIT           = -1,	
	VCPSTE_RECV_CMD           = 0,
	VCPSTE_PARSE_CMD          = 1,
	VCPSTE_EXEC_CMD           = 2,
	VCPSTE_PREPARE_END_RESP   = 9,	
	VCPSTE_WAIT_END_RESP      = 10,	
} MH_VCP_STATE_E;

typedef struct
{
	int flgRxReady; 
	int flgTxReady;
	int RxPnt; 
	int TxPnt;
	int RxLength; 
	int TxLength;
	uint8_t RxBuffer[16384];
	uint8_t TxBuffer[4096];
} MH_VCP_OBJECT_T;

 

 

 
void MH_VCP_Init(void);
void MH_VCP_Run(void);



 
#line 37 "..\\Source\\McStack\\servo_commu.h"

 
typedef enum
{
	SC_MAP_CMD    = 2,
	SC_DAS_CMD,
} SC_CMD_TYPE_E;

 
typedef struct
{    
	int  code; 
	char name[8];
} cmd_t;
 

 
extern int mapMemory[];

 
int esc_doCommand(char *,int,char *,char *,int,int);


 
#line 47 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\sw_protection.h"




























 




 
#line 36 "..\\Source\\McStack\\sw_protection.h"
#line 1 "..\\Source\\Mchal\\McHal_gpio.h"




























 




 
#line 36 "..\\Source\\Mchal\\McHal_gpio.h"

 

 
typedef struct {
	GPIO_T		*pPortGateDriveEnb;
	uint32_t	PinGateDriveEnb;
	GPIO_T		*pPortPwmWdtTrg;
	uint32_t	PinPwmWdtTrg;
	GPIO_T		*pPortMotorBrake;	
	uint32_t	PinMotorBrake;	
} MH_SwProtRegConfig_t;

 

 

 
int32_t MH_SwProtRegInit(MH_SwProtRegConfig_t *pHandle);
void MH_SwProtRegDeInit(MH_SwProtRegConfig_t *pHandle);
void MH_DebugHwInit(void);


 
#line 37 "..\\Source\\McStack\\sw_protection.h"
#line 38 "..\\Source\\McStack\\sw_protection.h"
#line 39 "..\\Source\\McStack\\sw_protection.h"
#line 40 "..\\Source\\McStack\\sw_protection.h"

 

 
typedef enum
{
	SAT_NO_ERR			    = 0,
	SAT_UNDER_VOLT			= 1,
	SAT_OVER_VOLT		    = 2,
	SAT_OVER_TEMP		    = 3,
	SAT_PHASE_ERR       	= 4,
	SAT_OVER_CURR_ID		= 5,
	SAT_OVER_CURR_IQ		= 6,
} SPT_ALARM_TYPE_E;

typedef struct {
	MH_SwProtRegConfig_t    SwProtRegister;
	MS_Vbus_Handle_t        *pVbusInfo;
	MS_Temp_Handle_t        *pTempInfo;
	MS_Curr_Handle_t        *pCurrInfo;
} MS_SwProt_Handle_t;

 

 

 
void MS_MapProtectBlkInit(void);
void MS_GetAvBusVoltage_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvTemp_V(MS_SwProt_Handle_t *pHandle);
void MS_GetAvI2T_V(MS_SwProt_Handle_t *pHandle);
void SWP_Run(MS_SwProt_Handle_t *pHandle);
void SWP_DriveCtrl(MS_SwProt_Handle_t *pHandle);



 
#line 48 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\trajectory_ctrl.h"




























 




 
#line 36 "..\\Source\\McStack\\trajectory_ctrl.h"

 

 
typedef enum
{
	TCM_START		= 0,
	TCM_RUN  		= 1,
	TCM_GET_SPD	= 2,	
} TC_MODE_E;

typedef enum
{
	TCC_NO_CMD		= 0,
	TCC_ABS_POS   = 1,
	TCC_REL_POS		= 2,	
	TCC_RESTART   = 3,	
	TCC_BRAKE    	= 9,
} TC_COMMAND_E;

typedef enum
{
	TCS_ADJ_SPD 	= 0,
	TCS_ACC_SPD   = 1,
	TCS_CONST_SPD = 2,	
	TCS_DEC_SPD   = 3,	
	TCS_STOP    	= 9,
} TC_STATE_E;

typedef enum
{
	TCD_FORWARD 	= 1,
	TCD_BACKWARD  = -1,
} TC_DIRECTION_E;

 

 

 
void TC_Init(void);  
void TC_Run(void);
void TC_Brake(void);



 

#line 49 "..\\Source\\McInterface\\mc.h"
#line 1 "..\\Source\\McStack\\das.h"




























 




 
#line 36 "..\\Source\\McStack\\das.h"

 





 
typedef enum
{
	DASSTE_IDLE    = 0,
	DASSTE_START,
	DASSTE_RUN,
	DASSTE_DONE,	
} DAS_STATE_E;

 

 
extern int dasMemory[];

 
void DAS_Init(void);
DAS_STATE_E DAS_Run(void);



 
#line 50 "..\\Source\\McInterface\\mc.h"

 
#line 58 "..\\Source\\McInterface\\mc.h"

 
typedef enum
{
	MCSTE_IDLE = 0,
	MCSTE_TUNE,
	MCSTE_FAULT,
	MCSTE_WAIT_CLR_ERR,
	MCSTE_STOP,
	MCSTE_START_VEC_ALIGN,
	MCSTE_VEC_ALIGN,
	MCSTE_START_ENC_ALIGN,
	MCSTE_ENC_ALIGN,
	MCSTE_STARTUP,
	MCSTE_RUN,
	MCSTE_SHUTDOWN,
} MC_STATE_E;

typedef enum
{
	MCCMD_NO_CMD = 0,
	MCCMD_START,
	MCCMD_ENC_ALIGN,
	MCCMD_SET_POS,
	MCCMD_STOP,
	MCCMD_FAULT_RESET,
} MC_CMD_E;

typedef struct
{

	MC_STATE_E   				State;
	volatile MC_CMD_E		    Cmd;
	MC_FAULT_E   				FaultFlags;
	volatile uint32_t           TickMs[3];

	 
	STPM_CTRL_T					StpMod;
	 
	int32_t					    HomingReach;
	int32_t					    HomingVelocity;
	 
	int32_t					    VelocityCmd;
	 
	int32_t					    PositionCmd;
	uint32_t				    ConvFactor_mDegToTick;
} MC_T;

 
typedef struct
{
    MS_Vbus_Handle_t VbusInfo;
} MI_Vbus_Handle_t;

typedef struct
{
    MS_Temp_Handle_t TempInfo;
} MI_Temp_Handle_t;

typedef struct
{
    MS_Curr_Handle_t    CurrInfo;
    MCMATH_AB_PHASE_T   Iab;
} MI_Curr_Handle_t;

typedef struct
{
    MS_IncremEnc_Handle_t IncremEncInfo;
} MI_IncremEnc_Handle_t;

typedef struct
{
    MS_Epwm_Handle_t PwmInfo;
    MCMATH_AB_PHASE_T Vab;
} MI_Pwm_Handle_t;

typedef struct
{
    MS_SwProt_Handle_t SwProtInfo;
} MI_SwProt_Handle_t;

typedef struct
{
    MS_PI_Handle_t PiInfo;
} MI_Pi_Handle_t ;

 

 

 
extern MC_T MotorCtrl;
extern MI_Vbus_Handle_t BusVoltageParamsM1;
extern MI_Temp_Handle_t TemperatureParamsM1;
extern MI_Curr_Handle_t CurrentParamsM1;
extern MI_IncremEnc_Handle_t IncremEncParamsM1;
extern MI_Pwm_Handle_t PwmParamsM1;
extern MI_SwProt_Handle_t SwProtParamsM1;
extern MI_Pi_Handle_t PosParamsM1;
extern MI_Pi_Handle_t SpdParamsM1;
extern MI_Pi_Handle_t AmpParamsM1;
extern MI_Pi_Handle_t AmpParamsM2;

void MI_IncremEncModuleInit(void);
void MI_PwmModuleInit(void);
void MI_AbsoluteEncModuleInit(void);
void MI_SwProtModuleInit(void);

 
	 
int32_t  		MC_Init(void);
void     		MC_DeInit(void);
void     		MC_TickRun(void);
MC_STATE_E	MC_TaskRun(void);
void        MC_StartMotor(void);
void        MC_StopMotor(void);
MC_STATE_E  MC_GetState(void);
	 
int32_t     MC_GetActualTorque(void);
	 
void        MC_SetTargetVelocity(int32_t Velocity);
int32_t     MC_GetActualVelocity(void);
	 
void        MC_ClearActualPosition(void);
void				MC_SetTargetPosition(int32_t Position, int32_t Velocity);
int32_t     MC_GetActualPosition(void);
	 
void        MC_FaultReset(void);
MC_FAULT_E	MC_GetFaultStatus(void);
	 
void				MC_StartHoming(int32_t TargetVelocity);
int32_t 		MC_CheckHomingDone(void);



 
#line 58 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
#line 1 "..\\Source\\CiA402\\cia402.h"




























 




 
#line 36 "..\\Source\\CiA402\\cia402.h"

 


 
typedef enum{
	CIA402_EMUL_DISABLED = 0,
	CIA402_ACTUAL_IS_TARGET_POS,
} CIA402_EMUL_MODE;

typedef union {
  uint32_t d32;
  struct{
    uint32_t pp:               1;   
    uint32_t vl:               1;   
    uint32_t pv:               1;   
    uint32_t tq:               1;   		
    uint32_t reserved_4:       1;   				
    uint32_t hm:               1;   						
    uint32_t ip:               1;   
    uint32_t csp:              1;   
    uint32_t csv:              1;   		
    uint32_t cst:              1;   				
    uint32_t cstca:            1;   						
    uint32_t reserved_31_11:  21;   
  } b;
} OBJ0x6502_SUPPORTED_DRIVE_MODES;

typedef union {
  uint32_t d32;
  struct{
    uint32_t axis_is_act:      1;   
    uint32_t brake_applied:    1;   
    uint32_t low_pwr_applied:  1;   
    uint32_t high_pwr_applied: 1;   		
    uint32_t axis_func_enb:    1;   				
    uint32_t cfg_allowed:      1;   						
    uint32_t reserved_31_6:   26;   
  } b;
} CIA402_FLAGS;

typedef struct{
	void *pCtrlObj;
  int32_t (*FuncStart)(void *pAxis);
  int32_t (*FuncStop)(void *pAxis);
  int32_t (*FuncRun)(void *pAxis);		
} CIA402_FUNC_INTF;

 
  
int32_t CIA402_Init(void);
int32_t CIA402_DeInit(void);
int32_t CIA402_EmulationControl(uint8_t mode);
uint8_t CIA402_CheckEmulationEnable(void);
uint8_t CIA402_GetHallStatus(void);



 
#line 59 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"





 


 
#line 76 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 89 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 103 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 116 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 
#line 129 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"




 

















 
#line 240 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"

#line 269 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"

#line 311 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"






#line 329 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"
	






 










 












#line 1 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 




 







 








 




 
#line 61 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
unsigned long SI4;  
unsigned long SI5;  
} 
TOBJ1600;




 
extern TOBJ1600 RxPdoMappingCspCsv0x1600



;
 





 




 
#line 120 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1601;




 
extern TOBJ1601 RxPdoMappingCsp0x1601



;
 





 




 
#line 177 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1602;




 
extern TOBJ1602 RxPdoMappingCsv0x1602



;
 





 




 
#line 240 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
unsigned long SI4;  
unsigned long SI5;  
} 
TOBJ1A00;




 
extern TOBJ1A00 TxPdoMappingCspCsv0x1A00



;
 





 




 
#line 299 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1A01;




 
extern TOBJ1A01 TxPdoMappingCsp0x1A01



;
 





 




 
#line 356 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SI1;  
unsigned long SI2;  
unsigned long SI3;  
} 
TOBJ1A02;




 
extern TOBJ1A02 TxPdoMappingCsv0x1A02



;
 





 




 
#line 407 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned short aEntries[1];   
} 
TOBJ1C12;




 
extern TOBJ1C12 sRxPDOassign



;
 





 




 
#line 456 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned short aEntries[1];   
} 
TOBJ1C13;




 
extern TOBJ1C13 sTxPDOassign



;
 





 




 
#line 498 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short ErrorCode0x603F



;
 





 




 
#line 529 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short Controlword0x6040



;
 





 




 
#line 560 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned short Statusword0x6041



;
 





 




 
#line 591 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long VlRampFunctionTime0x604F



;
 





 




 
#line 622 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short QuickStopOptionCode0x605A



;
 





 




 
#line 653 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short ShutdownOptionCode0x605B



;
 





 




 
#line 684 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short DisableOperationOptionCode0x605C



;
 





 




 
#line 715 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern short FaultReactionOptionCode0x605E



;
 





 




 
#line 746 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char ModesOfOperation0x6060



;
 





 




 
#line 777 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char ModesOfOperationDisplay0x6061



;
 





 




 
#line 808 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long PositionActualValue0x6064



;
 





 




 
#line 839 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long VelocityActualValue0x606C



;
 





 




 
#line 870 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TorqueActualValue0x6077



;
 





 




 
#line 901 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TargetPosition0x607A



;
 





 




 
#line 932 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long HomeOffset0x607C



;
 





 




 
#line 973 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
long MinPositionLimit;  
long MaxPositionLimit;  
} 
TOBJ607D;




 
extern TOBJ607D SoftwarePositionLimit0x607D



;
 





 




 
#line 1016 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long QuickStopDeceleration0x6085



;
 





 




 
#line 1047 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern char HomingMethod0x6098



;
 





 




 
#line 1088 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned long SpeedDuringSearchForSwitch;  
unsigned long SpeedDuringSearchForZero;  
} 
TOBJ6099;




 
extern TOBJ6099 HomingSpeed0x6099



;
 





 




 
#line 1131 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long HomingAcceleration0x609A



;
 





 




 
#line 1172 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned char InterpolationTimePeriodValue;  
char InterpolationTimeIndex;  
} 
TOBJ60C2;




 
extern TOBJ60C2 InterpolationTimePeriod0x60C2



;
 





 




 
#line 1225 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
char SupportHomingMethod1st;  
char SupportHomingMethod2st;  
} 
TOBJ60E3;




 
extern TOBJ60E3 SupportedHomingMode0x60E3



;
 





 




 
#line 1268 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long DigitalInputs0x60FD



;
 





 




 
#line 1299 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern long TargetVelocity0x60FF



;
 





 




 
#line 1330 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"



 
extern unsigned long SupportedDriveModes0x6502



;
 





 




 
#line 1371 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short u16SubIndex0;
unsigned short IndexDistance;  
unsigned short MaximumNumberOfModules;  
} 
TOBJF000;




 
extern TOBJF000 ModularDeviceProfile0xF000



;
 





 




 
#line 1421 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"




 
typedef struct  {
unsigned short   u16SubIndex0;   
unsigned long aEntries[1];   
} 
TOBJF010;




 
extern TOBJF010 ModuleProfileList0xF010



;
 







#line 1524 "..\\For_SSC_Tool\\Src\\AX58200_MotorControlObjects.h"

 
#line 361 "..\\For_SSC_Tool\\Src\\AX58200_MotorControl.h"










#pragma pack(push, 2)
typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
	long  ObjVelocityActualValue;
	short  ObjModesOfOperationDisplay;
}
TCiA402PDO1A00;

typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
}
TCiA402PDO1A01;

typedef struct 
{
	unsigned short ObjStatusWord;
	long  ObjPositionActualValue;
}
TCiA402PDO1A02;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetPosition;
	long  ObjTargetVelocity;
	short  ObjModesOfOperation;
}
TCiA402PDO1600;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetPosition;
}
TCiA402PDO1601;

typedef struct 
{
	unsigned short ObjControlWord;
	long  ObjTargetVelocity;
}
TCiA402PDO1602;
#pragma pack(pop)

typedef struct 
{
	TOBJ1600 *pRxPdoMappingCspCsv0x1600;
	TOBJ1601 *pRxPdoMappingCsp0x1601;
	TOBJ1602 *pRxPdoMappingCsv0x1602;
	TOBJ1A00 *pTxPdoMappingCspCsv0x1A00;
	TOBJ1A01 *pTxPdoMappingCsp0x1A01;
	TOBJ1A02 *pTxPdoMappingCsv0x1A02;
	TOBJ1C12 *psRxPDOassign;
	TOBJ1C13 *psTxPDOassign;
	unsigned short   *pErrorCode0x603F;
	unsigned short   *pControlword0x6040;
	unsigned short   *pStatusword0x6041;

	unsigned long	 *pVlRampFunctionTime0x604F;

	short    *pQuickStopOptionCode0x605A;
	short    *pShutdownOptionCode0x605B;
	short    *pDisableOperationOptionCode0x605C;
	short    *pFaultReactionOptionCode0x605E;
	char     *pModesOfOperation0x6060;
	char     *pModesOfOperationDisplay0x6061;
	long    *pPositionActualValue0x6064;
	long    *pVelocityActualValue0x606C;
	long    *pTorqueActualValue0x6077;
	long    *pTargetPosition0x607A;

	long    *pHomeOffset0x607C;
	char     *pHomingMethod0x6098;
	TOBJ6099 *pHomingSpeed0x6099;
	unsigned long   *pHomingAcceleration0x609A;
	TOBJ60E3 *pSupportedHomingMode0x60E3;
	unsigned long   *pDigitalInputs0x60FD;

	TOBJ607D *pSoftwarePositionLimit0x607D;
	unsigned long   *pQuickStopDeceleration0x6085;
	TOBJ60C2 *pInterpolationTimePeriod0x60C2;
	long    *pTargetVelocity0x60FF;
	unsigned long   *pSupportedDriveModes0x6502;
	TOBJF000 *pModularDeviceProfile0xF000;
	TOBJF010 *pModuleProfileList0xF010;
}
CiA402_OBJ_T;

typedef struct 
{
	unsigned char             bAxisIsActive;  
	unsigned char             bBrakeApplied;  
	unsigned char             bLowLevelPowerApplied;  
	unsigned char             bHighLevelPowerApplied;  
	unsigned char             bAxisFunctionEnabled;  
	unsigned char             bConfigurationAllowed;  

	unsigned short           i16State;  
	unsigned short           u16PendingOptionCode;  
	double           fCurPosition;  
	unsigned long           u32CycleTime;  
	
	CiA402_OBJ_T     CiA402Var;
	TOBJECT   *pObjDic;

	long						objTargetVelocityBackup;

	CIA402_FUNC_INTF *pFuncIntf;
	CIA402_FLAGS     flags;
	unsigned char             bHighLevelPowerAppliedPreStatus;
	unsigned char             bAxisFunctionEnabledPreStatus;
	unsigned char             EmulationEnable;
	long            TargetVelocity;	
}
CiA402_AXIS_T;

extern void CiA402_Init(void);
extern void CiA402_LocalError(unsigned short ErrorCode);
extern void CiA402_StateMachine(CiA402_AXIS_T *pAxis);
extern void CiA402_DeInit(void);

extern void APPL_Application(void);




extern void   APPL_AckErrorInd(unsigned short stateTrans);
extern unsigned short APPL_StartMailboxHandler(void);
extern unsigned short APPL_StopMailboxHandler(void);
extern unsigned short APPL_StartInputHandler(unsigned short *pIntMask);
extern unsigned short APPL_StopInputHandler(void);
extern unsigned short APPL_StartOutputHandler(void);
extern unsigned short APPL_StopOutputHandler(void);

extern unsigned short APPL_GenerateMapping(unsigned short *pInputSize,unsigned short *pOutputSize);
extern void APPL_InputMapping(unsigned short* pData);
extern void APPL_OutputMapping(unsigned short* pData);


 

#line 209 "..\\For_SSC_Tool\\Src\\ecatslv.c"





 






 
unsigned short    u16ALEventMask;                      

 
    volatile unsigned char u8dummy;


        volatile unsigned char SMActivate = 0;

TSYNCMAN		SyncManInfo;


unsigned char EepromLoaded = 0;




 

void ResetALEventMask(unsigned short intMask);






 
void ResetALEventMask(unsigned short intMask)
{
    unsigned short mask;
    HW_EscRead(((unsigned long *)&(mask)),((unsigned short)(0x0204)),2);
    
    mask &= intMask;


    ;


    HW_EscWrite(((unsigned long *)&(mask)),((unsigned short)(0x0204)),2);
    ;
}






 
void SetALEventMask(unsigned short intMask)
{
    unsigned short mask;
    HW_EscRead(((unsigned long *)&(mask)),((unsigned short)(0x0204)),2);
    

    mask |= intMask;

    ;


    HW_EscWrite(((unsigned long *)&(mask)),((unsigned short)(0x0204)),2);
    ;
}






 
void UpdateEEPROMLoadedState(void)
{
    unsigned short TmpVar = 0;
    
    HW_EscRead(((unsigned long *)&(TmpVar)),((unsigned short)(0x0502)),2);
    TmpVar = (TmpVar);


    if (((TmpVar & 0x0800) > 0)
        || ((TmpVar & 0x1000) > 0))
    {
        EepromLoaded = 0;
    }
    else
    {
        EepromLoaded = 1;
    }
}






 










 

TSYNCMAN  * GetSyncMan( unsigned char channel )
{
    HW_EscRead((unsigned long *)&SyncManInfo, 0x0800 + (channel * 8), 8 );



    return &SyncManInfo;
}






 
void DisableSyncManChannel(unsigned char channel)
{
    unsigned short Offset;
    volatile unsigned char smStatus = 0x01;
    Offset = (0x0807 + (8*channel));

    HW_EscWrite(((unsigned long *)&(smStatus)),((unsigned short)(Offset)),1);
    
     
    do
    {
        HW_EscRead(((unsigned long *)&(smStatus)),((unsigned short)(Offset)),1);
    }while(!(smStatus & 0x01));
}






 
void EnableSyncManChannel(unsigned char channel)
{
    unsigned short Offset;
    volatile unsigned char smStatus = 0x00;
    Offset = (0x0807 + (8*channel));

    HW_EscWrite(((unsigned long *)&(smStatus)),((unsigned short)(Offset)),1);
    
     
    do
    {
        HW_EscRead(((unsigned long *)&(smStatus)),((unsigned short)(Offset)),1);
    }while((smStatus & 0x01));
}











 

unsigned char    CheckSmSettings(unsigned char maxChannel)
{
    unsigned char i;
    unsigned char result = 0;
    TSYNCMAN  *pSyncMan;
    unsigned short SMLength = 0;
    unsigned short SMAddress = 0;


        
        if ((nMaxEscAddress < 0x2FFF)
            || (nMaxEscAddress < 0x2FFF)
            || (nMaxEscAddress < 0x2FFF)
            || (nMaxEscAddress < 0x2FFF))
        {
            
 


                return 0x0014;
        }

     
    pSyncMan = GetSyncMan(0);

    SMLength = pSyncMan->Length;
    SMAddress = pSyncMan->PhysicalStartAddress;



    if (!(pSyncMan->Settings[2] & 0x01))
    {
         
        result = 0x0016;
    }
    else if ((pSyncMan->Settings[0] & 0x0C) != 0x04)
    {
         
        result = 0x0016;
    }
    else if ((pSyncMan->Settings[0] & 0x02) != 0x02)
    {
         
        result = 0x0016;
    }
    else if (SMLength < 0x0024)
    {
         
        result = 0x0016;
    }
    else if (SMLength > 0x0080)
    {
         
        result = 0x0016;
    }
    else if (SMAddress < 0x1000)
    {
         
        result = 0x0016;
    }
    else if (SMAddress > 0x2FFF)
    {
         
        result = 0x0016;
    }


    if ( result == 0 )
    {
         
        pSyncMan = GetSyncMan(1);

    SMLength = pSyncMan->Length;
    SMAddress = pSyncMan->PhysicalStartAddress;


    if (!(pSyncMan->Settings[2] & 0x01))
    {
         
        result = 0x0016;
    }
    else if ((pSyncMan->Settings[0] & 0x0C) != 0x00)
    {
         
        result = 0x0016;
    }
    else if ((pSyncMan->Settings[0] & 0x02) != 0x02)
    {
         
        result = 0x0016;
    }
    else if (SMLength < 0x0024)
    {
         
        result = 0x0016;
    }
    else if (SMLength > 0x0080)
    {
         
        result = 0x0016;
    }
    else if (SMAddress < 0x1000)
    {
         
        result = 0x0016;
    }
    else if (SMAddress > 0x2FFF)
    {
         
        result = 0x0016;
    }
    }

    if ( result == 0 && maxChannel > 3 )
    {
         
        b3BufferMode = 1;
         
        pSyncMan = GetSyncMan(3);

    SMLength = pSyncMan->Length;
    SMAddress = pSyncMan->PhysicalStartAddress;



    if ((pSyncMan->Settings[2] & 0x01) != 0 && SMLength == 0)
    {
         
        result = 0x03 + 1;
    }
        else if (pSyncMan->Settings[2] & 0x01)
        {
             
                if (SMLength != nPdInputSize || nPdInputSize == 0 || SMLength > 0x0044)
                {
                     
                    result = 0x02 + 1;
                }
                else
                {
                     
                    if ((pSyncMan->Settings[0] & 0x0C) == 0x00)
                    {
                         
                        if (((nAlStatus == ((unsigned char) 0x02)) && (SMAddress >= 0x1000) && (SMAddress <= 0x2FFF))
                            || ((nAlStatus != ((unsigned char) 0x02)) && (SMAddress == nEscAddrInputData))
                            )
                        {
                             

                                if ((pSyncMan->Settings[0] & 0x02) == 0x02)
                                {
                                     
                                    b3BufferMode = 0;
                                }
                        }
                        else
                        {
                             
                            result = 0x01 + 1;
                        }
                    }
                    else
                    {
                         
                        result = 0x03 + 1;
                    }
                }
        }
        else if (SMLength != 0 || nPdInputSize != 0)
        {
             
            result = 0x02 + 1;
        }



        if ( result != 0 )
        {
            result = 0x001E;
        }
    }



    if (result == 0 && maxChannel > 2)
    {
         
        pSyncMan = GetSyncMan(2);

    SMLength = pSyncMan->Length;
    SMAddress = pSyncMan->PhysicalStartAddress;



    if ((pSyncMan->Settings[2] & 0x01) != 0 && SMLength == 0)
    {
         
        result = 0x03 + 1;
    }
        else if (pSyncMan->Settings[2] & 0x01)
        {
             
            if ( SMLength == nPdOutputSize && nPdOutputSize != 0 && SMLength <= ((unsigned short)0x044))
            {
                 
                if ( (pSyncMan->Settings[0] & 0x0C) == 0x04 )
                {
                     
                    if ( ( ( nAlStatus == ((unsigned char) 0x02) )&&( SMAddress >= 0x1000 )&&( SMAddress <= 0x2FFF ) )
                       ||( ( nAlStatus != ((unsigned char) 0x02) )&&( SMAddress == nEscAddrOutputData ) )
                        )
                    {
                         
                        {
                             
                            if (pSyncMan->Settings[0] & 0x40)
                            {
                                bWdTrigger = 1;
                            }
                            else
                            {
                                bWdTrigger = 0;
                            }

                            if ((pSyncMan->Settings[0] & 0x02) == 0x02)
                            {
                                 
                                b3BufferMode = 0;
                                }
                        }
                    }
                    else
                    {
                         
                        result = 0x01 + 1;
                    }
                }
                else
                {
                     
                    result = 0x03 + 1;
                }
            }
            else
            {
                 
                result = 0x02 + 1;
            }
        }
        else if (SMLength != 0 || nPdOutputSize != 0)
        {
             
            result = 0x02 + 1;
        }

        if ( result != 0 )
        {
            result = 0x001D;
        }
    }


    if ( result == 0 )
    {
         
        for (i = maxChannel; i < nMaxSyncMan; i++)
        {
            pSyncMan = GetSyncMan(i);
            SMActivate = pSyncMan->Settings[2];
        }
    }
    return result;
}











 

unsigned short StartInputHandler(void)
{
    TSYNCMAN  * pSyncMan;
    unsigned char        dcControl;

    unsigned short     wdiv = 0;
    unsigned short     wd = 0;
    unsigned long     cycleTimeSync0 = 0;  
    unsigned long     shiftTimeSync1 = 0;  
    unsigned char bSubordinatedCycles = 0;

    unsigned short    nPdInputBuffer = 3;

    unsigned short    nPdOutputBuffer = 3;

    unsigned short SyncType0x1C32 = 0;  
    unsigned short SyncType0x1C33 = 0;  

    unsigned short u16MinSuppSyncType = 0xFFFF;   

    u16MinSuppSyncType &= sSyncManOutPar.u16SyncTypesSupported;
    u16MinSuppSyncType &= sSyncManInPar.u16SyncTypesSupported;

    u16ALEventMask = 0;


    

 
    bEcatFirstOutputsReceived = 0;

     
    pSyncMan = GetSyncMan(2);
     
    nEscAddrOutputData = pSyncMan->PhysicalStartAddress;
     
    if (pSyncMan->Settings[0] & 0x02)
    {
       nPdOutputBuffer = 1;
    }


     
    pSyncMan = GetSyncMan(3);
     
    nEscAddrInputData = pSyncMan->PhysicalStartAddress;


     
    if (pSyncMan->Settings[0] & 0x02)
    {
        nPdInputBuffer = 1;
    }
    
 

    if (((nEscAddrInputData + nPdInputSize * nPdInputBuffer) > u16EscAddrSendMbx && (nEscAddrInputData < (u16EscAddrSendMbx + u16SendMbxSize)))
       || ((nEscAddrInputData + nPdInputSize * nPdInputBuffer) > u16EscAddrReceiveMbx && (nEscAddrInputData < (u16EscAddrReceiveMbx + u16ReceiveMbxSize)))
        )
    {
        return 0x001E;
    }

    if (
        ((nEscAddrOutputData + nPdOutputSize * nPdOutputBuffer) > u16EscAddrSendMbx && (nEscAddrOutputData < (u16EscAddrSendMbx + u16SendMbxSize)))
        ||((nEscAddrOutputData + nPdOutputSize * nPdOutputBuffer) > u16EscAddrReceiveMbx && (nEscAddrOutputData < (u16EscAddrReceiveMbx + u16ReceiveMbxSize)))
        ||
        ((nEscAddrOutputData + nPdOutputSize * nPdOutputBuffer) > nEscAddrInputData && (nEscAddrOutputData < (nEscAddrInputData + nPdInputSize)))
        )
    {

        
 
        return 0x001D;
    }

    

 

     
     
    HW_EscRead(((unsigned long *)&(dcControl)),((unsigned short)(0x0981)),1);

    
        HW_EscRead(((unsigned long *)&(cycleTimeSync0)),((unsigned short)(0x09A0)),4);
        cycleTimeSync0 = (cycleTimeSync0);

    
        HW_EscRead(((unsigned long *)&(shiftTimeSync1)),((unsigned short)(0x09A4)),4);
        shiftTimeSync1 = (shiftTimeSync1);


    SyncType0x1C32 = sSyncManOutPar.u16SyncType;
    SyncType0x1C33 = sSyncManInPar.u16SyncType;



    



 
    if((dcControl & (0x01 | 0x08)) != 0)
    {
         
        if((dcControl & (0x02 | 0x04)) == 0)
        {
            return 0x0030;
        }

         
        if(((dcControl & 0x02) == 0)
            && ((dcControl & 0x04) != 0))
        {
            return 0x0030;
        }

        if(u16MinSuppSyncType != 0)
        {
            if((((u16MinSuppSyncType & 0x0004) == 0) && ((dcControl & 0x02) != 0))
                ||(((u16MinSuppSyncType & 0x0008) == 0) && ((dcControl & 0x04) != 0)))
            {
                 
                return 0x0030;                   
    }
        }

        {
 
            unsigned long curMinCycleTime = 0x00030D40;
            curMinCycleTime = sSyncManOutPar.u32MinCycleTime;

             
            if (cycleTimeSync0 != 0 && (cycleTimeSync0 < curMinCycleTime || cycleTimeSync0 > 0xC3500000))
            {
                    return 0x0036;
            }
 
        }


         
        if(((dcControl & 0x02) != 0) && ((dcControl & 0x04) != 0))
        {
             
            if((shiftTimeSync1 > 0) && (shiftTimeSync1 >= cycleTimeSync0))
            {
                bSubordinatedCycles = 1;
            }
        }

         
        if(bSubordinatedCycles && ((u16MinSuppSyncType & 0x0010) == 0))
        {
             return 0x0030;
        }
    }


    

 
    if(bSyncSetByUser)
    {
        if((dcControl & (0x01 | 0x08)) == 0)
        {
             
            if((SyncType0x1C32 == 0x0002) || (SyncType0x1C32 == 0x0003)
                ||(SyncType0x1C33 == 0x0002) || (SyncType0x1C33 == 0x0003))
            {
                return 0x0030;
            }
        } 
    else
    {
            if((dcControl & 0x04) == 0)
            {
                 
                if((SyncType0x1C32 == (unsigned short)0x0003)
                    ||(SyncType0x1C33 == (unsigned short)0x0003))
                {
                    return 0x0030;
                }
            } 

            if((dcControl & 0x02) == 0)
            {
                 
                if((SyncType0x1C32 == (unsigned short)0x0002)
                    ||(SyncType0x1C33 == (unsigned short)0x0002))
                {
                    return 0x0030;
                }
            } 

        }
    } 
    else
    {
         
        if((dcControl & (0x01 | 0x08)) == 0)
        {
             

             
            if (nPdOutputSize > 0)
            {
                SyncType0x1C32 = 0x0001;
                
                if (nPdInputSize > 0)
                {
                    SyncType0x1C33 = 0x0022;
                }
                else
                {
                    SyncType0x1C33 = 0x0000;
                }
            }
            else if (nPdInputSize > 0)
            {
                SyncType0x1C32 = 0x0000;
                SyncType0x1C33 = 0x0001;
            }
            else
            {
                SyncType0x1C32 = 0x0000;
                SyncType0x1C33 = 0x0000;
            }

        }
        else
        {
            if (nPdOutputSize > 0)
            {
                 
                if (bSubordinatedCycles)
                {
                    SyncType0x1C32 = 0x0003;
                }
                else
                {
                    SyncType0x1C32 = 0x0002;
                }
            }
            else
            {
                SyncType0x1C32 = 0x0000;
            }


            if (nPdInputSize > 0)
            {
                if ((dcControl & 0x04) != 0)
                {
                     
                    SyncType0x1C33 = 0x0003;
                }
                else
                {
                     
                    SyncType0x1C33 = 0x0002;
                }
            }
            else
            {
                SyncType0x1C33 = 0x0000;
            }
        }
    }

     
    if(SyncType0x1C32 == 0x0003)
    {
 
        sSyncManOutPar.u32Sync0CycleTime = (unsigned long)cycleTimeSync0;
        sSyncManOutPar.u32CycleTime = (unsigned long)cycleTimeSync0;

        sSyncManInPar.u32Sync0CycleTime = (unsigned long)cycleTimeSync0;
        sSyncManInPar.u32CycleTime = (unsigned long)cycleTimeSync0;
    }
    else if(SyncType0x1C32 == 0x0002)
    {
        sSyncManOutPar.u32Sync0CycleTime = (unsigned long)cycleTimeSync0;
        sSyncManOutPar.u32CycleTime = (unsigned long)cycleTimeSync0;

        sSyncManInPar.u32Sync0CycleTime = (unsigned long)cycleTimeSync0;
        sSyncManInPar.u32CycleTime = (unsigned long)cycleTimeSync0;
 
    }

     
    if ( !b3BufferMode )
    {
         
        if (( SyncType0x1C32 == 0x0000 ) || ( SyncType0x1C33 == 0x0000 ))
        {
                return 0x0029;
        }
    }

     
        if (( SyncType0x1C32 != 0x0000 ) || ( SyncType0x1C33 != 0x0000 ))
        {
         
        bEscIntEnabled = 1;
    }

         
        if(bEscIntEnabled)
        {
            if(nPdOutputSize > 0)
            {
                u16ALEventMask = ((unsigned short) 0x0400);
            }
            else if(nPdInputSize > 0)
            {
                u16ALEventMask = ((unsigned short) 0x0800);
            }

        }

        if ((SyncType0x1C32 == 0x0002) || (SyncType0x1C32 == 0x0003)
            || (SyncType0x1C33 == 0x0002) || (SyncType0x1C33 == 0x0003)) 
        {
             
            bDcSyncActive = 1;

             
            if (nPdOutputSize == 0)
            {
               u16ALEventMask = 0;
            }
        }

    sSyncManOutPar.u16SyncType = SyncType0x1C32;
    sSyncManInPar.u16SyncType = SyncType0x1C33;

     
    LatchInputSync0Value = 0;
    LatchInputSync0Counter = 0;
    u16SmSync0Value = 0;
    u16SmSync0Counter = 0;


    if(bSubordinatedCycles == 1)
    {
 
        unsigned long cycleTimeSync1 = (shiftTimeSync1 + cycleTimeSync0);
 

         
        if(shiftTimeSync1 >= cycleTimeSync0)
        {

            u16SmSync0Value = (unsigned short)(cycleTimeSync1 / cycleTimeSync0);
            
            if((cycleTimeSync1 % cycleTimeSync0) == 0)
            {
                 
                u16SmSync0Value ++;
            }
        }
        else
        {
            u16SmSync0Value = 1;
        }

         
        LatchInputSync0Value = (unsigned short) (cycleTimeSync1 / cycleTimeSync0);

        if ((cycleTimeSync1 % cycleTimeSync0) > 0)
        {
            LatchInputSync0Value++;
        }

    }
    else 
    {
        if(SyncType0x1C32 == 0x0002)
        {
             
            u16SmSync0Value = 1;
        }   

        if(SyncType0x1C33 != 0x0003)
        {
            LatchInputSync0Value = 1;
        }
    }



     
    sCycleDiag.syncFailedCounter = 0;


    

 

     
    HW_EscRead(((unsigned long *)&(wd)),((unsigned short)(0x0420)),2);
    wd = (wd);

    if (nPdOutputSize > 0 &&  wd != 0 )
    {
     
    HW_EscRead(((unsigned long *)&(wdiv)),((unsigned short)(0x0400)),2);
    wdiv = (wdiv);
        if ( wdiv != 0 )
        {
             
            unsigned long d = wdiv+2;


            d *= wd;
             
             
            d += 24999;
            d /= 25000;
            EcatWdValue = (unsigned short) d;
        }
        else
        {
            wd = 0;
             
            EcatWdValue = 0;
        }
    }
    else
    {
         
        wdiv = 0;
        EcatWdValue = 0;
    }

    if((EcatWdValue == 0 && bWdTrigger) || (EcatWdValue != 0 && !bWdTrigger))
    {
        
 
        return 0x001F;
    }

    if ( bEscIntEnabled && nPdOutputSize != 0 )
    {
        
 
        u16ALEventMask |= ((unsigned short) 0x0400);
    }
 

    Sync0WdValue = 0;
    Sync0WdCounter = 0;
    Sync1WdCounter = 0;
    Sync1WdValue = 0;
    bDcRunning = 0;
    bSmSyncSequenceValid = 0;
    i16WaitForPllRunningTimeout = 0;

 
    sSyncManInPar.u16SmEventMissedCounter = 0;
    sSyncManInPar.u16CycleExceededCounter = 0;
    sSyncManInPar.u8SyncError = 0;


    sSyncManOutPar.u16SmEventMissedCounter = 0;
    sSyncManOutPar.u16CycleExceededCounter = 0;
    sSyncManOutPar.u8SyncError = 0;
 

     
    if ( (dcControl & 0x02) != 0 )
    {
        
 
        if(cycleTimeSync0 == 0)
        {
            Sync0WdValue = 0;
        }
        else
        {
            unsigned long Sync0Cycle = cycleTimeSync0/100000;

            if(Sync0Cycle < 5)
            {
                 
                Sync0WdValue = 1;
            }
            else
            {
                Sync0WdValue = (unsigned short)(Sync0Cycle*2)/10;
            }
        }

         
        if ( (dcControl & 0x04) != 0 )
        {
            if(shiftTimeSync1 < cycleTimeSync0)
        {
                 
                Sync1WdValue = Sync0WdValue;
        }
        else
        {
                 
 
                unsigned long Sync1Cycle = (shiftTimeSync1  + cycleTimeSync0 )/100000;
 
                if(Sync1Cycle < 5)
                {
                     
                    Sync1WdValue = 1;
                }
                else
                {
                     
                    Sync1WdValue = (unsigned short)((Sync1Cycle*2)/10);
                     
                }

                 
                Sync1WdValue += Sync0WdValue/2;
            }
    }
    }

    if(nPdOutputSize > 0)
    {
        EnableSyncManChannel(2);
    }

    if(nPdInputSize > 0)
    {
        EnableSyncManChannel(3);
    }

     
    PDO_InputMapping();

    return 0x0000;
}









 

unsigned short StartOutputHandler(void)
{
    

 
    unsigned short result = 0xFF;
    if(bLocalErrorFlag)
    {
         
        return u16LocalErrorCode;
    }
 


     
    if(bDcSyncActive)
    {
        i16WaitForPllRunningTimeout = 200;

        i16WaitForPllRunningCnt = 0;
    }


 

    sSyncManOutPar.u16SmEventMissedCounter = 0;
    sSyncManOutPar.u8SyncError = 0;


    sSyncManInPar.u16SmEventMissedCounter = 0;
    sSyncManInPar.u8SyncError = 0;

 

    return result;
}







 

void StopOutputHandler(void)
{
     
    bEcatFirstOutputsReceived = 0;
    bEcatOutputUpdateRunning = 0;
}





 

void StopInputHandler(void)
{
    if(nPdOutputSize > 0)
    {
         
        DisableSyncManChannel(2);
    }

    if(nPdInputSize > 0)
    {
         
        DisableSyncManChannel(3);
    }

     
    {
        unsigned short ResetMask = ((unsigned short) 0x04) | ((unsigned short) 0x08);
        ResetMask |= ((unsigned short) 0x0400);
        ResetMask |= ((unsigned short) 0x0800);

    ResetALEventMask( ~(ResetMask) );
    }
     
    bEcatFirstOutputsReceived = 0;
    bEscIntEnabled = 0;
 

    bDcSyncActive = 0;
    bDcRunning = 0;
    bSmSyncSequenceValid = 0;
    u16SmSync0Value = 0;
    u16SmSync0Counter = 0;

    Sync0WdValue = 0;
    Sync0WdCounter = 0;
    Sync1WdCounter = 0;
    Sync1WdValue = 0;
    LatchInputSync0Value = 0;
    LatchInputSync0Counter = 0;

     

    sSyncManOutPar.u16SmEventMissedCounter = 0;
    sSyncManOutPar.u16CycleExceededCounter = 0;
    sSyncManOutPar.u8SyncError = 0;


    sSyncManInPar.u16SmEventMissedCounter = 0;
    sSyncManInPar.u16CycleExceededCounter = 0;
    sSyncManInPar.u8SyncError = 0;
 

    i16WaitForPllRunningTimeout = 0;

    bWdTrigger = 0;
    bEcatInputUpdateRunning = 0;

     
    bSyncSetByUser = 0;
}





 

void BackToInitTransition(void)
{
     
    bSyncSetByUser = 0;
}







 
void SetALStatus(unsigned char alStatus, unsigned short alStatusCode)
{
    unsigned short Value = alStatusCode;

     
    if(nAlStatus != alStatus)
    {
        nAlStatus = alStatus;
    }


    if (alStatusCode != 0xFFFF)
    {
        Value = (Value);

        HW_EscWrite(((unsigned long *)&(Value)),((unsigned short)(0x0134)),2);
    }

    Value = nAlStatus;
    Value = (Value);
    HW_EscWrite(((unsigned long *)&(Value)),((unsigned short)(0x0130)),2);

}


















 

void AL_ControlInd(unsigned char alControl, unsigned short alStatusCode)
{
    unsigned short        result = 0;
    unsigned char            bErrAck = 0;
    unsigned char         stateTrans;
     
    EsmTimeoutCounter = -1;
    bApplEsmPending = 1;

     
    if ( alControl & ((unsigned char) 0x10) )
    {
        bErrAck = 1;
        nAlStatus &= ~((unsigned char) 0x10);
         
    }
    else if ((nAlStatus & ((unsigned char) 0x10))
        
        
         
        && (alControl & ((unsigned char) 0x0F)) != ((unsigned char) 0x01))
    {
        

 
        return;
    }
    else
    {
        nAlStatus &= ((unsigned char) 0x0F);
    }

    
 
    alControl &= ((unsigned char) 0x0F);
    stateTrans = nAlStatus;
    stateTrans <<= 4;
    stateTrans += alControl;

     
    switch ( stateTrans )
    {
    case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
    case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x02))):
    case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x02))):
    case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x02))):
        

 
        result = CheckSmSettings(1+1);
        break;
    case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
        {
        



 
        result = APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);

            if (result != 0)
            {
                break;
            }
        }
    case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
    case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x04))):
    case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x04))):
    case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x08))):
        

 
        result = CheckSmSettings(nMaxSyncMan);
        break;

    }

    if ( result == 0 )
    {
         
        nEcatStateTrans = 0;
        switch ( stateTrans )
        {
        case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03)))    :
             
            bBootMode = 1;

            if ( CheckSmSettings(1+1) != 0 )
            {
                bBootMode = 0;
                result = 0x0015;
                break;
            }
             
            ResetALEventMask(0);

            


 
            result = MBX_StartMailboxHandler();
            if (result == 0)
            {
                bApplEsmPending = 0;
                


 
            
                result = APPL_StartMailboxHandler();
                if ( result == 0 )
                {
                     
                    bMbxRunning = 1;
                }
            }

            if(result != 0 && result != 0xFF)
            {
                 
                    if (!bApplEsmPending)
                    {
                        APPL_StopMailboxHandler();
                    }

                 MBX_StopMailboxHandler();
            }

            BL_Start( ((unsigned char) 0x03) );

            if (result != 0)
            {
                bBootMode = 0;
            }



            break;

        case ((unsigned char)((((unsigned char) 0x03)) << 4) | (((unsigned char) 0x01)))    :
            if(bBootMode)
            {
                bBootMode = 0;
                 
                ResetALEventMask(0);
                MBX_StopMailboxHandler();
                result = APPL_StopMailboxHandler();
            }

            BL_Stop();

            BackToInitTransition();



            break;
        case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))) :

           UpdateEEPROMLoadedState();

            if (EepromLoaded == 0)
            {
                
                result = 0x0051;
            }
            if (result == 0)
            {
            


 
            result = MBX_StartMailboxHandler();
            if (result == 0)
            {
                bApplEsmPending = 0;
                


 
                result = APPL_StartMailboxHandler();
                if ( result == 0 )
                {
                    bMbxRunning = 1;
                }
            }

            if(result != 0 && result != 0xFF)
            {
                 
                    if (!bApplEsmPending)
                    {
                        APPL_StopMailboxHandler();
                    }

                 MBX_StopMailboxHandler();
            }

            }
            break;

        case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
             
            result = StartInputHandler();
            if ( result == 0 )
            {
                bApplEsmPending = 0;
                result = APPL_StartInputHandler(&u16ALEventMask);

                if(result == 0)
                {
                     
                    SetALEventMask( u16ALEventMask );

                    bEcatInputUpdateRunning = 1;
                }
            }

             
            if(result != 0 && result != 0xFF)
            {
                if(!bApplEsmPending)
                {
                     
                     
                    APPL_StopInputHandler();
                }

                StopInputHandler();
            }
            break;

        case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
             
            result = StartOutputHandler();
            if(result == 0)
            {
                bApplEsmPending = 0;
                result = APPL_StartOutputHandler();

                if(result == 0)
                {
                     
                    bEcatOutputUpdateRunning = 1;
                }

            }

            if ( result != 0 && result != 0xFF)
            {
                    if (!bApplEsmPending)
                    {
                        APPL_StopOutputHandler();
                    }

                StopOutputHandler();
            }

            break;

        case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x04))):
             
            APPL_StopOutputHandler();

            StopOutputHandler();

            bApplEsmPending = 0;

            break;

        case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x02))):
             
            result = APPL_StopOutputHandler();

            StopOutputHandler();

            bApplEsmPending = 0;

            if (result != 0)
            {
                break;
            }

            stateTrans = ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x02)));

        case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x02))):
             
            APPL_StopInputHandler();
           
            StopInputHandler();

            bApplEsmPending = 0;

            break;

        case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x01))):
             
            result = APPL_StopOutputHandler();

            StopOutputHandler();

            bApplEsmPending = 0;

            if (result != 0)
            {
                break;
            }
            
            stateTrans = ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x01)));

        case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x01))):
             
            result = APPL_StopInputHandler();
            
            StopInputHandler();

            bApplEsmPending = 0;

            if (result != 0)
            {
                break;
            }
            stateTrans = ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x01)));

        case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x01))):
            MBX_StopMailboxHandler();
            result = APPL_StopMailboxHandler();

            BackToInitTransition();
            break;

        case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x01))):
            BackToInitTransition();
        case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x02))):
        case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x04))):
        case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x08))):
            if(bErrAck)
            {
                APPL_AckErrorInd(stateTrans);
            }

 

                 
                if ( nAlStatus & (((unsigned char) 0x04) | ((unsigned char) 0x08)))
                {
                    if(nPdOutputSize > 0)
                    {
                        EnableSyncManChannel(2);
                    }
                    else 
                    if(nPdInputSize > 0)
                    {
                        EnableSyncManChannel(3);
                    }
                }
 
            
            result = 0xFE;
            break;

        case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x04))):
        case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x08))):
        case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x08))):
        case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x03))):
        case ((unsigned char)((((unsigned char) 0x04)) << 4) | (((unsigned char) 0x03))):
        case ((unsigned char)((((unsigned char) 0x08)) << 4) | (((unsigned char) 0x03))):
        case ((unsigned char)((((unsigned char) 0x03)) << 4) | (((unsigned char) 0x02))):
        case ((unsigned char)((((unsigned char) 0x03)) << 4) | (((unsigned char) 0x04))):
        case ((unsigned char)((((unsigned char) 0x03)) << 4) | (((unsigned char) 0x08))):
            result = 0x0011;
            break;

        default:
            result = 0x0012;
            break;
        }
    }
    else
    {
        
 
        switch (nAlStatus)
        {
        case ((unsigned char) 0x08):
             
            APPL_StopOutputHandler();
            StopOutputHandler();
        case ((unsigned char) 0x04):
             
            APPL_StopInputHandler();

            StopInputHandler();
        case ((unsigned char) 0x02):
            if ( result == 0x0016 )
            {
                 
                MBX_StopMailboxHandler();
                APPL_StopMailboxHandler();

                 
                DisableSyncManChannel(0);

                 
                DisableSyncManChannel(1);

                nAlStatus = ((unsigned char) 0x01);
            }
            else
            {
                nAlStatus = ((unsigned char) 0x02);
            }
        }
    }

    if ( result == 0xFF )
    {
        
 
        bEcatWaitForAlControlRes = 1;
         
        nEcatStateTrans = stateTrans;

         
        switch(nEcatStateTrans)
        {
            case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
            case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03))):
                EsmTimeoutCounter = 0x7D0;
            break;
            case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
            case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
                EsmTimeoutCounter = 0x2328;
                break;
           default:
                EsmTimeoutCounter = 200; 
                break;
        }
         
        EsmTimeoutCounter -= (short) (EsmTimeoutCounter / 10); 
         

    }
    else if ( alControl != (nAlStatus & ((unsigned char) 0x0F)) )
    {
         

        if ( (result != 0 || alStatusCode != 0) && ((alControl | nAlStatus) & ((unsigned char) 0x08)) )
        {
            
 

            
            if(bEcatOutputUpdateRunning)
            {
                APPL_StopOutputHandler();

                StopOutputHandler();
            }

            if(nPdOutputSize > 0)
            {
                 
                DisableSyncManChannel(2);
            }
            else
                if(nPdInputSize > 0)
            {
                 
                DisableSyncManChannel(3);
            }

        }
        if ( result != 0 )
        {
                if (nAlStatus == ((unsigned char) 0x08))
                {
                    nAlStatus = ((unsigned char) 0x04);
                }
            
 
            nAlStatus |= ((unsigned char) 0x10);
        }
        else
        {
             
            if ( alStatusCode != 0 )
            {
                 
                result = alStatusCode;
                alControl |= ((unsigned char) 0x10);
            }
             
            nAlStatus = alControl;
        }

        bEcatWaitForAlControlRes = 0;

         
        SetALStatus(nAlStatus, result);
    }
    else
    {
         

         bEcatWaitForAlControlRes = 0;

        
 
        SetALStatus(nAlStatus, 0);
    }

}







 
void AL_ControlRes(void)
{
    if(bEcatWaitForAlControlRes)
    {
        unsigned short result = 0;
        unsigned char Status = 0;
        unsigned short StatusCode = 0;

        if(EsmTimeoutCounter == 0)
        {
            Status =  (unsigned char)(nEcatStateTrans >> 4);

             
            switch(nEcatStateTrans)
            {
                case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
                case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03))):

                        if (!bApplEsmPending)
                        {
                            APPL_StopMailboxHandler();
                        }

                    MBX_StopMailboxHandler();
                    if(bLocalErrorFlag)
                    {
                         
                        StatusCode = u16LocalErrorCode;
                    }
                    else
                    {
                         
                        StatusCode = 0x0001;
                    }
                break;
                case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):

                        if (!bApplEsmPending)
                        {
                            APPL_StopInputHandler();
                        }

                    StopInputHandler();
                    
                    if(bLocalErrorFlag)
                    {
                         
                        StatusCode = u16LocalErrorCode;
                    }
                    else
                    {
                         
                        StatusCode = 0x0001;
                    }
                break;
                case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
                    if(bDcSyncActive)
                    {
                         
                        if(!bDcRunning)
                        {
                             
                            StatusCode = 0x002D;
                        }
                        else if(!bEcatFirstOutputsReceived && (nPdOutputSize > 0))
                        {
                             
                            StatusCode = 0x001B;
                        }
                        else
                        {
                             
                            StatusCode = 0x001A;
                        }
                    }
                    else
                    {
                        {
                             
                            Status = ((unsigned char) 0x08);
                            StatusCode = 0;
                             
                            bEcatOutputUpdateRunning = 1;
                        }
                    }

                     
                    if(StatusCode != 0)
                    {
                            if (!bApplEsmPending)
                            {
                                APPL_StopOutputHandler();
                            }

                        StopOutputHandler();
                    }
                break;
            }
        } 
        else
        {
             
            switch(nEcatStateTrans)
            {
                case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
                case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03))):
                    if(bApplEsmPending)
                    {
                        bApplEsmPending = 0;
                         
                        result = APPL_StartMailboxHandler();

                        if(result == 0)
                        {
                             
                            bMbxRunning = 1;
                            Status =  (unsigned char)(nEcatStateTrans & ((unsigned char) 0x0F));
                        }
                        else
                        {
                            
 

                            if(result != 0xFF)
                            {
                                APPL_StopMailboxHandler();
                                MBX_StopMailboxHandler();
                            }
                        }
                    }
                break;
                case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
                    if(bApplEsmPending)
                    {
                        bApplEsmPending = 0;
                        result = APPL_StartInputHandler(&u16ALEventMask);

                        if(result == 0)
                        {
                            bEcatInputUpdateRunning = 1;
                            Status = ((unsigned char) 0x04);
                        }
                        else
                        {
                            
 

                            if(result != 0xFF)
                            {
                                APPL_StopInputHandler();
                                StopInputHandler();
                            }
                        }
                    }
                break;
                case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
                   if(bApplEsmPending)
                    {
                        if(bDcSyncActive)
                        {
                            if(i16WaitForPllRunningTimeout > 0 && i16WaitForPllRunningTimeout <= i16WaitForPllRunningCnt)
                            {
                                
 

                                i16WaitForPllRunningTimeout = 0;
                                i16WaitForPllRunningCnt = 0;

 
                                bApplEsmPending = 0;
 
                                result = APPL_StartOutputHandler();

                                if(result == 0)
                                {
                                     
                                    bEcatOutputUpdateRunning = 1;
                                    Status = ((unsigned char) 0x08);
                                }
                                else
                                {
                                    if(result != 0xFF)
                                    {
                                        APPL_StopOutputHandler();
                                        StopOutputHandler();
                                    }
                                }
                            }
                        }
                        else
                        {
                            {
 
                                bApplEsmPending = 0;  
 
                                result = APPL_StartOutputHandler();

                                if(result == 0)
                                {
                                     
                                    bEcatOutputUpdateRunning = 1;
                                    Status = ((unsigned char) 0x08);
                                }
                                else
                                {
                                    if(result != 0xFF)
                                    {
                                        APPL_StopOutputHandler();
                                        StopOutputHandler();
                                    }
                                }
                            }
                        }       
                    }             
                break;
            }
        }

        if(Status != 0)
        {
             
            bEcatWaitForAlControlRes = 0;

            if (StatusCode != 0)
            {
                Status |= ((unsigned char) 0x10);
            }

            SetALStatus(Status,StatusCode);
        }
    }
}








 
void DC_CheckWatchdog(void)
{
 
 

    if(bDcSyncActive && bEcatInputUpdateRunning)
    {
         
        if((Sync0WdValue > 0) && (Sync0WdCounter >= Sync0WdValue))
        {
                 
                bDcRunning = 0;        
        }
        else
        {
            if(Sync0WdCounter < Sync0WdValue)
            {
                Sync0WdCounter ++;
            }

            bDcRunning = 1;
        }

        if(bDcRunning)
        {
             
            if(Sync1WdValue > 0)
            {
                if(Sync1WdCounter < Sync1WdValue)
                {
                    Sync1WdCounter ++;
                }
                else
                {
                     
                    bDcRunning = 0;
                }
            }
        }
        if(bDcRunning)
        {
           if(sSyncManOutPar.u16SmEventMissedCounter < sErrorSettings.u16SyncErrorCounterLimit)
            {
                bSmSyncSequenceValid = 1;

                 
                if (i16WaitForPllRunningTimeout > 0)
                {
                    i16WaitForPllRunningCnt++;
                }
            }
            else if (bSmSyncSequenceValid)
            {
                    bSmSyncSequenceValid = 0;

                 
                if (i16WaitForPllRunningTimeout > 0)
                {
                    i16WaitForPllRunningCnt = 0;
                }
            }
        }
        else if(bSmSyncSequenceValid)
        {
           bSmSyncSequenceValid = 0;
        }
    }
     
     
}






 
void CheckIfEcatError(void)
{
   
 
   if (EcatWdValue != 0)
   {
       
      unsigned short WdStatusOK = 0;

      HW_EscRead(((unsigned long *)&(WdStatusOK)),((unsigned short)(0x0440)),2);
      WdStatusOK = (WdStatusOK);

      if (!(WdStatusOK & 0x0001) && (nPdOutputSize > 0))
      {
          

         if (bEcatOutputUpdateRunning
            && bEcatFirstOutputsReceived
            )
         {
            AL_ControlInd(((unsigned char) 0x04), 0x001B);
            return;
         }

         else
         {
            bEcatFirstOutputsReceived = 0;
         }
      }
   }

   if(bDcSyncActive)
   {
       if(bEcatOutputUpdateRunning)
       {
            
           if(!bDcRunning)
           {
               AL_ControlInd(((unsigned char) 0x04), 0x002C);
               return;
           }
           else if(!bSmSyncSequenceValid)
           {
               AL_ControlInd(((unsigned char) 0x04), 0x001A);
               return;
           }
        
       }
   }
}







 

void ECAT_StateChange(unsigned char alStatus, unsigned short alStatusCode)
{
    unsigned char Status = alStatus;


    if(bEcatWaitForAlControlRes)
    {
         

        if(bApplEsmPending)
        {
            
 
            if(alStatusCode != 0)
            {
                bLocalErrorFlag = 1;
                u16LocalErrorCode = alStatusCode;
                EsmTimeoutCounter = 0;
            }
        }
        else
        {
             

            if(alStatusCode != 0)
            {
                bLocalErrorFlag = 1;
                u16LocalErrorCode = alStatusCode;

                 
                switch(nEcatStateTrans)
                {
                    case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
                    case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03))):
                     
                          APPL_StopMailboxHandler();
                          MBX_StopMailboxHandler();
                    break;
                    case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
                          APPL_StopInputHandler();
                          StopInputHandler();
                    break;
                    case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
                          APPL_StopOutputHandler();
                          StopOutputHandler();
                    break;
                }

                 
                Status =  (unsigned char)(nEcatStateTrans >> 4);
            }
            else
            {
                 
                 
                switch(nEcatStateTrans)
                {
                    case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x02))):
                    case ((unsigned char)((((unsigned char) 0x01)) << 4) | (((unsigned char) 0x03))):
                        bMbxRunning = 1;
                    break;
                    case ((unsigned char)((((unsigned char) 0x02)) << 4) | (((unsigned char) 0x04))):
 
                         
                        SetALEventMask(u16ALEventMask);
 
                        bEcatInputUpdateRunning = 1;
                    break;
                    case ((unsigned char)((((unsigned char) 0x04)) << 4) |( ((unsigned char) 0x08))):
                          bEcatOutputUpdateRunning = 1;
                    break;
                }

                 
                Status =  (unsigned char)(nEcatStateTrans & ((unsigned char) 0x0F));
            }
                 
                bEcatWaitForAlControlRes = 0;

                if (alStatusCode != 0)
                {
                    Status |= ((unsigned char) 0x10);
                }

                SetALStatus(Status,alStatusCode);

        }
    }
    else
    {
        if ( alStatusCode != 0 )
        {
            bLocalErrorFlag = 1;
            u16LocalErrorCode = alStatusCode;

     
            if (((nAlStatus & ((unsigned char) 0x10)) != ((unsigned char) 0x10)) || ((alStatus & ((unsigned char) 0x0F)) < (nAlStatus & ((unsigned char) 0x0F))))
            {
                 

                
                AL_ControlInd(alStatus, alStatusCode);
            }
     
        }
        else if (bLocalErrorFlag)
        {
             
            bLocalErrorFlag = 0;
            u16LocalErrorCode = 0x00;
        }
    }
}





 

void ECAT_Init(void)
{
    unsigned char i;

     
    HW_EscRead(((unsigned long *)&(nMaxSyncMan)),((unsigned short)(0x0005)),1);

    HW_EscRead(((unsigned long *)&(nMaxEscAddress)),((unsigned short)(0x0006)),2);
    
    nMaxEscAddress = (nMaxEscAddress << 10) + 0xFFF;

     
    UpdateEEPROMLoadedState();

     
    for (i = 0; i < nMaxSyncMan; i++)
    {
        DisableSyncManChannel(i);
    }

     
    MBX_Init();

     
    bBootMode = 0;
    bApplEsmPending = 0;
    bEcatWaitForAlControlRes = 0;
    bEcatFirstOutputsReceived = 0;
     bEcatOutputUpdateRunning = 0;
     bEcatInputUpdateRunning = 0;
    bWdTrigger = 0;
    EcatWdValue = 0;
    Sync0WdCounter = 0;
    Sync0WdValue = 0;
    Sync1WdCounter = 0;
    Sync1WdValue = 0;
    bDcSyncActive = 0;
    bLocalErrorFlag = 0;
    u16LocalErrorCode = 0x00;

    u16ALEventMask = 0;
    nPdOutputSize = 0;
    nPdInputSize = 0;

     
    nAlStatus    = ((unsigned char) 0x01);
    SetALStatus(nAlStatus, 0);
    nEcatStateTrans = 0;

    bEscIntEnabled = 0;

     
    COE_Init();

     
    ResetALEventMask(0);
}




 

void ECAT_Main(void)
{
    unsigned short ALEventReg;
    unsigned short EscAlControl = 0x0000;
    unsigned char sm1Activate = 0x01;


     
    MBX_Main();


    if ( bMbxRunning )
    {
         
         
        HW_EscRead(((unsigned long *)&(sm1Activate)),((unsigned short)((0x0806 + 8))),1);
    }

     
    ALEventReg = HW_GetALEventRegister();
    ALEventReg = (ALEventReg);


    if ((ALEventReg & ((unsigned short) 0x01)) && !bEcatWaitForAlControlRes)
    {
        
 
        HW_EscRead(((unsigned long *)&(EscAlControl)),((unsigned short)(0x0120)),1);
        EscAlControl = (EscAlControl);


        
 
        ALEventReg &= ~((((unsigned short) 0x01)) | (((unsigned short) 0x10)));

        AL_ControlInd((unsigned char)EscAlControl, 0);  
        
         
    }

    if ( (ALEventReg & ((unsigned short) 0x10)) && !bEcatWaitForAlControlRes && (nAlStatus & ((unsigned char) 0x10)) == 0 && (nAlStatus & ~((unsigned char) 0x10)) != ((unsigned char) 0x01) )
    {
        
 
        ALEventReg &= ~(((unsigned short) 0x10));

         
        AL_ControlInd(nAlStatus & ((unsigned char) 0x0F), 0);
    }

    if(bEcatWaitForAlControlRes)
    {
        AL_ControlRes();
    }
    




 
    if ( bMbxRunning )
    {
        
 
            if (!(sm1Activate & 0x01))
            {
                AL_ControlInd(nAlStatus & ((unsigned char) 0x0F), 0);
            }

        if ( ALEventReg & (((unsigned short) 0x0200)) )
        {
            

 
            u8dummy = 0;
            HW_EscWrite(((unsigned long *)&(u8dummy)),((unsigned short)(u16EscAddrSendMbx)),1);

            
 
            ALEventReg &= ~(((unsigned short) 0x0200));
            MBX_MailboxReadInd();
        }

         

        if ( ( (sm1Activate & 0x02) && !bMbxRepeatToggle )
            ||( !(sm1Activate & 0x02) && bMbxRepeatToggle ))
        {
            
 
            MBX_MailboxRepeatReq();
            
 
            sm1Activate &= 0x02;
            HW_EscWrite(((unsigned long *)&(sm1Activate)),((unsigned short)((0x0807 + 8))),1);
        }

         
        ALEventReg = HW_GetALEventRegister();
        ALEventReg = (ALEventReg);

        if ( ALEventReg & (((unsigned short) 0x0100)) )
        {
            

 
            
 
            ALEventReg &= ~(((unsigned short) 0x0100));
            MBX_CheckAndCopyMailbox();

        }
    }
}


 

