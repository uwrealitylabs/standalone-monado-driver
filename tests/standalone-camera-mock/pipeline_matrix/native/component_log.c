// Diagnostic-only log transport. Converter/queue/genlock code is unchanged.
// Avoid pulling the full runtime's JSON/UI logging into this component build.
#include "util/u_logging.h"
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
static _Atomic unsigned long long errors=0,warnings=0;
void matrix_log_reset(void){atomic_store(&errors,0);atomic_store(&warnings,0);}
unsigned long long matrix_log_errors(void){return atomic_load(&errors);}
unsigned long long matrix_log_warnings(void){return atomic_load(&warnings);}
enum u_logging_level u_log_get_global_level(void){return U_LOGGING_TRACE;}
void u_log(const char *file,int line,const char *func,enum u_logging_level level,const char *format,...) {
    (void)file;(void)line;
    if(level==U_LOGGING_ERROR)atomic_fetch_add(&errors,1);
    if(level==U_LOGGING_WARN)atomic_fetch_add(&warnings,1);
    fprintf(stderr,"%s: ",func);
    va_list args;va_start(args,format);vfprintf(stderr,format,args);va_end(args);
    fputc('\n',stderr);
}
