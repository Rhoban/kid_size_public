#include <stdio.h>
#include "buildinfos.h"

#define STR(...)  #__VA_ARGS__
#define XSTR(A) STR(A)

void buildinfos_print()
{
    printf("Starting RhobanServer, built by %s@%s commit %s at %s on %s\n",
        XSTR(VERSION_USER),
        XSTR(VERSION_HOSTNAME),
        XSTR(VERSION_NUM),
        __DATE__,  __TIME__);
}
