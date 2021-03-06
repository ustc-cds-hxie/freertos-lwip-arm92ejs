/****************************************************************************
 * Copyright (c) 2021, 2022, Haiyong Xie
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not 
 * use this file except in compliance with the License. You may obtain a copy 
 * of the License at http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   - Neither the name of the author nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER, AUTHOR OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/stats.h"
#include "lwip/inet.h"
#include "print.h"
#include "printf.h"
#include "command.h"
#include "debug.h"

#include "ifconfig.h"
#include "ping.h"
#include "vconf.h"

#include "net.h"
#include "cmdlwiperf.h"
#include "iperfapp.h"

#define MAX_COMMAND_BUF_SIZE (5*1024)

static char buf[MAX_COMMAND_BUF_SIZE];

void cmdCmdTest(int argc, char* argv[]);
void ramDiskTestTask(void *pvParameters);
void cmdRamDiskTest(int argc, char* argv[]);
void cmdArp(int argc, char* argv[]);
void cmdNet(int argc, char* argv[]);
void cmdIperfServer(int argc, char *argv[]);
void cmdIperfApp(int argc, char *argv[]);

/****************************************************************************
 *
 ****************************************************************************/
const ShellCmd SHELL_CMDS[] =
{
#if 0
   {"pwd", "current directory", fsUtils_pwd},
   {"cd", "change directory", fsUtils_cd},
   {"ls", "list files in current directory", fsUtils_ls},
   {"cat", "display content of a file", fsUtils_cat},
   {"lsof", "list open file handlers", vfsInfo},
   {"ifconfig", "ifconfig <interface> <IP> <netmask> <gateway>", netIfconfig},
   {"ping", "ping <IP>", cmdPing},
#endif
   {"ps", "Display threads information", cmdPs},
   {"top", "Display threads runtime information", cmdTop},
   {"ifconfig", "show network interface information, and config interface", cmdIfconfig},
   {"vconf", "show / config vlan config for interfaces: vconf, vconf <intf> <VLAN ID>", cmdVconf},   
   {"ping", "ping <IP>", cmdPing},
   {"lwiperf", "iperf server mode (both TCP and UDP)", cmdIperfServer},
   {"iperfapp", "iperf server / client app (TCP only)", cmdIperfApp},
   {"arp", "show/update arp table: arp, arp -d [IP], arp -i IP MAC Intf", cmdArp},
   {"stat", "show usage information of memory heap, memory pool, and system sem/mutex/mbox", cmdStat},
   {"net", "lwIP utilities: open/lstn/acpt/send/recv/udpc/udpl/udpn/udpb/usnd/stat/idxtoname/nametoidx/gethostnm", cmdNet},
   {"cmdtest", "run batch of common commands", cmdCmdTest},
   {"ramdisktest", "run batch of RAM disk test", cmdRamDiskTest},
   {"help", "display help message", cmdHelp},
   {NULL, NULL}
};

void vTaskCmdTest(void *pvParameters){

    char buf[1024];

    int i;
    int maxtests = 20;

    for (i=0; i< maxtests; i++){
        int k = xTaskGetTickCount() % 4;

        printf_("---------------------------------\n");
        printf_("i = %d, k = %d\n", i, k);
        printf_("---------------------------------\n");

        switch(k){
            case 0:
                // ps
                vTaskList(buf);
                printf_("Thread Name\tState\tPrio\tRStack\tThreadID\n");
                printf_("------------------------------------------------\n");
                printf_(buf);
                break;
            case 1:
                // top
                vTaskGetRunTimeStats(buf);
                printf_("Thread Name\tRuntime\t\t%%CPU\n");
                printf_("------------------------------------\n");
                printf_("%s", buf);
                break;
            case 2:
                // ifconfig
                stats_display();
                break;
            case 3:
                // ifconfig
                cmdIfconfig(0, NULL);
                break;
        }
        vTaskDelay( 10 / portTICK_RATE_MS );
    }

    printf_("\nvTaskTest ends.\n");

    vTaskDelete(NULL);

    /* suppress a warning since 'params' is ignored */
    (void) pvParameters;
}

void cmdCmdTest(int argc, char* argv[]){
    if ( pdPASS != xTaskCreate(vTaskCmdTest, "CmdTest", 1024, NULL, 2, NULL) )
    {
        SANE_PLATFORM_ERROR(("Could not create CmdTest task\r\n"));
    }
}

void cmdRamDiskTest(int argc, char* argv[]){
    if ( pdPASS != xTaskCreate(ramDiskTestTask, "DiskTest", 1024, NULL, 2, NULL) )
    {
        SANE_PLATFORM_ERROR(("Could not create DiskTest task\r\n"));
    }
}

/*
 * the following three FreeRTOS macros need to be defined in order to call vTaskList()
 *
 * configGENERATE_RUN_TIME_STATS
 * configUSE_TRACE_FACILITY
 * configUSE_STATS_FORMATTING_FUNCTIONS
 */

void cmdPs(int argc, char* argv[])
{
    vTaskList(buf); 
    printf_("Thread Name\tState\tPrio\tRStack\tThreadID\n");
    printf_("------------------------------------------------\n");
    printf_(buf);
}

/*
 * the following three FreeRTOS macros need to be defined in order to call vTaskGetRunTimeStats()
 *
 * configGENERATE_RUN_TIME_STATS
 * configSUPPORT_DYNAMIC_ALLOCATION
 * configUSE_STATS_FORMATTING_FUNCTIONS
 */

void cmdIperfServer(int argc, char *argv[])
{

    lwip_iperf_dual_server_init();

}

void cmdIperfApp(int argc, char *argv[])
{

  cmd_iperfapp(argc, argv);

}

void cmdTop(int argc, char* argv[])
{
    vTaskGetRunTimeStats(buf); 
    printf_("Thread Name\tRuntime\t\t%%CPU\n");
    printf_("------------------------------------\n");
    printf_("%s", buf);
}

void cmdPing(int argc, char *argv[])
{
   if (argc != 2) {
      printf("Usage: ping [destination]\n");
      return;
   }

   ip_addr_t target;
   target.addr = inet_addr(argv[1]);
   ping_thread((void *)&target);
}

static struct command netcommands;

void cmdNet(int argc, char *argv[])
{
    struct command *com = &netcommands;
    int i;

    if (strncmp((const char *)argv[1], "open", 4) == 0) {
    com->exec = com_open;
    com->nargs = 2;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "lstn", 4) == 0) {
    com->exec = com_lstn;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "acpt", 4) == 0) {
    com->exec = com_acpt;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "clos", 4) == 0) {
    com->exec = com_clos;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

#if LWIP_STATS
  } else if (strncmp((const char *)argv[1], "stat", 4) == 0) {
    com->exec = com_stat;
    com->nargs = 0;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];
#endif
  } else if (strncmp((const char *)argv[1], "send", 4) == 0) {
    com->exec = com_send;
    com->nargs = 2;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "recv", 4) == 0) {
    com->exec = com_recv;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "udpc", 4) == 0) {
    com->exec = com_udpc;
    com->nargs = 3;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "udpb", 4) == 0) {
    com->exec = com_udpb;
    com->nargs = 2;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "udpl", 4) == 0) {
    com->exec = com_udpl;
    com->nargs = 3;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "udpn", 4) == 0) {
    com->exec = com_udpn;
    com->nargs = 3;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "usnd", 4) == 0) {
    com->exec = com_usnd;
    com->nargs = 2;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

#if LWIP_SOCKET
  } else if (strncmp((const char *)argv[1], "idxtoname", 9) == 0) {
    com->exec = com_idxtoname;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];

  } else if (strncmp((const char *)argv[1], "nametoidx", 9) == 0) {
    com->exec = com_nametoidx;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];
#endif /* LWIP_SOCKET */
#if LWIP_DNS
  } else if (strncmp((const char *)argv[1], "gethostnm", 9) == 0) {
    com->exec = com_gethostbyname;
    com->nargs = 1;
    for (i = 0; i < com->nargs && i + 2 < argc; i++)
        com->args[i] = argv[i+2];
#endif /* LWIP_DNS */
  } else if (strncmp((const char *)argv[1], "help", 4) == 0) {
    com->exec = com_help;
    com->nargs = 0;
  } else {
    return;
  }

  com->conn = NULL;
  com->exec(com);

}

void cmdHelp(int argc, char *argv[])
{
   printf_("-----------------------------------------------------------------------------------\n");
   printf_("%16s\tUsage\n", "Command");
   printf_("-----------------------------------------------------------------------------------\n");
   for (int i = 0; SHELL_CMDS[i].name != NULL; i++)
   {
      printf_("%16s\t%s\n", SHELL_CMDS[i].name, SHELL_CMDS[i].helpmsg);
   }
}

