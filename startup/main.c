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

#include <stddef.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>

#include "FreeRTOSConfig.h"

#include "lwip/init.h"

#include "print.h"
#include "printf.h"
#include "receive.h"

#include "ethernetif.h"
#include "console.h"

/*
** Startup task
*/
void startupTask(void* pvParameters);

/*
** demo task functions defined in app_init.c
*/
void demoTasks();


void show_version( void )
{
    printf_("\n"
"*********************************************************\n" 
"*       _____________        __    __    _     _____    *\n"
"*  ____//__][______||       ( (`  / /\\  | |\\ | | |_     *\n"
"* (o _ |  -|   _   o|       _)_) /_/--\\ |_| \\| |_|__    *\n"
"*  `(_)-------(_)---'     (c) 2022 All rights reserved. *\n"
"*                                                       *\n"
"*    Simuluated Antomotive Network Environment (SANE)   *\n"
"*                                                       *\n"
"*********************************************************\n" 
"    \n");
}

/* 
** Startup function that creates and runs two FreeRTOS tasks 
*/

/*
 * I M P O R T A N T :
 * Make sure (in startup.s) that main is entered in Supervisor mode.
 * When vTaskStartScheduler launches the first task, it will switch
 * to System mode and enable interrupt exceptions.
*/

void main(void)
{
    consoleInit();
  
    show_version();

    //demoTasks();

    printf_("Loading FreeRTOS %s ... \n", tskKERNEL_VERSION_NUMBER);
    printf_("FreeRTOS %s is ready.\n", tskKERNEL_VERSION_NUMBER);

    /* Create tcp_ip stack thread */
    printf_("Loading lwIP v%s ... \n", LWIP_VERSION_STRING);
    tcpip_init( NULL, NULL );	
    printf_("lwIP v%s is ready.\n", LWIP_VERSION_STRING);

    /* Initilaize the LwIP stack and interfaces */
    
    lwip_network_init();
    
    /* Initialize webserver demo */
    
    //http_server_socket_init();

#if 0
    /*
    ** Create the Startup Task - This is where all the rest of the Application tasks are created
    */
    if ( pdPASS != xTaskCreate(startupTask, "start", 1024, NULL, PRIOR_START_TASK , NULL))
    {
        FreeRTOS_Error("Could not create Startup task\r\n");
    }
    else
    {
       vDirectPrintMsg("Created startupTask, starting scheduler\n");
    }
#endif


#ifdef USE_DHCP
  /* Start DHCPClient */
  xTaskCreate(LwIP_DHCP_task, (int8_t *) "DHCP", configMINIMAL_STACK_SIZE * 2, NULL,DHCP_TASK_PRIO, NULL);
#endif

    vTaskStartScheduler();

    /*
    ** vTaskStartScheduler should never return.
    ** If it does return, typically not enough heap memory is reserved.
    */
    printf_("We should never reach here. System Halt.\n");

    /* just in case if an infinite loop is somehow omitted in FreeRTOS_Error */
    while ( 1 );
}