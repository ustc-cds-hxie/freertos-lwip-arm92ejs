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

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "printf.h"

#include "history.h"
#include "readline.h"

#include "shell.h"
#include "command.h"


ReadlineData readlineData = READLINE_DATA(256);
HistoryData historyData = HISTORY_DATA(10);

/****************************************************************************
 *
 ****************************************************************************/
static int parseArgs(char* line, char* argv[SHELL_MAX_ARGS])
{
   int argc = 0;
   int i = 0;

   while (argc < (SHELL_MAX_ARGS - 1))
   {
      char quote = '\0';

      while (line[i] == ' ')
         i++;
      if (line[i] == '\0')
         break;

      argv[argc++] = &line[i];

      while (line[i] != '\0')
      {
         if (line[i] == quote)
         {
            quote = '\0';
            memmove(&line[i], &line[i + 1], strlen(&line[i]));
         }
         else if (quote != '\0')
         {
            i++;
         }
         else if ((line[i] == '\'') || (line[i] == '\"'))
         {
            quote = line[i];
            memmove(&line[i], &line[i + 1], strlen(&line[i]));
         }
         else if (line[i] != ' ')
         {
            i++;
         }
         else
         {
            line[i++] = '\0';
            break;
         }
      }
   }

   return argc;
}

inline HistoryData *getHistoryData() { return &historyData; }

inline ReadlineData *getReadlineData() { return &readlineData; }

/* 
** Shell
*/
static void vShellTask( void *pvParameters )
{
    ShellCmd* cmd = (ShellCmd *) pvParameters;

    for( ; ; )
    {
        /* Print out the name of this task. */

        char* line = readline(SHELL_PROMPT);
        char* argv[SHELL_MAX_ARGS];
        
        int argc, i;
        if (line == NULL) 
           break;
        
        if (line != NULL && line[0] != '\0')
        {         

           argc = parseArgs(line, argv);
           
           for (i = 0; cmd[i].name != NULL; i++)
           {
              if (strcmp(cmd[i].name, argv[0]) == 0) 
              {
                 add_history(line);
                 cmd[i].fx(argc, argv);
                 break;
              }
           }
           
           if (cmd[i].name == NULL)
           {
              printf_("command not found: %s\n", argv[0]);
           }
         }
         rl_free(line);

        vTaskDelay( 100 / portTICK_RATE_MS );
    }

    /*
     * we should never reach here
     */
    SANE_PLATFORM_ERROR(("Shell exited. System still runs but there is no shell.\n"));

    /*
     * If the task implementation ever manages to break out of the
     * infinite loop above, it must be deleted before reaching the
     * end of the function!
     */
    vTaskDelete(NULL);
}

void start_shell_task()
{
       /* And finally create two tasks: */
    if ( pdPASS != xTaskCreate(vShellTask, "shell", 128, (void*) &SHELL_CMDS, 1, NULL) )
    {
        SANE_PLATFORM_ERROR(("Could not create task1\r\n"));
    }

}