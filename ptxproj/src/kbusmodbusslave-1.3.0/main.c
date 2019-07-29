//------------------------------------------------------------------------------
/// Copyright (c) WAGO Kontakttechnik GmbH & Co. KG
///
/// PROPRIETARY RIGHTS are involved in the subject matter of this material.
/// All manufacturing, reproduction, use and sales rights pertaining to this
/// subject matter are governed by the license agreement. The recipient of this
/// software implicitly accepts the terms of the license.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
///
///  \file     main.c
///
///  \brief    Starting the main loop and initialize the application.
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>

#include "kbus.h"
#include "modbus.h"
#include "utils.h"
#include "conffile_reader.h"
#include "oms_led.h"

static int daemon_flag = 1;
static int main_running = 1;

/** Debug-level variable. Is declared extern in utils.h */
int vlevel = 0;

static struct option long_options[] =
{
    {"nodaemon", no_argument, 0, 'd'},
    {"verbosity", required_argument, 0, 'v'},
    {"help",      no_argument,        0, 'h'},
    {0,0,0,0}
};

/**
 * @brief Print help text
 * @param[in] progname - Progname to be displayed
 * @param[in] quit - Flag if the application should be closed aferwards.
 * @return status
 */
static int usage( char *progname, int quit )
{
    printf ("%s %s - %s-%s\n\n", progname, VERSION, __TIME__, __DATE__);
    printf ("Usage:\t%s [OPTIONS]\n", progname );
    printf ("Options:\n");
    printf ("\t-d,        --nodaemon\t\tnot running in background\n");
    printf ("\t-v[level], --verbosity [level]\tactivate verbose info - level: 1..7\n");
    printf ("\t-h,        --help\t\tPrints this screen\n");
    printf ("\nConfiguration file: /etc/kbusmodbusslave.conf\n");
    if ( quit ) exit( 1 );
    return 0;
}

/**
 * @brief Start the application in background via fork
 */
static void start_daemon(void)
{
    pid_t pid;

    pid = fork();
    /* Fork off the parent process */
    if (pid < 0)
        exit (EXIT_FAILURE);

    /* If we got a good PID, then
       we can exit the parent process. */
    if (pid > 0)
        exit (EXIT_SUCCESS);

    if (setsid() < 0) {
        fprintf(stderr, "Unable to set session id!\n");
        exit (EXIT_FAILURE);
    }
    /* ignore signal SIGHUP */
    signal (SIGHUP, SIG_IGN);

    chdir ("/");

    umask (0);

    /* Close out the standard file descriptors */
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

/**
 * @brief Callback function for signal handler. It will exit the main-loop 
 * and shutdown the application.
 * @param[in] sig - Signal to be printed for debug
 */
void signal_handler(int sig)
{
    dprintf(VERBOSE_STD, "Received Signal (%s)\n",strsignal(sig));
    main_running=0;
}

int main_startUpModules(void)
{
    //Start Modbus-Thread
    if (modbus_start() < 0)
    {
        fprintf(stderr, "Failed to start Modbus thread!\n");
        return -1;
    }

    //Start KBUS-Thread
    if (kbus_start() < 0)
    {
        fprintf(stderr, "Failed to start KBUS thread!\n");
        return -2;
    }

    //Start OMS LED-Thread
    if (oms_led_start() < 0)
    {
        fprintf(stderr, "Failed to start OMS LED thread!\n");
        return -3;
    }

    return 0;
}

void main_shutdownModules(void)
{
    oms_led_stop();
    kbus_stop();
    modbus_stop();
}

/**
 * @brief MAIN - Loop
 * @return Status on application termination
 * @retval 0 on success
 * @retval <0 on error
 * @param[in] argc - Arguments count
 * @param[in] *argv[] - Pointer of arguments table
 */
int main(int argc, char *argv[])
{
    int c;

    if (conf_init() < 0)
    {
        dprintf(VERBOSE_STD, "Unable to set configuration defaults - EXIT\n");
        exit(-1);
    }
    //read configuration file
    if(conf_getConfig() < 0)
    {
        dprintf(VERBOSE_STD, "No configuration is found - EXIT\n");
        exit(-1);
    }

    //parse programm options
    while ( (c = getopt_long(argc, argv, "hdv:", long_options, NULL)) != -1)
    {
        switch (c) {

         case 'v':
          if (str2int(&vlevel, optarg, 10) == STR2INT_SUCCESS)
          {
              vlevel=atoi(optarg);
              dprintf (VERBOSE_INFO, "verbosity level is %d\n", vlevel);
          }
          break;

         case 'd':
          dprintf(VERBOSE_STD, "Not running in background\n");
          daemon_flag = 0;
          break;

         case ':':       /* Option without required operand */
         case 'h':
         case '?':
          usage( argv[0], TRUE );
          break;

         default:
          printf ("?? getopt returned character code 0%o ??\n", c);
        }
    }

    //Demonize
    if(daemon_flag)
    {
        start_daemon();
    }
    dprintf(VERBOSE_STD, "%s running...\n", argv[0]);

    //Connect signal handler
    signal(SIGINT, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGABRT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (main_startUpModules() < 0)
    {
        exit(-1);
    }

    //MAIN THREAD LOOP
    while (main_running)
    {
        //sleep 1000ms
        usleep(1000*1000);
    }

    main_shutdownModules();
    conf_deInit();
    return 0;
}
