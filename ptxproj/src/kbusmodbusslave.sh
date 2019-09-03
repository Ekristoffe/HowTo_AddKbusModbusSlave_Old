#! /bin/sh
# Starts and stops kbusmodbusslave
# /etc/init.d/kbusmodbusslave.sh
### BEGIN INIT INFO
# Provides:     kbusmodbusslave
# Required-Start:       $syslog
# Required-Stop:        $syslog
# Default-Start:        2 3 4 5
# Default-Stop:         0 1 6
# Short-Description:    kbusmodbusslave initialisation
### END INIT INFO

RUNTIME_CHECK="/./etc/config-tools/get_runtime_config running-version"

DAEMON_PATH="/usr/bin"
DAEMON="kbusmodbusslave"
DAEMONOPTS=""

NAME=kbusmodbusslave
DESC="KBUS Modbus Slave for PFC"

PIDFILE=/var/run/$NAME.pid
SCRIPTNAME=/etc/init.d/$NAME.sh

case "$1" in
start)
    if [ -z $RUNTIME_CHECK ]; then 
        printf "%s\n" "Runtime check Fail"
    else
        if [ $RUNTIME_CHECK != 0 ]; then 
            printf "%s\n" "Runtime is running, service not started"
        else
            printf "%-50s" "Starting $NAME..."
            cd $DAEMON_PATH
            PID=`$DAEMON $DAEMONOPTS > /dev/null 2>&1 & echo $!`
            #echo "Saving PID" $PID " to " $PIDFILE
            if [ -z $PID ]; then
                printf "%s\n" "Fail"
            else
                echo $PID > $PIDFILE
                printf "%s\n" "Ok"
            fi
        fi
    fi
;;
status)
        printf "%-50s" "Checking $NAME..."
        if [ -f $PIDFILE ]; then
            PID=`cat $PIDFILE`
            if [ -z "`ps axf | grep ${PID} | grep -v grep`" ]; then
                printf "%s\n" "Process dead but pidfile exists"
            else
                echo "Running"
            fi
        else
            printf "%s\n" "Service not running"
        fi
;;
stop)
        printf "%-50s" "Stopping $NAME"
            PID=`cat $PIDFILE`
            cd $DAEMON_PATH
        if [ -f $PIDFILE ]; then
            kill -HUP $PID
            printf "%s\n" "Ok"
            rm -f $PIDFILE
        else
            printf "%s\n" "pidfile not found"
        fi
;;

restart)
  	$0 stop
  	$0 start
;;

*)
        echo "Usage: $0 {status|start|stop|restart}"
        exit 1
esac
