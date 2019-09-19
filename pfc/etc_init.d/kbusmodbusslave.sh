#! /bin/sh
# Starts and stops kbusmodbusslave
# /etc/init.d/kbusmodbusslave.sh
### BEGIN INIT INFO
# Provides:     kbusmodbusslave
# Required-Start:       $syslog
# Required-Stop:        $syslog
# Default-Start:        2 3 4 5
# Default-Stop:         0 1 6
# Short-Description:    start and stop kbusmodbusslave
# Description: KBUS Modbus Slave for PFC
### END INIT INFO

DAEMON_PATH="/usr/bin"
DAEMON="kbusmodbusslave"
DAEMONOPTS=" -v7 -d"

NAME=kbusmodbusslave
DESC="KBUS Modbus Slave for PFC"

PIDFILE=/var/run/$NAME.pid
SCRIPTNAME=/etc/init.d/$NAME.sh

# Function to print usage to stdout.
#
# Return: 0 on success, unequal to 0 otherwise
#-----------------------------------------------------------------------------#
usage() {
	echo "Usage: $0 {status|start|stop|restart}"
}

# Function to check if the runtime is set in the wbm (CoDeSys 2.3 or e!Cockpit).
#
# Return: true if the runtime is set, false otherwise
#-----------------------------------------------------------------------------#
runtime(){
	printf "%s\n" "Checking runtime ..."
	RUNTIME_CHECK=`/./etc/config-tools/get_runtime_config running-version`
	printf "%s\n" "Runtime check output: $RUNTIME_CHECK."
	if [[ -z $RUNTIME_CHECK ]]; then 
		printf "%s\n" "Runtime check failed."
		true
	else
		if [[ $RUNTIME_CHECK != 0 ]]; then 
			printf "%s\n" "Runtime running."
			true
		else
			printf "%s\n" "Runtime stopped."
			false
		fi
	fi
}

# Function to check if the service is running.
#
# Return: true if the service is running, false otherwise
#-----------------------------------------------------------------------------#
running(){
	printf "%s\n" "Checking service $NAME ..."
	PID=`pidof $DAEMON`
	printf "%s\n" "Service check output: $PID."
	if [[ -z $PID ]]; then
		printf "%s\n" "Service $NAME stopped."
		false
	else
		printf "%s\n" "Service $NAME running."
		true
	fi

}

# Main switch
#-----------------------------------------------------------------------------#
case $1 in
	start)	# start the service
		if ! runtime; then
			if ! running; then
				printf "%s\n" "Starting service $NAME..."
				nohup $DAEMON_PATH/$DAEMON $DAEMONOPTS &> /var/log/$NAME.txt &disown;
				
				sleep 2
				if running; then
					printf "%s\n" "Service $NAME started"
				else
					printf "%s\n" "Service $NAME failed to start"
				fi
			else
				printf "%s\n" "Service $NAME already running"
			fi
		else
			printf "%s\n" "Runtime is running, service $NAME will not start"
		fi
	;;

	stop)	# stop the service
		if running; then
			printf "%s\n" "Stopping service $NAME..."
			STOPPING=`killall $DAEMON`
			sleep 2
			if [[ -z $STOPPING ]]; then
				printf "%s\n" "Service $NAME killed."
				running
			else
				printf "%s\n" "Service $NAME not found."
			fi
		fi
	;;

	restart)	# stop and restart the service if the service is already running, otherwise start the service
		$0 stop
		$0 start
	;;

	# try-restart)	# restart the service if the service is already running
	# ;;
	
	# reload)	# cause the configuration of the service to be reloaded without actually stopping and restarting the service
	# ;;
	
	force-reload)	# cause the configuration to be reloaded if the service supports this, otherwise restart the service if it is running
		$0 restart
	;;
	
	status)	# print the current status of the service
		running;
	;;

	*)
		usage
		exit 1
esac
