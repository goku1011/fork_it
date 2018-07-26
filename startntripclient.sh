#!/bin/bash
#
# $Id$
# Purpose: Start ntripclient

# change these 3 according to your needs
Stream='MILP_RTCM3'
User='CRTNSAIC'
Password='SAICSURVEY'

DateStart=`date -u '+%s'`
SleepMin=10     # Wait min sec for next reconnect try
SleepMax=10000  # Wait max sec for next reconnect try
(while true; do
  #./ntripclient -s www.euref-ip.net -r 80 -d $Stream -u $User -p $Password
  ./ntripclient -m MILP_RTCM3 -s 132.239.152.175 -p SAICSURVEY -r 2103 -u CRTNSAIC -M n -D /dev/ttyUSB3 -B 115200 -T 1 -Y N -A 8
  if test $? -eq 0; then DateStart=`date -u '+%s'`; fi
  DateCurrent=`date -u '+%s'`
  SleepTime=`echo $DateStart $DateCurrent | awk '{printf("%d",($2-$1)*0.02)}'`
  if test $SleepTime -lt $SleepMin; then SleepTime=$SleepMin; fi
  if test $SleepTime -gt $SleepMax; then SleepTime=$SleepMax; fi
  # Sleep 2 percent of outage time before next reconnect try
  sleep $SleepTime
done)
