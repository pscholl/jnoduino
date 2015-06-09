#!/usr/bin/env bash 

DIRECTORY=$(dirname "$0")
TIMEOUT=20
RET=1

# wait until the reset occured
until [[ $TIMEOUT -eq 0 || $RET -eq 0 ]]; do
  "$DIRECTORY"/dfu-programmer $1 erase
  RET=$?
  let TIMEOUT-=1
  sleep .1
done

if [ $RET -eq 1 ]; then
  echo -n "check permission on /dev/bus/usb/ or hard-reset the device"
  exit -1
fi

# we might be called with either --verbose 5 or --quiet, so we
# pass 4 instead of 3 args to get everything
"${DIRECTORY}/dfu-programmer" $1 flash $2 $3 $4 
"${DIRECTORY}/dfu-programmer" $1 start
sleep 1
