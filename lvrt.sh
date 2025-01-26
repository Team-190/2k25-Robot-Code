#!/bin/sh
exec 2>&1 >/dev/null
while true; do
  APP_PATH=`nirtcfg --file /etc/natinst/share/lvrt.conf --get section=LVRT,token=RTTarget.ApplicationPath`
  if [ "$APP_PATH" = /home/lvuser/natinst/bin/TBLStartupApp.rtexe ]
  then
    APP_BOOT=`nirtcfg --file /etc/natinst/share/lvrt.conf --get section=LVRT,token=RTTarget.LaunchAppAtBoot,value=true | tr "[:upper:]" "[:lower:]"`
    APP_DISABLED=`nirtcfg --get section=SYSTEMSETTINGS,token=NoApp.enabled,value=false | tr "[:upper:]" "[:lower:]"`
    if [ "$APP_BOOT" = true ] && [ "$APP_DISABLED" = false ]
    then
      /usr/local/frc/bin/frcRunRobot.sh
    fi
  else
    exec -a lvrt ./ni-lvrt
  fi
  sleep 1
done