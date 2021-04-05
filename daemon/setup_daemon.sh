#!/bin/sh
cwd=$(dirname $0)
echo ${cwd}
cwd=$(cd ${cwd};pwd)

cmd="${cwd}/daemonctl.py start --nohup --all"
echo "Start up command is: " ${cmd}
echo -n "Writing to ~/.profile [Y/n]?"
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
echo ${cmd} >> ~/.profile
tail ~/.profile
fi

cmd="alias fuck=${cwd}/daemonctl.py\nfuck config\nfuck status"
echo "Start up command is: " ${cmd}
echo -n "Writing to ~/.bashrc [Y/n]?"
read ANSWER
if [ "$ANSWER" = "Y" -o "$ANSWER" = "y" -o "$ANSWER" = "" ]; then
echo ${cmd} >> ~/.bashrc
tail ~/.bashrc
fi
