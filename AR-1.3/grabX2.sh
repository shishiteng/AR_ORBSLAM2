#!/bin/sh -x

window=$1
out=$2
if [ $1 = ""];then 
  echo "参数错误,正确形式:grab.sh [window id] [outfile]";
  echo "window id获取方法：xwininfo -frame";
  exit;
fi

ffmpeg -f x11grab -s $(xwininfo -id $window | awk '/geometry/ {print $2}' | awk 'BEGIN {FS="+"} {print $1}') -r 25 -i :0.0+$(xwininfo -id $window | awk '/Absolute upper-left X/ {print $4}'),$(xwininfo -id $window | awk '/Absolute upper-left Y/ {print $4}')  $out
