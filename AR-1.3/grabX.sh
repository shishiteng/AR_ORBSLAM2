#!/bin/sh

out=$1
if [ $1 = ""];then
  echo "参数错误,正确形式:grab.sh [outfile]";
  exit;
fi

ffmpeg -f x11grab -s 1920x1080 -i :0 $out
