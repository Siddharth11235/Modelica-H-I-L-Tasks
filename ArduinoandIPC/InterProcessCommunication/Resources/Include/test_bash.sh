#!/bin/bash
cd /dev
for FILE in `ls`
do
    if test $FILE
    then
      b=${FILE:0:6}
      if  [ "$b" == "ttyACM" ]; then
      echo "/dev/"$FILE
      fi
    fi
done
