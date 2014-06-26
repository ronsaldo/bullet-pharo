#/bin/sh

./make-bindings.sh &> log; cat log | grep "Invalid Smalltalk" | sed 's/^.* name for //g; s/bt\(.*\)/%rename(BT\1) \0;/g'
