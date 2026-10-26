#!/bin/sh
# finish_loop.sh [rounds]: alternate the grid router and a DRC check until
# nothing is left unconnected
cd "$(dirname "$0")"
n=${1:-30}
i=0
while [ $i -lt $n ]; do
  i=$((i + 1))
  cost=$((20 + i * 10))
  python3 handroute.py $cost 2>&1 | grep -v "memory leak"
  out=$(python3 finish.py --step check 2>&1 | grep "^DRC")
  echo "round $i: $out"
  echo "$out" | grep -q "unconnected pads: 0" && break
done
