#!/bin/bash
# tool: dp.sh
# category: process
# purpose: batch-deproject several wrist-image pixels onto one horizontal plane z through robot_client.py deproject (free command) and print base-frame x,y per pixel; handy for rim/edge pixel pairs whose midpoint gives a circle centre
# usage: bash knowledge/tools/dp.sh <capture> <plane_z> <u1> <v1> [<u2> <v2> ...]    (run from the session directory)
# inputs/outputs: calls python3 robot_client.py . deproject with plane_z for each pixel; prints "u=.. v=.. -> x=.. y=.." per pixel
# assumptions: wrist camera; plane_z in metres in the base frame; robot_client.py in the current directory
# verified: used successfully in the session that wrote it
cap=$1; z=$2; shift 2
while [ $# -ge 2 ]; do
  python3 robot_client.py . deproject "{\"capture\":$cap,\"cam\":\"wrist\",\"u\":$1,\"v\":$2,\"plane_z\":$z}" | python3 -c "import sys,json; p=json.load(sys.stdin)['point_base']; print('u=$1 v=$2 -> x=%.4f y=%.4f'%(p['x'],p['y']))"
  shift 2
done
