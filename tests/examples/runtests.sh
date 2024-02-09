#! /bin/sh

if [ -z "$INET4_PROJ"]
then
	echo "\$INET4_PROJ is not set, using INET4_PROJ=/workspace/inet4"
	INET4_PROJ=/workspace/inet4
fi

mkdir work
opp_test gen -v *.test || exit 1
(cd work; opp_makemake -o work -f --deep -I../../../src -L../../../src -lESTNeT; make) || exit 1
opp_test run -v *.test -a '-n "../../../../examples;../../../../src;'$INET4_PROJ'/src"'
