#! /bin/sh
mkdir work
opp_test gen -v *.test || exit 1
(cd work; opp_makemake -o work -f --deep -I../../../src -L../../../src -lESTNeT; make) || exit 1
opp_test run -v *.test
