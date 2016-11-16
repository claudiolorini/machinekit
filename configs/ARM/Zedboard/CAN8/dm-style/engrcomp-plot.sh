#!/usr/bin/env bash

gnuplot <<- EOF
  set term png size 800,700 font arial 14
  set output "engrcomp.png"

  set noborder
  set key off

  set xrange [-150.0:150.0]
  set yrange [-100.0:100.0]

  set xtics 50
  set ytics 50
  set cbtics 0.1
  set format cb "%1.1f"

  set title "Mappa piano X-Y"

  set view map
  set size ratio 1
  set object 1 rect from graph 0, graph 0 to graph 1, graph 1 back
  set object 1 rect fc rgb "white" fillstyle solid 1.0
  set palette rgbformulae 22,13,-31

  splot 'engrcomp.txt' using 1:2:3 with points pointtype 5 pointsize 1 palette linewidth 16
EOF
