.. _CC2538DK-demo-sample:

CC2538DK Demonstration
######################

Overview
********

This sample demonstrates running zephyr on cc2538dk, using some GPIOs

Requirements
************

Building, Flashing and Running
******************************

mkdir build
mkdir build/cc2538dk
cd build/cc2538dk
BOARD=cc2538dk cmake ../..
make

(flash with desired tool)
