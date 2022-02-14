#!/bin/sh

case "$IFACE" in
    lo)
        # The loopback interface does not count.
        # only run when some other interface comes up
        exit 0
        ;;
    *)
        ;;
esac

mqttssh
source ~/ASV_Loyola_US/install/setup.bash
ros2 launch asv_loyola_us system.launch.py