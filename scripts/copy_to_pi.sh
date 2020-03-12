#!/bin/bash

SCRIPTDIR=$(dirname "$(readlink -f "$0")")

# Copy the Pi folder to the Rasberry PI home directory
# You must create a file LightMap/scripts/ssh_address.txt with the
# PI host name and address, example pi@192.168.1.1

address=$(cat $SCRIPTDIR/ssh_address.txt)
scp -r $SCRIPTDIR/../Pi pi@192.168.1.244:./
