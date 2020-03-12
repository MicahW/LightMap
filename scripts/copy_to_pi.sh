#!/bin/bash

SCRIPTDIR=$(dirname "$(readlink -f "$0")")

# Copy the Pi folder to the Rasberry PI home directory
# You must create a file LightMap/scripts/ssh_address.txt with the
# PI host name and address, example pi@192.168.1.1

# Get the address
address=$(cat $SCRIPTDIR/ssh_address.txt)

# Remove the old folder
echo "Deleting remote Pi folder"
ssh $address "rm -rf Pi"

# Add the new folder
echo "Copying local Pi folder to remote"
scp -r $SCRIPTDIR/../Pi $address:./
