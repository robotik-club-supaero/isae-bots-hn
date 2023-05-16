#!/bin/bash

# Set the default time threshold to 3600 seconds
threshold=3600

# Check if an argument is provided and update the threshold if applicable
if [ $# -gt 0 ]; then
    threshold=$1
fi

echo "-------------------------------------------------"
echo "Uploading files modified in the last $threshold seconds"
echo "-------------------------------------------------"

files=$(find dev/ -type f -newermt "-$threshold secs")

for file in $files
do

    # Check if the file ends with '.pyc' or contains 'pycache'
    if [[ $file == *.pyc ]] || [[ $file == *pycache* ]]
    then
        # echo "Ignoring file: $file"
        continue
    fi

    
    scp "$file" "pi@gr:isae-bots-hn-2023/$file"
    echo "$file"
done
