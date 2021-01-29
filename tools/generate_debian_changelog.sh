#!/usr/bin/env bash

# We want to extract "1.2.3" from "v1.2.3-5-g123abc".
default_tag_version=`git describe --always --tags $(git rev-list --tags --max-count=1) | sed 's/v\([0-9]*\.[0-9]*\.[0-9]*\).*$/\1/'`

# Default to 1 for package version
if [ -z "$1" ]; then
    package_version=1
else
    package_version=$1
fi

if [ -z "$2" ]; then
    tag_version=$default_tag_version
else
    tag_version=$2
fi

# Date according to RFC 5322
date=`date -R`

echo "mavsdk ($tag_version-$package_version) unstable; urgency=medium"
echo ""
echo "  * Initial release"
echo ""
echo " -- Helge Bahmann <helge@auterion.com>  $date"
