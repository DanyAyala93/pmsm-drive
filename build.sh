#!/bin/bash
set -xe

# Set "root level" Directory
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)

# This script is meant to help run build commands through docker-compose
$SCRIPT_DIR/proj/cgi_cicd/docker/build.sh $*