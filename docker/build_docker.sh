#!/bin/bash

# Default values
HUBNAME="tumgeka"
TAG="latest"
NAME="carla-autoware-bridge" 

# Parse arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --hubname)
            HUBNAME="$2"
            shift # past argument
            shift # past value
            ;;
        --tag)
            TAG="$2"
            shift # past argument
            shift # past value
            ;;
        --help)
            usage
            ;;
        *)  # unknown option
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Building the CARLA-Autoware-Bridge Dockerfile"

docker build -f $SCRIPT_DIR/Dockerfile --tag $HUBNAME/$NAME:$TAG $PARENT_DIR