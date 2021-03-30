#/bin/sh
PARTS="generic"

if [[ "$#" -ne 1 ]]; then
    echo "invalid usage, expects $0 /path/to/controller"
    exit 1
fi

if [[ ! -d "$PARTS" ]]; then
    echo "can not find parts directory (expected in $PARTS)"
    exit 2
fi

if [[ ! -d "$1" ]]; then
    echo "controller $1 does not exist, set it up with the Webots wizard first!"
    exit 3
fi

cp -i "$PARTS/"*.py "$1"

echo "all good!"
