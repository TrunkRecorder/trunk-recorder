#!/bin/bash

until ./trunk-recorder; do
    echo "Server 'myserver' crashed with exit code $?.  Respawning.." >&2
    sleep 5
done
