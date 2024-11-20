#!/bin/sh

cd build/static/js && for filename in main.*; do mv "$filename" "main.js""$(echo "$filename" | sed -e 's/main.*.js//g')"; done && cd ..
cd css && for filename in main.*; do mv "$filename" "main.css""$(echo "$filename" | sed -e 's/main.*.css//g')"; done && cd ../../..