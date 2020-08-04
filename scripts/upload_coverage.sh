#!/bin/bash

TOKEN=$1

bash <(curl -s https://codecov.io/bash) \
    -t $TOKEN