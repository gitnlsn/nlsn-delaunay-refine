#!/bin/bash

# RUNNABLE='target/debug/nlsn_delaunay-6642eb74e3402f63'
RUNNABLE=$1
kcov --include-path='./src/planar/','./src/elements/','./src/properties/'   \
    --verify                                                                \
    target/cov                                                              \
    $RUNNABLE
