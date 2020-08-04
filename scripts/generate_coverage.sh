#!/bin/bash


kcov --include-path='./src/planar/','./src/elements/','./src/properties/'   \
    --verify                                                                \
    target/cov                                                              \
    $1
