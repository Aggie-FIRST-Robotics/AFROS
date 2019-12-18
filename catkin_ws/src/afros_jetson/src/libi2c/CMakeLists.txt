cmake_minimum_required(VERSION 2.8.3)

set(
        libi2c-src
        i2c.c
)

add_library(libi2c ${libi2c-src})
