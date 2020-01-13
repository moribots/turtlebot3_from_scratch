#!/bin/bash

# COMPILE
g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp;

# PLACE test1_input.txt INTO TEST, THEN PLACE OUTPUT OF TEST INTO output.txt
./rigid2d_test < test1_input.txt > output.txt

# COMPARE OUTPUT WITH ANSWER
cmp -s output.txt test1_answer.txt && echo "Success"
cmp -s output.txt test1_answer.txt || echo "Failure"
