#!/bin/sh

dump_path=~/push_results/`date +%m-%d_%I:%M:%S`
mkdir -p $dump_path/images
mkdir -p $dump_path/ft_data
echo -n $dump_path
