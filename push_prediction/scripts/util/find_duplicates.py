#!/usr/bin/env python

import glob

def find_duplicates_in(filename):
	print filename
	with open(filename) as f:
		last_line = -1
		for line in f:
			i = line.split(',')[0]
			if i.isdigit(): 
				if int(i) == last_line:
					print "Duplicate id:", i
				last_line = int(i)

if __name__=="__main__":
	directories = glob.glob('04**')
	print
	print "checking for duplicates"
	for d in directories:
		print "----------------"
		for filename in ["pre_poses.csv", "post_poses.csv", "pushes.csv"]:
			find_duplicates_in(d + "/" + filename)
	print
	print "done!"
	print
