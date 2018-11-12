#!/usr/bin/env python

import glob
import os, errno
import shutil

def create_export_dir(directory):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory + "/images")
        except OSError as e:
            print e
            return False
        return True
    else:
    	print "Error: Export directory already exists!"
    	return False

def get_csv_data(directory, filename):
    data = []
    with open(directory+"/" + filename + ".csv", 'r') as f: 
        for line in f:
            data.append(line.split(","))
    return data

def get_file(directory, filename):
    # create/open file in append mode
    return open(directory+"/"+filename+".csv", 'a')

def copy_to_export(start_id, pre_poses, post_poses, pushes, export_dir):
    pre_poses_csv = get_file(export_dir, "pre_poses")
    post_poses_csv = get_file(export_dir, "post_poses")
    pushes_csv = get_file(export_dir, "pushes")

    if(pre_poses_csv != None and post_poses_csv != None and pushes_csv != None):
            append_to_csv(start_id, pre_poses_csv, pre_poses)
            append_to_csv(start_id, post_poses_csv, post_poses)
            append_to_csv(start_id, pushes_csv, pushes)

def append_to_csv(start_id, csv_file, data):
    # if csv is empty, write header at top
    if(start_id == 0):
        csv_file.write(",".join(data[0]))

    # iterate over samples and append to csv with possibly higher counter
    for sample in data[1:]:
        csv_file.write(str(int(sample[0])+start_id) + ',')
        csv_file.write(','.join(sample[1:]))

def copy_images(start_id, from_dir, to_dir):
    for img in glob.glob(from_dir+"/images/*.png"):
        fn = img.split("/")[-1]
        fn_arr = fn.split("_")
        new_fn = str(int(fn_arr[0]) + start_id) + "_" + "_".join(fn_arr[1:])
        shutil.copy(img, to_dir+"/images/"+new_fn)

if __name__=="__main__":
    print "Data-merge started!"
    directories = glob.glob("04*/")
    export_dir = "export_data_2"

    print "Creating export directory at: ", export_dir
    create_export_dir(export_dir)
    total_id = 0
    for d in directories:
        print "Entering ", d, 
        pre_poses = get_csv_data(d, "pre_poses")
        post_poses = get_csv_data(d, "post_poses")
        pushes = get_csv_data(d, "pushes")

        last_id = -1
        if(len(pushes) > 1) and pushes[-1][0].isdigit():
            last_id = int(pushes[-1][0])
            print "with ", (last_id + 1), "samples"

        if(last_id > 0 and len(pre_poses) == len(post_poses) == len(pushes)):
            copy_to_export(total_id, pre_poses, post_poses, pushes, export_dir)
            copy_images(total_id, d, export_dir)
            total_id += last_id + 1
            print "Check!"
        else:
            print "Error! Found data is invalid."

    print "Done!"
