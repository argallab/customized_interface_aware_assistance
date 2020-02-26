#!/bin/bash

# extract topics from bag files
# generate p(Um|Ui)
# generate p(Um|a)
# generate p(Ui|a)
# generate p(Ui|a) optimization
# plot distributions

# TO DO: 
# Get bag files given subject name (search in .ros and find appropriate ones)
# file paths for storing and runnig

# for entry in "/root/.ros"/*
# do 
# 	for file in ${entry}; 
# 	do
# 		echo "$file"
# 	done
# done


# input: bagfile name, subject name 
bagfile=$1
echo "Bagfile: $bagfile"
subject_id=$2
echo "Subejct id: $subject_id"

# p(Um|Ui): (interface_distorition)
# python extract_topics_from_bag.py $bagfile "${subject_id}_p_um_given_ui"

# p(Um|a): (command_following)
# python extract_topics_from_bag.py $bagfile "${subject_id}_p_um_given_a"

# P(Ui|a) (internal_model)
# python extract_topics_from_bag.py $bagfile "${subject_id}_p_ui_given_a"


# Build distributions: 

# P(Um|Ui)
# python p_um_given_ui_distribution_preprocessing.py -id ${subject_id}

# P(Um|a)
# python p_um_given_a_distribution_preprocessing.py -id ${subject_id}

# P(Ui|a)
python command_issuing_distribution_preprocessing.py -path deepak -command_prompt _slash_command_prompt.csv -input _slash_joy_sip_puff.csv -id deepak

# P(Um|a) optimization
# python p_ui_given_a_distribution_preprocessing.py -id subject-id




