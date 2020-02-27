#!/bin/bash

# extract topics from bag files
# generate p(Um|Ui)
# generate p(Um|a)
# generate p(Ui|a)
# generate p(Ui|a) optimization
# plot distributions


# input: bagfile name, subject name 
# bagfile=$1
# echo "Bagfile: $bagfile"
subject_id=$1
echo "Subejct id: $subject_id"

search_dir="/root/.ros/"

for full_file in ${search_dir}*.bag;
do 
	file_name=${full_file##*/} 
	name="$(cut -d'_' -f1 <<<$file_name)"
	p_of="$(cut -d'_' -f3 <<<$file_name)"
	given="$(cut -d'_' -f5 <<<$file_name)"
	# echo "$file_name"
	# echo "$name"
	if [[ "$name" == "$subject_id" ]]; then
		if [[ "$p_of" == 'um' ]] && [[ "$given" == 'ui' ]]; then # p(Um|Ui)
			p_um_given_ui_bag=$full_file
		fi
		if [[ "$p_of" == 'um' ]] && [[ "$given" == 'a' ]]; then # p(Um|a)
			p_um_given_a_bag=$full_file
		fi
		if [[ "$p_of" == 'ui' ]] && [[ "$given" == 'a' ]]; then # p(Ui|a)
			p_ui_given_a_bag=$full_file
		fi
		# echo $full_file
	fi
done


# p(Um|Ui): (interface_distorition)
echo "Extracting: $p_um_given_ui_bag"
python extract_topics_from_bag.py $p_um_given_ui_bag "${subject_id}_p_um_given_ui"

# p(Um|a): (command_following)
echo "Extracting: $p_um_given_a_bag"
python extract_topics_from_bag.py $p_um_given_a_bag "${subject_id}_p_um_given_a"

# P(Ui|a) (internal_model)
echo "Extracting: $p_ui_given_a_bag"
python extract_topics_from_bag.py $p_ui_given_a_bag "${subject_id}_p_ui_given_a"


# Build distributions: 

# P(Um|Ui)
echo "Generating p(um|ui)"
python p_um_given_ui_distribution_preprocessing.py -id ${subject_id}

# P(Um|a)
echo "Generating p(um|a)"
python p_um_given_a_distribution_preprocessing.py -id ${subject_id}

# P(Ui|a)
echo "Generating p(ui|a)"
python p_ui_given_a_distribution_preprocessing.py -id ${subject_id}

# P(Um|a) optimization
echo "Generating p(ui|a) implicity using optimization"
python p_ui_given_a_distribution_preprocessing_implicit.py -id ${subject_id}




