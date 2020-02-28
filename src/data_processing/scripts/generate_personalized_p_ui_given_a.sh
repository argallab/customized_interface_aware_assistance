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

declare -A p_ui_given_a_bags_array
i=0
for full_file in ${search_dir}*.bag;
do
	file_name=${full_file##*/}
	name="$(cut -d'_' -f1 <<<$file_name)"
	p_of="$(cut -d'_' -f3 <<<$file_name)"
	given="$(cut -d'_' -f5 <<<$file_name)"
	# echo "$file_name"
	# echo "$name"
	if [[ "$name" == "$subject_id" ]]; then
		if [[ "$p_of" == 'ui' ]] && [[ "$given" == 'a' ]]; then # p(Ui|a)
			p_ui_given_a_bags_array[$i]=$full_file
			i=i+1
		fi
		# echo $full_file
	fi
done

max_h=0
max_m=0
max_x=0
i=0
for file in "${p_ui_given_a_bags_array[@]}";
do
	file_name=${file##*/}
	hour="$(cut -d'_' -f9 <<<$file_name)" # assuming all the days are the same
	mins="$(cut -d'_' -f10 <<<$file_name)"
	secs="$(cut -d'_' -f11 <<<$file_name)" 	
	if (( $hour == $max_h )); then
		if (( $mins == $max_m)); then
			if (( $secs > $max_s )); then
				max_s=$secs
				i=$i
			fi
		fi
		if (( $mins > $max_m )); then
			max_m=$mins
			i=$i
		fi
	fi
	if (( $hour > $max_h )); then
		max_h=$hour
		i=$i
	fi
	i=i+1
done
			
p_ui_given_a_bag=${p_ui_given_a_bags_array[$i]}

# P(Ui|a) (internal_model)
echo "Extracting: $p_ui_given_a_bag"
python extract_topics_from_bag.py $p_ui_given_a_bag "${subject_id}_p_ui_given_a"

# Build distributions:
# P(Ui|a)
echo "Generating p(ui|a)"
python p_ui_given_a_distribution_preprocessing.py -id ${subject_id}
