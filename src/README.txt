Phase 1:
###########################################################
a) Sip and Puff training:

roslaunch simulators sip_puff_training_simulator.launch iteration:=3

Upon launch, in rqt_reconfigure select prompted
press 's' to start prompted training

b) P_UM_GIVEN_UI Test

Tell them to give command as soon as they see it, have 4 seconds to respond
For hard puff, give hard puff, don't slow it down
roslaunch simulators p_um_given_ui_sim.launch SNP:=true duration:=4 iteration:=3 id:=NAME-OF-SUBJECT save:=true
press 's' to start

c) create personalized distribution:
c-1: Extract topics from bag file:
cd /home/corrective_mode_switch_assistance/src/data_processing/scripts
python extract_topics_from_bag.py ~/.ros/bag_name NAME-Of-SUBJECT_p_um_given_ui

c-2: Build distributions from extracted csv file
python p_um_given_ui_distribution_preprocessing.py -path ../raw_data/dan_p_um_given_ui/ -command_prompt _command_prompt.csv -input _joy_sip_puff.csv -id dan


Phase 2:
##########################################################
Open qualtrics:
https://northwesterneng.az1.qualtrics.com/jfe/form/SV_5biu1XSbex3VKVn

a) P_UI_GIVEN_A keyboard test

roslaunch simulators p_ui_given_a_sim.launch subject_id:=NAME-OF-SUBJECT save:=true iteration:=2 block:=3

b) P_UM_GIVEN_A mapping training:

b1) p_um_given_a sim training:

Get familiar with the test environment:
roslaunch simulators p_um_given_a_sim.launch subject_id:=NAME-OF-SUBJECT SNP:=true save:=false

b2) p_um_given_a test:
roslaunch simulators p_um_given_a_sim.: subject_id:=NAME-OF-SUBJECT SNP:=true save:=true

d) create personalized distribution:
d-1: Extract topics from bag file:
cd /home/corrective_mode_switch_assistance/src/data_processing/scripts
python extract_topics_from_bag.py ~/.ros/bag_name NAME-Of-SUBJECT_p_um_given_a
python extract_topics_from_bag.py ~/.ros/bag_name NAME-Of-SUBJECT_p_ui_given_a

d-1: Build distribution from csv files:
python p_um_given_a_distribution_preprocessing.py -path ../raw_data/subject-id_p_um_given_a -id subject-id


e) create p_ui_given_a distribution:
python p_ui_given_a_distribution_preprocessing.py -id subject-id (directly from p_ui_given_a_sim test)
python p_ui_given_a_distribution_preprocessing_implicit.py -id subject-id (optimization based from p_um_given_a test)



Phase 3:
###########################################################
a) create experiment blocks for subject:
python create_experiment_blocks --subject_id name_of_subject

a2) Generate randomized ordering of blocks
python experiment_block_order_generator (note down the order of blocks on a piece of paper)

(Repeat b-1 and b-2 until all blocks are completed)

a-3) Quick training to explain the path and what information is shown on screen.
roslaunch simulators modeinference_sim.launch SNP:=true JOINT:=true subject_id:=NAME-Of-SUBJECT assistance_block:="no" block_id:=0 training:=1 save:=true optim:=0

b-1) run experiment:
roslaunch simulators modeinference_sim.launch SNP:=true JOINT:=true subject_id:=NAME-Of-SUBJECT assistance_block:="no" block_id:=0 training:=0 save:=true optim:=0

b-2) After each block Post Task Survey:

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_51NCtXBYaxKFZVH


Phase 4:
##################################################################
End of All Blocks:
Post Session Ranking Survey:

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_bvKTMj4kxDaXdv7
