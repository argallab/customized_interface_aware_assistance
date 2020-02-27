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


Phase 3: 
###########################################################
Generate distributions from data collected during Phase 1 and Phase 2
cd src/data_processing/scripts
./generate_personalized_distributions_from_bags.sh NAME-OF-SUBJECT



Phase 4:
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


Phase 5:
##################################################################
End of All Blocks:
Post Session Ranking Survey:

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_bvKTMj4kxDaXdv7
