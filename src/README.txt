PHASE 0:

#Generate trials for the subject and the trial order.
a) create experiment blocks for subject:
python create_experiment_blocks --subject_id name_of_subject

b) Generate randomized ordering of blocks
python experiment_block_order_generator (note down the order of blocks on a piece of paper)

c) Open 6 different incognito tabs on the tablet for all post-task qualtrics survey so that it is ready to go
https://northwesterneng.az1.qualtrics.com/jfe/form/SV_51NCtXBYaxKFZVH

d) Open a separate incognito window for qualtrics familiarization phase
https://northwesterneng.az1.qualtrics.com/jfe/form/SV_5biu1XSbex3VKVn

e) Open a separate incognito window for POST SESSION SURVEY
https://northwesterneng.az1.qualtrics.com/jfe/form/SV_bvKTMj4kxDaXdv7


Phase 1:
###########################################################

Opening spiel:

The study will last approximately 2 hours and will consist of different phases. The first phase will be a familiarization phase
in which you will get to know what the study is about, what the tasks are and how you will be performing these tasks.
Following the familiarization phase, we will proceed to a training phase, during which you will have be trained on more detailed aspects
the various essential skills that are required to perform the tasks. You will be tested on how well you have acquired these skills. The training will be repeated
until you reach a minimum level of proficiency.
Following that we will proceed to the main study4

Let us get started with the familiarization phase.

SHOW THEM tab from d) in Phase 0

a) P_UI_GIVEN_A keyboard test

SHOW SCREENSHOT of Phase 1
Explain what the subject will be doing as a part of the task
This is essentially the same kind of testing you underwent during the familiarization phase, except that you will be using the keybaord
to enter your choices. There will be a time limit of 5 seconds per question. Your goal is to be as accurate as possible.

roslaunch simulators p_ui_given_a_sim.launch subject_id:=NAME-OF-SUBJECT save:=true iteration:=1 block:=6

Check if they have reached proficiency by running

cd src/data_processing/scripts
./generate_personalized_p_ui_given_a.sh NAME-OF-SUBJECT

If more training is needed there will be a printout at the end which says NEEEDS MORE TRAINING. If so, repeat the procedure.

Phase 2:
###########################################################
Spiel:

Now that you have a conceptual understanding of what commands need to be issued in order to make the circle move in specific directions or perform
mode switches, we will now get familiarized and trained on using the actual control interface itself.

This phase will proceed in 3 stages.
(SHOW SCREENSHOTS FOR PHASE 2)
First (stage a) you will freely explore the different commands and you will be given real time feedback on how you are doing
Second, you will undergo prompted training where you will asked to issue certain commands with feedback.
Lastly, you will just be given a prompt with no feedback.

Lets start with the free exploration phase
a) Sip and Puff training:

show picture of ph2

roslaunch simulators sip_puff_training_simulator.launch iteration:=5

Have them explore what it is to issue different levels of commands.
Upon launch, in rqt_reconfigure select prompted
press 's' to start prompted training

b) P_UM_GIVEN_UI Test

Tell them to give command as soon as they see it, have 4 seconds to respond
For hard puff, give hard puff, don't slow it down
roslaunch simulators p_um_given_ui_sim.launch SNP:=true duration:=4 iteration:=15 subject_id:=NAME-OF-SUBJECT save:=true
press 's' to start

Phase 3: (Putting it all together)
##########################################################

b) P_UM_GIVEN_A mapping training:

b1) p_um_given_a sim training:

Get familiar with the test environment:
roslaunch simulators p_um_given_a_sim.launch subject_id:=NAME-OF-SUBJECT SNP:=true save:=false training:=1

Do tthe pumgivena experiment
roslaunch simulators p_um_given_a_sim.launch subject_id:=NAME-OF-SUBJECT SNP:=true save:=true training:=0


Phase 4:
###########################################################
Generate distributions from data collected during Phase 1 and Phase 2
cd src/data_processing/scripts ./generate_personalized_distributions_from_bags.sh NAME-OF-SUBJECT

DO SANITY CHECK OF THE DISTRIBUTIONS BY OPENING THEM AND INSPECTING THEM.


Phase 5:
###########################################################

(Repeat b-1 and b-2 until all blocks are completed)

a-3) Quick training to explain the path and what information is shown on screen.
roslaunch simulators modeinference_sim.launch SNP:=true JOINT:=true subject_id:=NAME-Of-SUBJECT assistance_block:="no" block_id:=0 training:=1 save:=false optim:=0

b-1) run experiment:
roslaunch simulators modeinference_sim.launch SNP:=true JOINT:=true subject_id:=NAME-Of-SUBJECT assistance_block:="no" block_id:=0 training:=0 save:=true optim:=0

b-2) After each block Post Task Survey:

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_51NCtXBYaxKFZVH


Phase 6:
##################################################################
End of All Blocks:
Post Session Ranking Survey:

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_bvKTMj4kxDaXdv7



001 - n_1, f_1, c_0, c_1, n_0, f_0
002 - n_1, f_1, c_0, f_0, n_0, c_1
003 - f_1, c_0, n_1, n_0, c_1, f_0
004 - n_1, f_1, c_0, f_0, c_1, n_0


005 - n_0, c_0, f_0, f_1, c_1, n_1
006 - f_1, c_0, n_1, f_0, n_0, c_1
007 - n_0, c_0, f_1, n_1, f_0, c_1
008 - c_1, n_0, f_1, c_0, n_1, f_0
009 - c_0, f_1, n_0, c_1, n_1, f_0
