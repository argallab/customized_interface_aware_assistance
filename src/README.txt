Phase 1: 

a) Sip and Puff training: 

roslaunch simulators sip_puff_training_simulator.launch iteration:=3

Upon launch, in rqt_reconfigure select prompted
press 's' to start prompted training

b) Sip and Puff test: 

Tell them to give command as soon as they see it, have 4 seconds to respond 
For hard puff, give hard puff, don't slow it down
roslaunch simulators command_following.launch SNP:=true duration:=4 iteration:=3 id:=mahdieh save:=true
press 's' to start

c) create personalized distribution: 


Phase 2: 

a) Action-command mapping training: 

Open qualtrics: 
https://northwesterneng.az1.qualtrics.com/jfe/form/SV_5biu1XSbex3VKVn

b) p_um_given_a sim training: 

Get familiar with the test environment: 

c) p_um_given_a test: 

d) create personalized distribution: 


Phase 3: 
create experiment blocks for subject: 
python create_experiment_blocks --subject_id name_of_subject 

Post Task Survey: 

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_51NCtXBYaxKFZVH


Post Session Ranking Survey: 

https://northwesterneng.az1.qualtrics.com/jfe/form/SV_bvKTMj4kxDaXdv7