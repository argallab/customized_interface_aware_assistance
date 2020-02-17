#!/usr/bin/env python

import rosbag_pandas
import os #for file management make directory
import shutil #for file management, copy file
from IPython import embed
from pandas import DataFrame 

def test_number_1():
	df = rosbag_pandas.bag_to_dataframe('/home/corrective_mode_switch_assistance/src/data_processing/scripts/mahdieh.bag')

	embed()

	export_csv = df.to_csv (r'/home/corrective_mode_switch_assistance/src/data_processing/scripts/mahdieh.csv', index = None, header=True) #Don't forget to add '.csv' at the end of the path


	# for topicName in listOfTopics:
	# 		#Create a new CSV file for each topic
	# 		filename = folder + '/' + string.replace(topicName, '/', '_slash_') + '.csv'
	# 		with open(filename, 'w+') as csvfile:
	# 			filewriter = csv.writer(csvfile, delimiter = ',')
	# 			firstIteration = True	#allows header row
	# 			for subtopic, msg, t in bag.read_messages(topicName):	# for each instant in time that has data for topicName
	# 				#parse data from this instant, which is of the form of multiple lines of "Name: value\n"
	# 				#	- put it in the form of a list of 2-element lists
	# 				msgString = str(msg)
	# 				msgList = string.split(msgString, '\n')
	# 				instantaneousListOfData = []
	# 				for nameValuePair in msgList:
	# 					splitPair = string.split(nameValuePair, ':')
	# 					for i in range(len(splitPair)):	#should be 0 to 1
	# 						splitPair[i] = string.strip(splitPair[i])
	# 					instantaneousListOfData.append(splitPair)
	# 				#write the first row from the first element of each pair
	# 				if firstIteration:	# header
	# 					headers = ["rosbagTimestamp"]	#first column header
	# 					for pair in instantaneousListOfData:
	# 						headers.append(pair[0])
	# 					filewriter.writerow(headers)
	# 					firstIteration = False
	# 				# write the value from each pair to the file
	# 				values = [str(t)]	#first column will have rosbag timestamp
	# 				for pair in instantaneousListOfData:
	# 					if len(pair) > 1:
	# 						values.append(pair[1])
	# 				filewriter.writerow(values)
	# 	bag.close()
	# print "Done reading all " + numberOfFiles + " bag files."

if __name__ == '__main__':
	test_number_1()
