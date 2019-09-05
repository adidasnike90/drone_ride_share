#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2019 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file	runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date	2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import csv

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:
	sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def generate_routefile():
	random.seed(66)  # make tests reproducible
	N = 36000  # number of time steps
	# demand per second from different directions
	pWE = 1. / 4
	pEW = 1. / 4
	pNS = 1. / 3
	with open("data/cross.rou.xml", "w") as routes:
		print("""<routes>
		<vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="20" \
guiShape="passenger"/>
		<vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="20" guiShape="passenger"/>

		<route id="right" edges="51o 1i 2o 52i" />
		<route id="left" edges="52o 2i 1o 51i" />
		<route id="down" edges="54o 4i 3o 53i" />""", file=routes)
		vehNr = 0
		for i in range(N):
			if random.uniform(0, 1) < pWE:
				print('	<vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
					vehNr, i), file=routes)
				vehNr += 1
			if random.uniform(0, 1) < pEW:
				print('	<vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
					vehNr, i), file=routes)
				vehNr += 1
			if random.uniform(0, 1) < pNS:
				print('	<vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
					vehNr, i), file=routes)
				vehNr += 1
		print("</routes>", file=routes)


def get_v_state(vid): # return the current state of the target vehicle
	tls = traci.vehicle.getNextTLS(vid)
	tls_id = tls[0][0]
	tls_distance = tls[0][2]
	tls_state = tls[0][3]
	tls_next_switch = traci.trafficlight.getNextSwitch(tls_id) # get the time of next light
	time_to_green = 0
	if tls_state == 'r':
		time_to_green = tls_next_switch - traci.simulation.getTime()
	
	cur = vid
	leader_count = 0 # leading vehicles
	while traci.vehicle.getLeader(cur):
		cur, dis = traci.vehicle.getLeader(cur)
		if dis > tls_distance: # filter out cars that beyond the tls
			break
		leader_count += 1
	
	res = dict()
	res['vid'] = vid
	res['tls'] = tls_id
	res['tls_state'] = tls_state
	res['init_time'] = traci.simulation.getTime()
	res['tls_distance'] = tls_distance
	res['stop_distance'] = -1
	res['time_to_green'] = time_to_green
	res['tls_light_count'] = 0
	res['leading_cars'] = leader_count
	res['tracking_flag'] = False # flag for recording stop duration
	res['stop_pos'] = -1 # if a car stopped before passing the tls, this record the distance between the stop point to the tls
	res['stop_time'] = -1 # the time point that the car stops
	res['time_of_pass'] = -1 # time of passing the tls
	res['stop_leading_cars'] = -1 # number of leading cars when stop accurs
	res['stop_duration'] = -1
	return res


def run():
	"""execute the TraCI control loop"""
	step = 0
	v_dict = dict()
	# we start with phase 2 where EW has green
	traci.trafficlight.setPhase("0", 2)
	while traci.simulation.getMinExpectedNumber() > 0:
		traci.simulationStep()
		v_list = traci.vehicle.getIDList()
		
		for v in v_list:
			if not traci.vehicle.getNextTLS(v): # filter out cars that have passed the tls
				continue
			v_new = get_v_state(v)
			s = traci.trafficlight.getControlledLanes(v_new['tls'])
			#print(s)
			if v not in v_dict and not traci.vehicle.isStopped(v) and traci.vehicle.getNextTLS(v) and step % 55 == 0: # if the vehicle is moving, added to the dictionary and track it
				v_dict[v] = get_v_state(v)
				
			if v in v_dict and traci.vehicle.getNextTLS(v):
				if v_new['tls_state'] != v_dict[v]['tls_state']: # count how many rounds the tls changed
					v_dict[v]['tls_light_count'] += 1
					v_dict[v]['tls_state'] = v_new['tls_state']
					
				if traci.vehicle.getSpeed(v) == 0:
					if not v_dict[v]['tracking_flag']:
						v_dict[v]['stop_starts'] = traci.simulation.getTime()
						v_dict[v]['stop_distance'] = traci.vehicle.getNextTLS(v)[0][2]
						v_dict[v]['stop_leading_cars'] = v_new['leading_cars']
						v_dict[v]['tracking_flag'] = True
				elif v_dict[v]['tracking_flag']:
					v_dict[v]['stop_ends'] = traci.simulation.getTime()
					v_dict[v]['stop_duration'] = v_dict[v]['stop_ends'] - v_dict[v]['stop_starts']
					v_dict[v]['stop_time'] = v_dict[v]['stop_ends']
					v_dict[v]['tracking_flag'] = False
				
				if traci.vehicle.getNextTLS(v)[0][2] < 1:
					v_dict[v]['time_of_pass'] = traci.simulation.getTime()
					with open('saved.csv', 'a') as csvfile:
						writer = csv.writer(csvfile)
						data = [v_dict[v]['tls'], v_dict[v]['vid'], v_dict[v]['init_time'], v_dict[v]['tls_distance'], 
								v_dict[v]['stop_distance'], v_dict[v]['time_to_green'], v_dict[v]['tls_light_count'], 
								v_dict[v]['time_of_pass'], v_dict[v]['leading_cars'], v_dict[v]['stop_leading_cars'], v_dict[v]['stop_duration']]
						writer.writerow(data)
						#print('writting......')
					v_dict.pop(v)

		step += 1
	
	traci.close()
	sys.stdout.flush()


def get_options():
	optParser = optparse.OptionParser()
	optParser.add_option("--nogui", action="store_true",
						 default=False, help="run the commandline version of sumo")
	options, args = optParser.parse_args()
	return options


# this is the main entry point of this script
if __name__ == "__main__":
	options = get_options()
	
	# this script has been called from the command line. It will start sumo as a
	# server, then connect and run
	if options.nogui:
		sumoBinary = checkBinary('sumo')
	else:
		sumoBinary = checkBinary('sumo-gui')

	# first, generate the route file for this simulation
	generate_routefile()

	# this is the normal way of using traci. sumo is started as a
	# subprocess and then the python script connects and runs
	traci.start([sumoBinary, "-c", "data/cross.sumocfg",
							 "--tripinfo-output", "tripinfo.xml"])
	run()

