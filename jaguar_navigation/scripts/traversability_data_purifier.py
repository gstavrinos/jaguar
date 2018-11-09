#!/usr/bin/env python
import os
import sys
import math
import rospkg

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    dir_ = rospack.get_path('jaguar_navigation')+'/logs/'
    if not os.path.exists(dir_):
        print 'Logs directory in jaguar_navigation does not exists. Exiting...'
    elif len(sys.argv) <= 2:
        print 'No input file and/or decision (0/1) given'
    else:
        file_ = dir_ + sys.argv[1]
        decision = int(sys.argv[2])

        mode = 0
        # 0 = robot position
        # 1 = slope
        # 2 = step
        # 3 = roughness

        robot_positions = []

        slope_estimations = []
        slope_positions = []
        step_estimations = []
        step_positions = []
        roughness_estimations = []
        roughness_positions = []

        with open(file_, 'r') as log:
            for line in log:
                line = line.strip()
                if line == "#":
                    mode += 1
                elif line == "###":
                    mode = 0
                else:
                    if mode == 0:
                        robot_positions.append(line.split(','))
                    elif mode == 1:
                        slope = line.split(',')
                        slope_positions.append(slope[:1])
                        slope_estimations.append(slope[2])
                    elif mode == 2:
                        step = line.split(',')
                        step_positions.append(step[:1])
                        step_estimations.append(step[2])
                    elif mode == 3:
                        roughness = line.split(',')
                        roughness_positions.append(roughness[:1])
                        roughness_estimations.append(roughness[2])
        with open(dir_ + sys.argv[1].split('.',1)[0]+'.csv', 'a+') as output:
            for slope_pos in slope_positions:
                # Find the magic triplets
                robot_i = -1
                step_i = -1
                roughness_i = -1
                for rpos in robot_positions:
                    if abs(float(slope_pos[0])-float(rpos[0])) <= 0.1 and abs(float(slope_pos[0])-float(rpos[0])) <= 0.1:
                        slope_i = slope_positions.index(spos)
                        break
                for spos in step_positions:
                    if abs(float(spos[0])-float(slope_pos[0])) <= 0.1 and abs(float(spos[0])-float(slope_pos[0])) <= 0.1:
                        step_i = step_positions.index(spos)
                        break
                for rpos in roughness_positions:
                    if abs(float(slope_pos[0])-float(rpos[0])) <= 0.1 and abs(float(slope_pos[0])-float(rpos[0])) <= 0.1:
                        roughness_i = roughness_positions.index(rpos)
                        break

                # We will log this triplet only if the following condition is true
                # Yeah, yeah, there are ways to do it that are easier to read! Shut up now.
                if ((decision == 0 and robot_i < 0) or (decision == 1 and robot_i >= 0)) and roughness_i >= 0 and step_i >= 0:
                    towrite = str(float(slope_estimations[slope_positions.index(slope_pos)])) + ',' + str(float(step_estimations[step_i])) + ',' + str(float(roughness_estimations[roughness_i])) + ',' + str(decision) +'\n'
                    output.write(towrite)
                    print towrite
