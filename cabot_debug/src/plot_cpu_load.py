#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2019  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import os
import sys
import re
import rosbag
import numpy
import traceback
import multiprocessing

from optparse import OptionParser
from matplotlib import pyplot as plt
from bisect import bisect

parser = OptionParser(usage="""
Example
{0} -f <bag file>                        # show a list of process whose maximum usage is over 50%
{0} -f <bag file> -s                     # plot overall usage
{0} -f <bag file> -p <pid>|all           # plot a <pid>/all cpu usage
{0} -f <bag file> -p all -S              # plot stacked cpu usage (max > 50%)
{0} -f <bag file> -p all -S -t 0         # plot stacked cpu usage (all)
{0} -f <bag file> -s -d .                # plot to file
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to plot')
parser.add_option('-s', '--summary', action='store_true', help='plot summary')
parser.add_option('-p', '--pid', type=str, help='plot specific process cpu load')
parser.add_option('-t', '--max_threthold', type=float, help='minimum maximum cpu load threthold', default=50.0)
parser.add_option('-T', '--ave_threthold', type=float, help='minimum average cpu load threthold', default=0.0)
parser.add_option('-d', '--dir', type=str, help='output directry', default=None)
parser.add_option('-m', '--max_y', type=float, help='maximum y axix', default=300.0)
parser.add_option('-M', '--stack_max_y', type=float, help='stack maximum y axix', default=100.0*multiprocessing.cpu_count())
parser.add_option('-S', '--stack', action='store_true', help='stack plot')
parser.add_option('-D', '--min_duration', type=float, help='minimum duration', default=15.0)


(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
filename=os.path.splitext(os.path.split(bagfilename)[-1])[0]
bag = rosbag.Bag(bagfilename)
data = []
times = []
summary = tuple([[] for i in range(30)])

pidindex = 9
pidmap = {}

maxcpu = 0
maxmem = 0
count = 0
prev = 0
for topic, msg, t in bag.read_messages(topics=["/top"]):
    lines = msg.data.split("\n")
    items = re.split(" +", lines[2])
    now = t.to_sec()
    if now <= prev:
        continue
    prev = now

    try:
        temp = {}

        for l in lines[7:]:
            line2 = l
            items2 = re.split(" +", l.strip())
            if len(items2) < 12:
                continue
            if items2[11] == "sleep":
                continue
            pid = items2[0]
            cpu = float(items2[8])
            mem = float(items2[9])
            process = " ".join(items2[11:])

            if maxcpu < cpu:
                maxcpu = cpu
            if maxmem < mem:
                maxmem = mem

            key = process

            if not key in pidmap:
                pidmap[key] = [process, pid]

            (index, _) = pidmap[key]

            if not key in temp:
                temp[key] = {"cpu":cpu, "mem":mem}
            else:
                #temp[key]["cpu"] += cpu
                #temp[key]["mem"] += mem
                pass

        data.append(temp)
        times.append(now)

        for (i,v) in enumerate(items[1::2]):
            summary[i+1].append(float(v))
    except:
        print("warning: error parsing")
        #print(line2)
        #print(items2)
        #print("\n".join(lines))
        traceback.print_exc()
        #system.exit()


bag.close()


from pylab import rcParams
rcParams['figure.figsize'] = 40, 20

if options.dir is not None and options.dir != "." and options.dir != "..":
    try:
        os.makedirs(options.dir)
    except:
        print >> sys.stderr, "warning: {} exists".format(options.dir)

if options.summary:
    temp = summary[:9]
    temp[0].extend(times)
    
    plt.stackplot(*temp, labels=['User','System','Nice','Idle','IO-wait','Hardware interrupt','Software interrupt','VM'])
    plt.legend(bbox_to_anchor=(1.0, 1), loc="upper left")
    plt.ylim([0,100])
    plt.xlim([min(temp[0]),max(temp[0])])
    if options.dir:
        plt.savefig(os.path.join(options.dir, filename+"-summary.png"))
    else:
        plt.show()
    sys.exit(0)


def process_data(data):
    ret = {}
    interval = (times[-1] - times[0]) / len(data)
    
    for _, (key, (process, pid)) in enumerate(pidmap.items()):
        temp = [[], []]
        count = 0
        for i in range(len(data)):
            if not key in data[i]:
                temp[0].append(0)
                temp[1].append(0)
            else:
                count+=1
                temp[0].append(data[i][key]["cpu"])
                temp[1].append(data[i][key]["mem"])

        if max(temp[0]) < options.max_threthold:
            continue

        if numpy.average(temp[0]) < options.ave_threthold:
            continue

        duration = interval * count
        if duration < options.min_duration:
            continue
        ret[key] = temp

    return ret

def sort_pids(data2):
    keys = data2.keys()
    return sorted(keys, reverse=True, key=lambda x: numpy.average(data2[x][0]))
    

if options.pid:
    data2 = process_data(data)
    
    pids = []
    if options.pid is not None:
        if options.pid == "all":
            pids = sort_pids(data2)
        else:
            for key in pidmap:
                _, p = pidmap[key]
                if p == options.pid:
                    pids.append(key)

    if options.stack:
        temp = []
        labels = []
        temp.append(times)
        for key in pids:
            process, pid = pidmap[key]

            if key in data2:
                temp.append(data2[key][0])
                labels.append(pid+" "+process[:80])
            else:
                pass


        plt.clf()
        plt.stackplot(*temp, labels=labels)
        plt.xlim([min(times),max(times)])
        plt.ylim(0, options.stack_max_y)
        plt.legend(bbox_to_anchor=(1.0, 1), loc='upper left')
        if options.dir:
            plt.savefig(filename+"-stack.png")
        else:
            plt.show()
    else:
        for key in pids:
            process, pid = pidmap[key]
            plt.clf()
            plt.plot(times, data2[key][0], label=process)
            plt.ylim([0,options.max_y])
            plt.xlim([min(times),max(times)])
            plt.legend()
            if options.dir:
                plt.savefig(os.path.join(options.dir, filename+"-"+pid+".png"))
            else:
                plt.show()
    sys.exit(0)


labels = ['', 'User','System','Nice','Idle','IO-wait','Hardware interrupt','Software interrupt','VM']
for i in range(1,len(labels)):
    print("{:20}: {:05.2f}%".format(labels[i], numpy.average(summary[i])))

print("-----")
print("{:8} {:8} {:8} {:8} {:8} {}".format(
    "Average",
    "Max",
    "Ave(mem)",
    "Max(mem)",
    "PID",
    "Process"))

data2 = process_data(data)
for key in sort_pids(data2):
    process, pid = pidmap[key]
    data = data2[key]
    print("{:8.2f} {:8.2f} {:8.2f} {:8.2f} {:8} {}".format(
        numpy.average(data[0]),
        max(data[0]),
        numpy.average(data[1]),
        max(data[1]),
        pid,
        process))
