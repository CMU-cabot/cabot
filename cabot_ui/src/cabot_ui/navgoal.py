# Copyright (c) 2020  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import inspect
import numpy

from cabot_ui import geoutil, geojson

import rospy
import tf
import nav_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import std_msgs.msg

from cabot import util
from actionlib_msgs.msg import GoalStatus

class GoalInterface(object):
    def enter_goal(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def exit_goal(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def naviget_to_pose(self, goal_pose, bt_xml, done_cb):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def naviget_through_poses(self, goal_poses, bt_xml, done_cb):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def set_clutch(self, flag):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def turn_towards(self, orientation, callback, clockwise=0):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def goto_floor(self, floor, callback):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def please_call_elevator(self, pos):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def publish_path(self, global_path, convert=True):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def global_map_name(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def please_pass_door(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def door_passed(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))


def make_goals(delegate, groute, anchor, yaw=None):
    # based on the navcog routeing, this function will make one or multiple goal towards the destination
    # one of goal could be just a goal of navigation2 stack, other could be special goal like getting on
    # or waiting for an elevator cab
    goals = []
    temp = []
    index = 0

    rospy.loginfo("--make_goals-{}--------------------".format(len(groute)))
    rospy.loginfo(groute)
    
    while index < len(groute):
        link_or_node = groute[index]
        temp.append(link_or_node)
        index += 1

        if not isinstance(link_or_node, geojson.RouteLink):
            continue

        link = link_or_node

        if link.is_temp:
            continue

        # there is a manual door

        # find manual door
        doors = list(filter(lambda x:isinstance(x, geojson.DoorPOI) and not x.is_auto, link.pois))
        if doors:
            if len(doors) > 1:
                # will not support
                rospy.logerr("don't put multiple door pois into a link")
                return None
            else:
                # set goal 1 meter behind the door
                goals.append(NavGoal(delegate, temp, anchor, target_poi=doors[0], set_back=(1.0, 0.0)))
                #rospy.loginfo(goals[-1])
                # ask user to pass the door
                goals.append(DoorGoal(delegate, doors[0]))
                #rospy.loginfo(goals[-1])
                temp = []

        # find queue target
        queue_targets = list(filter(lambda x:isinstance(x, geojson.QueueTargetPOI), link.pois))
        if queue_targets:
            if len(queue_targets) > 1:
                # will not support
                rospy.logerr("don't put multiple queue target pois into a link")
                return None
            else:
                # find link with queue enter node from whole route
                queue_enter_link_idx = None
                for idx_temp_r, temp_r in enumerate(groute):
                    if isinstance(temp_r, geojson.RouteLink) and temp_r.end_node._id==queue_targets[0].enter_node._id:
                        queue_enter_link_idx = idx_temp_r
                        break
                if queue_enter_link_idx is None:
                    rospy.logerr("queue enter node is not found")
                    return None

                # navigate to queue enter node
                queue_enter_groute = groute[:queue_enter_link_idx+1]
                queue_enter_groute.append(groute[queue_enter_link_idx].end_node)
                if len(queue_enter_groute)==0:
                    rospy.logerr("route to queue enter node is not found")
                    return None
                goals.append(NavGoal(delegate, queue_enter_groute, anchor))

                # find route to queue target node
                queue_target_groute = groute[queue_enter_link_idx+1:]
                queue_target_groute.insert(0, groute[queue_enter_link_idx].end_node)
                if len(queue_target_groute)==0:
                    rospy.logerr("route to queue target node is not found")
                    return None
                # find queue wait POI that is not copied in queue route yet
                queue_link_idx_queue_wait_dict = {}
                for idx_temp_r, temp_r in enumerate(queue_target_groute):
                    if isinstance(temp_r, geojson.RouteLink):
                        queue_waits = list(filter(lambda x:isinstance(x, geojson.QueueWaitPOI) and not x.is_copied, temp_r.pois))
                        if queue_waits:
                            if len(queue_waits) > 1:
                                rospy.logerr("don't put multiple queue wait pois into a queue route")
                                return None
                            else:
                                queue_link_idx_queue_wait_dict[idx_temp_r] = queue_waits

                for idx_temp_r, temp_r in enumerate(queue_target_groute):
                    if isinstance(temp_r, geojson.RouteLink) and idx_temp_r in queue_link_idx_queue_wait_dict:
                        queue_waits = queue_link_idx_queue_wait_dict[idx_temp_r]
                        # if POI that is not copied is found, copy queue wait POI along the queue route
                        if queue_waits is not None and len(queue_waits)==1:
                            temp_queue_wait = queue_waits[0]
                            temp_queue_wait_interval = queue_waits[0].interval
                            while True:
                                temp_queue_wait_position = numpy.array([temp_queue_wait.local_geometry.x, temp_queue_wait.local_geometry.y])
                                temp_r_start_position = numpy.array([queue_target_groute[idx_temp_r].start_node.local_geometry.x,
                                                                    queue_target_groute[idx_temp_r].start_node.local_geometry.y])
                                dist_poi_r_start = numpy.linalg.norm(temp_queue_wait_position-temp_r_start_position)
                                if dist_poi_r_start>temp_queue_wait_interval:
                                    temp_r_end_pose = geoutil.Pose.pose_from_points(queue_target_groute[idx_temp_r].end_node.local_geometry,
                                                                                    queue_target_groute[idx_temp_r].start_node.local_geometry)
                                    temp_queue_wait_x = temp_queue_wait.local_geometry.x - math.cos(temp_r_end_pose.r) * temp_queue_wait_interval
                                    temp_queue_wait_y = temp_queue_wait.local_geometry.y - math.sin(temp_r_end_pose.r) * temp_queue_wait_interval

                                    temp_queue_wait = queue_waits[0].copy_to_link(queue_target_groute[idx_temp_r], temp_queue_wait_x, temp_queue_wait_y)
                                else:
                                    break
                ### create multiple goals to queue target node
                goals.extend(make_queue_goals(delegate, queue_target_groute, anchor, is_entering=True))

                # navigate to queue exit node
                queue_exit_groute = delegate._datautil.get_route(link.target_node._id, queue_targets[0].exit_node._id)
                if len(queue_exit_groute)==0:
                    rospy.logerr("route to queue exit node is not found")
                    return None
                ### create one goal to queue exit node
                goals.append(QueueNavLastGoal(delegate, queue_exit_groute, anchor))
                ### create multiple goals to queue exit node
                #goals.extend(make_queue_goals(delegate, queue_exit_groute, anchor, is_entering=False))
                
                temp = []

        if link.is_elevator:
            continue

        if link.target_node.is_elevator:
            # find cab POIs
            # elevator cab POIs are associated with non elevator links
            src_cabs = list(filter(lambda x:isinstance(x, geojson.ElevatorCabPOI), temp[-1].pois))
            rospy.loginfo("src_cabs %s", str(src_cabs))
            if len(src_cabs) > 1:
                # TODO
                rospy.logerr("multiple cabs are not supported yet")
                return None
            elif len(src_cabs) == 0:
                rospy.logerr("require elevator cab POI")
                rospy.logerr(temp[-1])
                return None
            else:
                # navigat to in front of the elevator cab (3.0 meters from the cab as default)
                goals.append(NavGoal(delegate, temp, anchor, target_poi=src_cabs[0], set_back=src_cabs[0].set_back))
                #rospy.loginfo(goals[-1])
                # turn towards door
                goals.append(ElevatorWaitGoal(delegate, src_cabs[0]))
                # wait cab and get on the cab
                goals.append(ElevatorInGoal(delegate, src_cabs[0]))
                #rospy.loginfo(goals[-1])
                temp = []

        if link.source_node.is_elevator:
            dest_cabs = list(filter(lambda x:isinstance(x, geojson.ElevatorCabPOI), link.pois))
            if len(dest_cabs) > 1:
                rospy.logerr("multiple cabs are not supported yet")
                return None
            else:
                # turn towards exit door (assumes cab POI has angle facing towards opening door)
                goals.append(ElevatorTurnGoal(delegate, dest_cabs[0]))
                #rospy.loginfo(goals[-1])
                # wait until the door open
                goals.append(ElevatorFloorGoal(delegate, dest_cabs[0]))
                #rospy.loginfo(goals[-1])
                # forward 3.0 meters (as default) without global mapping support
                goals.append(ElevatorOutGoal(delegate, dest_cabs[0], set_forward=dest_cabs[0].set_forward))
                #rospy.loginfo(goals[-1])
                temp = [link]

        # TODO: escalator
    if len(temp) > 1:
        goals.append(NavGoal(delegate, temp, anchor, is_last=True))

    if yaw is not None:
        goals.append(TurnGoal(delegate, anchor, yaw))

    rospy.loginfo(goals)
    return goals


class Goal(geoutil.TargetPlace):
    def __init__(self, delegate, **kwargs):
        super(Goal, self).__init__(**kwargs)
        if delegate is None:
            raise RuntimeError("please provide proper delegate object")
        self.delegate = delegate
        self.is_last = kwargs['is_last'] if 'is_last' in kwargs else False
        self._ready_to_execute = False
        self._is_completed = False
        self._is_canceled = False
        self._current_statement = None
        self.global_map_name = self.delegate.global_map_name()

    def enter(self):
        self._is_completed = False
        self._is_canceled = False
        self.delegate.enter_goal(self)

    def check(self, current_pose):
        pass

    def update_goal(self, goal):
        pass

    @property
    def is_completed(self):
        return self._is_completed

    @property
    def is_canceled(self):
        return self._is_canceled

    @property
    def need_to_announce_arrival(self):
        return False

    def exit(self):
        self.delegate.exit_goal(self)

    @property
    def current_statement(self):
        return self._current_statement

    def __str__(self):
        ret = "%s, (%s)\n" % (type(self), hex(id(self)))
        for key in self.__dict__:
            value = getattr(self, key)
            ret += "%s: %s\n"%(key, str(value))
        
        import inspect
        for method in inspect.getmembers(type(self), predicate=lambda o: isinstance(o, property)):
            key = method[0]
            value = method[1].__get__(self, type(self))
            ret += "%s: %s\n"%(key, str(value))

        return ret

    def __repr__(self):
        return "%s<%s>"%(type(self), super(Goal, self).__repr__())


class NavGoal(Goal):
    DEFAULT_BT_XML = "package://cabot_bt/behavior_trees/navigate_w_replanning_and_recovery.xml"

    def __init__(self, delegate, navcog_route, anchor, target_poi=None, set_back=(0, 0), **kwargs):
        if navcog_route is None or len(navcog_route) == 0:
            raise RuntimeError("navcog_route should have one more object")
        if anchor is None:
            raise RuntimeError("anchor should be provided")

        # need init global_map_name for initialization
        self.global_map_name = delegate.global_map_name()
        self.navcog_route = navcog_route
        self.anchor = anchor
        (self.ros_path, last_pose) = self._create_ros_path()
        self.pois = self._extract_pois()

        floor = None
        last_obj = navcog_route[-1]
        if isinstance(last_obj, geojson.Node):
            floor = last_obj.floor
        elif isinstance(last_obj, geojson.RouteLink):
            floor = last_obj.end_node.floor


        # if target_poi is specified, the last_pose will be the target_poi
        if target_poi is not None:
            last_pose = target_poi.to_pose_msg()

            # if set_back is specified too, move the last_pose based on the last link direction
            if set_back[0] > 0 or set_back[1] > 0:
                i = 1
                while isinstance(last_obj, geojson.Node) and i < len(navcog_route):
                    i += 1
                    last_obj = navcog_route[-i]
                if not isinstance(last_obj, geojson.RouteLink):
                    raise RuntimeError("There should be at least one link towards target POI,"
                                       "if it is provided with set_back (={}) \n===POI===\n{}\n========="
                                       .format(set_back, str(target_poi)))

                backward = geoutil.Pose.pose_from_points(last_obj.source_node.local_geometry,
                                                         last_obj.target_node.local_geometry)

                rospy.loginfo("set_back %.2f * (%s)", backward.r, str(set_back))
                
                last_pose.position.x += math.cos(backward.r) * set_back[0] - math.sin(backward.r) * set_back[1]
                last_pose.position.y += math.sin(backward.r) * set_back[0] + math.cos(backward.r) * set_back[1]

        super(NavGoal, self).__init__(delegate, angle=180, floor=floor, pose_msg=last_pose, **kwargs)
        
        self._need_to_announce_arrival = self.is_last
        if 'need_to_announce_arrival' in kwargs:
            self._need_to_announce_arrival = bool(kwargs['need_to_announce_arrival'])

        self._goal_name_pron = None
        self._goal_description = None
        if self._need_to_announce_arrival:
            if not isinstance(navcog_route[-1], geojson.Node):
                rospy.loginfo(navcog_route[-1])
                return
            rospy.loginfo(str(navcog_route[-1]))
            if not navcog_route[-1].facility:
                return
            
            if navcog_route[-1].facility.name_pron:
                self._goal_name_pron = navcog_route[-1].facility.name_pron
            else:
                self._goal_name_pron = navcog_route[-1].facility.name
            self._goal_description = navcog_route[-1].facility.long_description
            if self.goal_name_pron:
                rospy.loginfo("%s", self.goal_name_pron.encode('utf-8'))
            if self.goal_description:
                rospy.loginfo("%s", self.goal_description.encode('utf-8'))

    @property
    def need_to_announce_arrival(self):
        return self._need_to_announce_arrival

    @property
    def goal_name_pron(self):
        return self._goal_name_pron

    @property
    def goal_description(self):
        return self._goal_description

    def _create_ros_path(self):
        """convert a NavCog path to ROS path"""

        # convert route to points
        points = []
        rospy.loginfo("create_ros_path")
        for (index, item) in enumerate(self.navcog_route[:-1]):
            if isinstance(item, geojson.RouteLink):
                if item.is_leaf and item.length < 3.0:
                    continue
            convert = lambda g, a=self.anchor: geoutil.global2local(g, a)

            if isinstance(item.geometry, geojson.Point):
                points.append(convert(item.geometry))
            elif isinstance(item.geometry, geojson.LineString):
                if item.target_node is None:
                    print("item-------------")
                    print(item)
                points.append(convert(item.target_node.geometry))
            else:
                rospy.loginfo("geometry is not point or linestring {}".format(item.geometry))
            rospy.loginfo("%d: %s", index, str(points[-1]))
    

        # make a path from points
        path = nav_msgs.msg.Path()
        path.header.frame_id = "map"
        path.poses = []
        quat = None
        pori = None
        for i in range(0, len(points)):
            start = points[i]
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = self.global_map_name
            pose.pose = geometry_msgs.msg.Pose()
            pose.pose.position.x = start.x
            pose.pose.position.y = start.y
            pose.pose.position.z = 0
            if i+1 < len(points):
                end = points[i+1]
                direction = math.atan2(end.y - start.y, end.x - start.x)
                quat = tf.transformations.quaternion_from_euler(0, 0, direction)

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                path.poses.append(pose)                
                pori = pose.pose.orientation
            else:
                pose.pose.orientation = pori
                path.poses.append(pose)                

        rospy.loginfo("path %s", path)
        return (path, path.poses[-1])

    def _extract_pois(self):
        """extract pois along the route"""
        temp = []
        for (_, item) in enumerate(self.navcog_route):
            if isinstance(item, geojson.RouteLink):
                print(item._id)
                for poi in item.pois:
                    print("  ", type(poi), poi._id)
                temp.extend(item.pois)
        return temp

    def enter(self):
        rospy.loginfo("NavGoal enter")
        # reset social distance setting if necessary
        if self.delegate.initial_social_distance is not None and self.delegate.current_social_distance is not None \
            and (self.delegate.initial_social_distance.x!=self.delegate.current_social_distance.x \
                or self.delegate.initial_social_distance.y!=self.delegate.current_social_distance.y):
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)
        
        rospy.loginfo("NavGoal set social distance")
        # publish navcog path
        path = self.ros_path
        path.header.stamp = rospy.get_rostime()
        self.delegate.publish_path(path)
        rospy.loginfo("NavGoal publish path")
        super(NavGoal, self).enter()

        # wanted a path (not only a pose) in planner plugin, but it is not possible
        # bt_navigator will path only a pair of consecutive poses in the path to the plugin
        # so we use navigate_to_pose and planner will listen the published path
        # self.delegate.navigate_through_poses(self.ros_path.poses[1:], NavGoal.DEFAULT_BT_XML, self.done_callback)
        self.delegate.navigate_to_pose(self.ros_path.poses[-1], NavGoal.DEFAULT_BT_XML, self.done_callback)

    def done_callback(self, status, result):
        rospy.loginfo("NavGoal completed status={} result={}".format(status, result))
        self._is_completed = (status == GoalStatus.SUCCEEDED)
        self._is_canceled = (status != GoalStatus.SUCCEEDED)

    def update_goal(self, goal):
        rospy.loginfo("Updated goal position")
        #self.delegate.send_goal(goal, self.done_callback)

class TurnGoal(Goal):
    def __init__(self, delegate, anchor, yaw, **kwargs):
        self.yaw_target = (- yaw + 90 + anchor.rotate) / 180.0 * math.pi
        rospy.loginfo("TurnGoal {} {} {}".format(yaw, anchor.rotate, self.yaw_target))
        pose = geoutil.Pose(x=0, y=0, r=self.yaw_target)
        pose._angle = 0 ## dummy
        pose._floor = 0 ## dummy
        super(TurnGoal, self).__init__(delegate, target=pose, **kwargs)

    def enter(self):
        super(TurnGoal, self).enter()
        rospy.loginfo("call turn_towards")
        rospy.loginfo("turn target %s", str(self.orientation))
        self.delegate.turn_towards(self.orientation, self.done_callback)

    def done_callback(self, status, result):
        rospy.loginfo("TurnGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)

class DoorGoal(Goal):
    def __init__(self, delegate, poi):
        super(DoorGoal, self).__init__(delegate, target=poi)

    def enter(self):
        self.delegate.please_pass_door()
        self.delegate.set_clutch(False)
        self.delegate.enter_goal(self)

    def check(self, current_pose):
        if self.is_approaching(current_pose):
            return
        elif self.is_approached(current_pose):
            return
        elif self.is_passed(current_pose):
            self._is_completed = True

    def exit(self):
        self.delegate.door_passed()
        super(DoorGoal, self).exit()

class ElevatorGoal(Goal):
    # using odom frame
    ELEVATOR_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_elevator.xml"
    LOCAL_ODOM_BT_XML = "package://cabot_bt/behavior_trees/navigate_w_local_odom.xml"

    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorGoal, self).__init__(delegate, target=cab_poi, **kwargs)
        self.cab_poi = cab_poi

class ElevatorWaitGoal(ElevatorGoal):
    ELEVATOR_SOCIAL_DISTANCE_X = 0.0
    ELEVATOR_SOCIAL_DISTANCE_Y = 0.0

    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorWaitGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        # change social distance setting
        msg = geometry_msgs.msg.Point()
        msg.x = self.ELEVATOR_SOCIAL_DISTANCE_X
        msg.y = self.ELEVATOR_SOCIAL_DISTANCE_X
        self.delegate.set_social_distance_pub.publish(msg)

        super(ElevatorWaitGoal, self).enter()
        
        rospy.loginfo("call turn_towards")
        rospy.loginfo("current pose"+str(self.delegate.current_pose))
        rospy.loginfo("cab poi     "+str(self.cab_poi.local_geometry))
        pose = geoutil.Pose.pose_from_points(self.cab_poi.door_geometry, self.delegate.current_pose)
        rospy.loginfo("turn target %s", str(pose))
        self.delegate.turn_towards(pose.orientation, self.done_callback)

    def done_callback(self, status, result):
        pos = self.cab_poi.where_is_buttons(self.delegate.current_pose)
        self.delegate.please_call_elevator(pos)
        self._is_completed = (status == GoalStatus.SUCCEEDED)

class ElevatorInGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorInGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorInGoal, self).enter()
        # use odom frame for navigation
        self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), ElevatorGoal.ELEVATOR_BT_XML, self.done_callback)

    def done_callback(self, status, result):
        rospy.loginfo("ElevatorInGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)


class ElevatorTurnGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorTurnGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorTurnGoal, self).enter()
        rospy.loginfo("call turn_towards")
        pose = geoutil.Pose(x=self.cab_poi.x,y=self.cab_poi.y,r=self.cab_poi.r)
        rospy.loginfo("turn target %s", str(pose))
        self.delegate.turn_towards(pose.orientation, self.done_callback, clockwise=-1)

    def done_callback(self, status, result):
        rospy.loginfo("ElevatorTurnGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)


class ElevatorFloorGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, **kwargs):
        super(ElevatorFloorGoal, self).__init__(delegate, cab_poi, **kwargs)

    def enter(self):
        super(ElevatorFloorGoal, self).enter()
        self.delegate.goto_floor(self.cab_poi.floor, self.done_callback)
        
    def done_callback(self, status, result):
        rospy.loginfo("ElevatorFloorGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)


class ElevatorOutGoal(ElevatorGoal):
    def __init__(self, delegate, cab_poi, set_forward=(3.0, 0.0), **kwargs):
        super(ElevatorOutGoal, self).__init__(delegate, cab_poi, **kwargs)
        self.set_forward = set_forward

    def enter(self):
        super(ElevatorOutGoal, self).enter()
        rospy.loginfo("ElevatorOutGoal enter")
        pose = self.delegate.current_odom_pose
        
        start = geometry_msgs.msg.PoseStamped()
        start.header.frame_id = "local/odom"
        start.pose = pose.to_pose_msg()
        
        rospy.loginfo("Robot position in odom "+str(pose))

        pose.x = pose.x + math.cos(pose.r) * self.set_forward[0] - math.sin(pose.r) * self.set_forward[1]
        pose.y = pose.y + math.sin(pose.r) * self.set_forward[0] + math.cos(pose.r) * self.set_forward[1]
        
        rospy.loginfo("goal is "+str(pose))
        
        end = geometry_msgs.msg.PoseStamped()
        end.header.frame_id = "local/odom"
        end.pose = pose.to_pose_msg()
        path = nav_msgs.msg.Path()
        path.header.frame_id = "local/odom"
        path.poses = [start, end]
        rospy.loginfo("publish path "+str(path))
        self.delegate.publish_path(path, False)

        self.delegate.navigate_to_pose(end, ElevatorGoal.LOCAL_ODOM_BT_XML, self.done_callback, namespace='/local')

    def done_callback(self, status, result):
        rospy.loginfo("ElevatorOutGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)

        # reset social distance setting
        if self.delegate.initial_social_distance is not None:
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)

'''
TODO

class EscalatorInGoal(Goal):
    def __init__(self, delegate, links):
        # find a palce before the elevator
        remain = 2.0
        last_link = None
        for link in reversed(links):
            if remain - link.length < 0:
                last_link = link
                break
            remain -= link.length

        self.last_link = last_link
        ratio = remain / last_link.length
        sg = last_link.start_node.local_geometry
        eg = last_link.end_node.local_geometry
        x = sg.x * ratio + eg.x * (1-ratio)
        y = sg.y * ratio + eg.y * (1-ratio)

        print (remain, last_link.length, ratio)
        print sg
        print eg
        print (x, y)
        
        super(EscalatorInGoal, self).__init__(delegate, angle=0, floor=last_link.floor, x=x, y=y, r=0)

class EscalatorOutGoal(Goal):
    def __init__(self, links):
        pass
'''


# create multiple goals to queue target node
def make_queue_goals(delegate, queue_route, anchor, is_entering):
    is_exiting = not is_entering

    goals = []
    for queue_r_idx, queue_r in enumerate(queue_route):
        if queue_r_idx==0 or queue_r_idx==len(queue_route)-1:
            # skip node
            continue
        else:
            # find navigation start, end nodes
            if queue_r_idx==1:
                queue_r_start = queue_route[0]
            else:
                queue_r_start = queue_route[queue_r_idx-1].end_node
            if queue_r_idx==len(queue_route)-2:
                queue_r_end = queue_route[queue_r_idx+1]
            else:
                queue_r_end = queue_route[queue_r_idx+1].start_node
            # add goal to turn for goal
            queue_r_end_pose = geoutil.Pose.pose_from_points(queue_r_end.local_geometry, queue_r_start.local_geometry)
            goals.append(QueueTurnGoal(delegate, queue_r.floor, queue_r_end_pose))

            # check if next goal is queue target node (e.g. cashier in store)
            is_target = False
            if queue_r_idx==len(queue_route)-2 and is_entering:
                is_target = True

            # check if link has wait POIs, and get interval if it exists
            queue_interval = None
            queue_waits = list(filter(lambda x:isinstance(x, geojson.QueueWaitPOI), queue_r.pois))
            if queue_waits and len(queue_waits) >= 1:
                queue_interval = queue_waits[0].interval

            # add navigation goal
            if queue_r_idx==1 and is_entering:
                goals.append(QueueNavFirstGoal(delegate, [queue_r_start, queue_r, queue_r_end], anchor, queue_interval, is_target))
            elif queue_r_idx==len(queue_route)-2 and is_exiting:
                goals.append(QueueNavLastGoal(delegate, [queue_r_start, queue_r, queue_r_end], anchor, queue_interval))
            else:
                goals.append(QueueNavGoal(delegate, [queue_r_start, queue_r, queue_r_end], anchor, queue_interval, is_target, is_exiting))

    return goals


# current code assumes queue exit route is narrow, and different BT XML is used for changing footprint
class QueueNavGoal(NavGoal):
    QUEUE_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue.xml"
    QUEUE_EXIT_BT_XML = "package://cabot_bt/behavior_trees/navigate_for_queue_exit.xml"

    def __init__(self, delegate, navcog_route, anchor, queue_interval, is_target, is_exiting, **kwargs):
        # set is_last as True if the goal is queue target goal or queue last goal for annoucing arrival information
        super(QueueNavGoal, self).__init__(delegate, navcog_route, anchor, is_last=(is_target or isinstance(self, QueueNavLastGoal)), **kwargs)
        self.queue_interval = queue_interval
        self.is_target = is_target
        self.is_exiting = is_exiting

    def enter(self):
        # change queue_interval setting
        if self.queue_interval is not None:
            self.delegate.current_queue_interval = self.queue_interval

        # publish navcog path
        path = self.ros_path
        path.header.stamp = rospy.get_rostime()
        self.delegate.publish_path(path)
        super(NavGoal, self).enter()
        if self.is_exiting:
            self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), QueueNavGoal.QUEUE_EXIT_BT_XML, self.done_callback)
        else:
            self.delegate.navigate_to_pose(self.to_pose_stamped_msg(frame_id=self.global_map_name), QueueNavGoal.QUEUE_BT_XML, self.done_callback)

    def done_callback(self, status, result):
        self._is_completed = (status == GoalStatus.SUCCEEDED)
        if self._is_completed and self.is_target:
            # If robot arrive queue target (e.g. cashier), sleep for a while.
            # User will release handle and robot will stop.
            rospy.sleep(10)

class QueueTurnGoal(Goal):
    def __init__(self, delegate, floor, pose, **kwargs):
        super(QueueTurnGoal, self).__init__(delegate, angle=180, floor=floor, pose_msg=pose.to_pose_msg(), **kwargs)
        self.target_orientation = pose.orientation

    def enter(self):
        super(QueueTurnGoal, self).enter()
        rospy.loginfo("QueueTurnGoal turn_towards, target %s", str(self.target_orientation))
        self.delegate.turn_towards(self.target_orientation, self.done_callback)

    def done_callback(self, status, result):
        rospy.loginfo("QueueTurnGoal completed")
        self._is_completed = (status == GoalStatus.SUCCEEDED)


class QueueNavFirstGoal(QueueNavGoal):
    QUEUE_SOCIAL_DISTANCE_X = 0.8
    QUEUE_SOCIAL_DISTANCE_Y = 0.8

    def __init__(self, delegate, navcog_route, anchor, queue_interval, is_target, **kwargs):
        super(QueueNavFirstGoal, self).__init__(delegate, navcog_route, anchor, queue_interval, is_target, is_exiting=False, **kwargs)

    def enter(self):
        # change social distance setting
        msg = geometry_msgs.msg.Point()
        msg.x = self.QUEUE_SOCIAL_DISTANCE_X
        msg.y = self.QUEUE_SOCIAL_DISTANCE_Y
        self.delegate.set_social_distance_pub.publish(msg)

        # initialize flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = True
        self.delegate.need_queue_proceed_info = False

        super(QueueNavFirstGoal, self).enter()


class QueueNavLastGoal(QueueNavGoal):
    def __init__(self, delegate, navcog_route, anchor, **kwargs):
        super(QueueNavLastGoal, self).__init__(delegate, navcog_route, anchor, queue_interval=None, is_target=False, is_exiting=True, **kwargs)
    
    def done_callback(self, status, result):
        super(QueueNavLastGoal, self).done_callback(status, result)
        
        # reset social distance setting
        if self.delegate.initial_social_distance is not None:
            msg = self.delegate.initial_social_distance
            self.delegate.set_social_distance_pub.publish(msg)

        # reset queue_interval setting
        if self.delegate.initial_queue_interval is not None:
            self.delegate.current_queue_interval = self.delegate.initial_queue_interval

        # reset flag to call queue start arrived info and queue proceed info
        self.delegate.need_queue_start_arrived_info = False
        self.delegate.need_queue_proceed_info = False
