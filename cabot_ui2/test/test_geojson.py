#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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
"""Test geojson module"""

import json
import unittest

from ament_index_python.packages import get_package_share_directory
from cabot_ui import geojson, geoutil, navgoal


class TestGeojson(unittest.TestCase):
    """Test class"""

    def setUp(self):
        self.path = get_package_share_directory('cabot_ui')

    def test_geometry(self):
        """test geometry marshalling"""
        text = """{"coordinates":[[-79.9456263990628,40.44335090952231],[-79.94568007928659,40.44335962963177]],"type":"LineString"}"""  # noqa E502
        dic = json.loads(text)
        obj = geojson.Geometry.marshal(dic)
        self.assertTrue(isinstance(obj, geojson.LineString))
        self.assertAlmostEqual(obj.start.lat, 40.44335090952231)
        self.assertAlmostEqual(obj.end.lng, -79.94568007928659)

        text = """{"coordinates":[-79.9461898503219,40.442684275499886],"type":"Point"}"""
        dic = json.loads(text)
        obj = geojson.Geometry.marshal(dic)
        self.assertTrue(isinstance(obj, geojson.Point))
        self.assertAlmostEqual(obj.lat, 40.442684275499886)
        self.assertAlmostEqual(obj.lng, -79.9461898503219)

    def test_link(self):
        """test link marshalling"""
        text = """{"geometry":{"coordinates":[[-79.9456263990628,40.44335090952231],[-79.94568007928659,40.44335962963177]],"type":"LineString"},"_id":"EDITOR_link_1498230276608","type":"Feature","properties":{"roof":1,"handrail":1,"hulop_file":"EDITOR","distance":4.7,"start_id":"EDITOR_node_1490112640587","tfc_restr":1,"vtcl_slope":1,"waterway":1,"end_id":"EDITOR_node_1490911862119","tfc_s_type":99,"brail_tile":99,"vSlope_max":0,"link_id":"EDITOR_link_1498230276608","condition":1,"tfc_signal":99,"elevator":99,"rt_struct":7,"route_type":1,"width":99,"hSlope_max":0,"main_user":1,"lev_diff":1,"direction":1}}"""  # noqa E502

        dic = json.loads(text)
        obj = geojson.Object.marshal(dic)
        self.assertTrue(isinstance(obj, geojson.Link))
        self.assertEqual(obj._id, "EDITOR_link_1498230276608")
        self.assertEqual(obj.properties.roof, 1)

    def test_node(self):
        """test node marshalling"""
        text = """{"geometry":{"coordinates":[-79.9461898503219,40.442684275499886],"type":"Point"},"_id":"EDITOR_node_1498966657552","type":"Feature","properties":{"hulop_file":"EDITOR","in_out":3,"lon":-79.9461898503219,"floor":8,"link1_id":"EDITOR_link_1498968162649","lat":40.442684275499886,"node_id":"EDITOR_node_1498966657552"}}"""  # noqa E502

        dic = json.loads(text)
        obj = geojson.Object.marshal(dic)
        self.assertTrue(isinstance(obj, geojson.Node))
        self.assertEqual(obj._id, "EDITOR_node_1498966657552")
        self.assertEqual(obj.properties.floor, 8)

    def test_facility(self):
        """test facility marshalling"""
        text = """{"geometry":{"coordinates":[-79.94564917391125,40.443339374307556],"type":"Point"},"_id":"EDITOR_poi_1496169882162","type":"Feature","properties":{"parking":99,"hulop_height":4,"address":"","escalator":99,"hulop_file":"EDITOR","name_ja":"","nursing":99,"brail_tile":99,"lon":-79.94564917391125,"hulop_sub_category":"_nav_obstacle_","facil_id":"EDITOR_poi_1496169882162","toilet":99,"hulop_minor_category":"recycle bin","elevator":99,"barrier":99,"hulop_major_category":"_nav_poi_","tel":"","facil_type":99,"lat":40.443339374307556,"name_en":""}}"""  # noqa E502

        dic = json.loads(text)
        obj = geojson.Object.marshal(dic)
        self.assertTrue(isinstance(obj, geojson.Facility))
        self.assertEqual(obj._id, "EDITOR_poi_1496169882162")
        self.assertEqual(obj.properties.parking, 99)

    def test_elevator(self):
        text = """{"geometry":{"coordinates":[[-79.9456263990628,40.44335090952231],[-79.94568007928659,40.44335962963177]],"type":"LineString"},"_id":"EDITOR_link_1498230276608","type":"Feature","properties":{"roof":1,"handrail":1,"hulop_file":"EDITOR","distance":4.7,"start_id":"EDITOR_node_1490112640587","tfc_restr":1,"vtcl_slope":1,"waterway":1,"end_id":"EDITOR_node_1490911862119","tfc_s_type":99,"brail_tile":99,"vSlope_max":0,"link_id":"EDITOR_link_1498230276608","condition":1,"tfc_signal":99,"elevator":99,"rt_struct":7,"route_type":4,"width":99,"hSlope_max":0,"main_user":1,"lev_diff":1,"direction":1}}"""  # noqa E502

        dic = json.loads(text)
        obj = geojson.Object.marshal(dic)
        self.assertTrue(obj.is_elevator)

    def test_escalator(self):
        text = """{"geometry":{"coordinates":[[-79.9456263990628,40.44335090952231],[-79.94568007928659,40.44335962963177]],"type":"LineString"},"_id":"EDITOR_link_1498230276608","type":"Feature","properties":{"roof":1,"handrail":1,"hulop_file":"EDITOR","distance":4.7,"start_id":"EDITOR_node_1490112640587","tfc_restr":1,"vtcl_slope":1,"waterway":1,"end_id":"EDITOR_node_1490911862119","tfc_s_type":99,"brail_tile":99,"vSlope_max":0,"link_id":"EDITOR_link_1498230276608","condition":1,"tfc_signal":99,"elevator":99,"rt_struct":7,"route_type":5,"width":99,"hSlope_max":0,"main_user":1,"lev_diff":1,"direction":1}}"""  # noqa E502

        dic = json.loads(text)
        obj = geojson.Object.marshal(dic)
        self.assertTrue(obj.is_escalator)

    def test_list(self):
        text = """[{"geometry":{"coordinates":[-79.9461898503219,40.442684275499886],"type":"Point"},"_id":"EDITOR_node_1498966657552","type":"Feature","properties":{"hulop_file":"EDITOR","in_out":3,"lon":-79.9461898503219,"floor":8,"link1_id":"EDITOR_link_1498968162649","lat":40.442684275499886,"node_id":"EDITOR_node_1498966657552"}},{"geometry":{"coordinates":[-79.9461898503219,40.442684275499886],"type":"Point"},"_id":"EDITOR_node_1498966657552","type":"Feature","properties":{"hulop_file":"EDITOR","in_out":3,"lon":-79.9461898503219,"floor":8,"link1_id":"EDITOR_link_1498968162649","lat":40.442684275499886,"node_id":"EDITOR_node_1498966657552"}}]"""  # noqa E502
        list_ = json.loads(text)
        obj = geojson.Object.marshal_list(list_)
        self.assertIsNotNone(obj[0])
        self.assertIsNotNone(obj[1])

    def _prepare_data(self):
        import os
        dir_path = os.path.dirname(os.path.realpath(__file__))

        node_map = open(dir_path+"/data/node_map1.json")
        _ = geojson.Object.marshal_dict(json.load(node_map))
        features = open(dir_path+"/data/features1.json")
        _ = geojson.Object.marshal_list(json.load(features))

        route = open(dir_path+"/data/route2.json")
        self.route = geojson.Object.marshal_list(json.load(route))

        self.anc = geoutil.Anchor(lat=40.443259,
                                  lng=-79.945874,
                                  rotate=15.1)

        for obj in geojson.Object.get_all_objects():
            obj.reset()
            obj.update_anchor(self.anc)

    def test_door_poi(self):
        """test door poi marshalling"""
        self._prepare_data()

        obj = geojson.Object.get_object_by_id("EDITOR_facil_1547762317242")
        self.assertIsInstance(obj, geojson.DoorPOI)

        targetX = int(obj.local_geometry.x*10)
        for x in range(0, targetX):
            pose = geoutil.Pose(x=x/10.0, y=0.0, r=0.0)
            obj.is_approaching(pose)
            obj.is_approached(pose)
            obj.is_passed(pose)
        self.assertEqual(obj._was_approaching, True)
        self.assertEqual(obj._was_approached, True)
        self.assertEqual(obj._was_passed, False)

        for x in range(targetX, targetX+20):
            pose = geoutil.Pose(x=x/10.0, y=0.0, r=0.0)
            obj.is_approaching(pose)
            obj.is_approached(pose)
            obj.is_passed(pose)
        self.assertEqual(obj._was_approaching, True)
        self.assertEqual(obj._was_approached, True)
        self.assertEqual(obj._was_passed, True)

        geojson.Object.reset_all_objects()

        self.assertEqual(obj._was_approaching, False)
        self.assertEqual(obj._was_approached, False)
        self.assertEqual(obj._was_passed, False)

    def test_is_leaf(self):
        self._prepare_data()
        link = geojson.Object.get_object_by_id("EDITOR_link_1490207604690")
        self.assertIsNotNone(link)
        self.assertEqual(link.is_leaf, True)

    def test_directional_poi(self):
        self._prepare_data()
        poi_f = geojson.Object.get_object_by_id("EDITOR_facil_1547762317242")
        poi_b = geojson.Object.get_object_by_id("EDITOR_poi_1495221059522")

        self.assertIsNotNone(poi_f)
        self.assertIsNotNone(poi_b)

        targetX = int(poi_f.local_geometry.x*10)
        for x in range(0, targetX):
            pose = geoutil.Pose(x=x/10.0, y=0.0, r=0.0)
            poi_f.is_approaching(pose)
            poi_f.is_approached(pose)
            poi_b.is_approaching(pose)
            poi_b.is_approached(pose)

        self.assertEqual(poi_f._was_approaching, True)
        self.assertEqual(poi_f._was_approached, True)
        self.assertFalse(poi_b._was_approaching)
        self.assertFalse(poi_b._was_approached)

    def global_map_name(self):
        return "map"

    def test_doorgoal(self):
        self._prepare_data()
        door = geojson.Object.get_object_by_id("EDITOR_facil_1547762317242")
        goal = navgoal.DoorGoal(self, door)

        self.assertIsNotNone(goal)

        targetX = int(goal.x*10)
        for x in range(0, targetX):
            pose = geoutil.Pose(x=x/10.0, y=0.0, r=0.0)
            goal.is_approaching(pose)
            goal.is_approached(pose)

        self.assertEqual(goal._was_approaching, True)
        self.assertEqual(goal._was_approached, True)
