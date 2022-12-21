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

"""
Data Utility

Author: Daisuke Sato<daisukes@cmu.edu>
"""

import time
import json

from rclpy.node import Node
import requests
from cabot_ui import geojson
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class DataUtil(object):
    """Data Utility class"""

    SEARCH_API = "routesearch"

    def __init__(self, node: Node):
        self._node = node
        self._protocol = node.declare_parameter("protocol", "http").value
        self._hostname = node.declare_parameter("map_server_host", "").value
        if not node.has_parameter("lang"):
            self._lang = node.declare_parameter("lang", "en").value
        else:
            self._lang = node.get_parameter("lang").value
        self._dist = node.declare_parameter("lookup_dist", 1000).value

        if not self._hostname:
            CaBotRclpyUtil.warn("'hostname' should be specified")

        self._user = 'Cabot'
        self._anchor = None
        self._initialized = False
        self._latitude = None
        self._longitude = None

        # public data
        self.landmarks = None
        self.node_map = None
        self.features = None
        self.current_route = None
        self.is_ready = False
        self.is_analyzed = False

    def _update(self):
        geojson.Object.update_anchor_all(self._anchor)
        self.analyze_features()

    def set_anchor(self, anchor):
        self._anchor = anchor
        self._latitude = anchor.lat
        self._longitude = anchor.lng
        self.is_analyzed = False
        CaBotRclpyUtil.info(F"set_anchor ({self._latitude},{self._longitude})")
        self._update()

    def get_search_url(self):
        """get the URL for search api"""
        url = F"{self._protocol}://{self._hostname}/{self.SEARCH_API}"
        CaBotRclpyUtil.info(url)
        return url

    def init_by_server(self, retry_count=100):
        """initialize server state for a user"""
        if self.is_ready:
            return
        CaBotRclpyUtil.info("init server")
        try:
            self.init_by_data(landmarks=self.get_landmarks(),
                              node_map=self.get_node_map(),
                              features=self.get_features())
        except RuntimeError as err:
            CaBotRclpyUtil.info(F"{err}")
            if retry_count <= 0:
                return
            time.sleep(5)
            CaBotRclpyUtil.info(F"retrying to init server {retry_count})")
            self.init_by_server(retry_count=retry_count-1)

    def init_by_data(self, landmarks=[], node_map={}, features=[]):
        """initialize datautil with data"""
        CaBotRclpyUtil.info("init_by_data")
        if len(node_map) == 0 or len(features) == 0:
            raise RuntimeError("No node or features on the server") from None
        self.landmarks = landmarks
        self.node_map = node_map
        self.features = features
        self.is_ready = True

    def get_landmarks(self, filename=None):
        """get landmarks"""
        if filename is None:
            try:
                data={
                    'action': 'start',
                    'user': self._user,
                    'lang': self._lang,
                    'lat': self._latitude,
                    'lng': self._longitude,
                    'dist': self._dist,
                }
                req = requests.post(self.get_search_url(), data=data)
            except Exception:
                raise RuntimeError(F'could not send request to get landmarks {data}') from None
            if req.status_code != requests.codes.ok:
                raise RuntimeError('could not initialize landmarks') from None
            data = json.loads(req.text)
        else:
            data = json.load(open(filename))

        import tempfile
        f = open(tempfile.gettempdir()+"/landmarks.json", "w")
        f.write(json.dumps(data, indent=4))
        f.close()

        if 'landmarks' in data:
            self._initialized = True
            return geojson.Object.marshal_list(data['landmarks'])
        return None

    def get_node_map(self, filename=None):
        """get node map"""
        if not self._initialized:
            raise RuntimeError("server is not initialized")

        if filename is None:
            try:
                req = requests.post(self.get_search_url(), data={
                    'action': 'nodemap',
                    'user': self._user,
                    'lang': self._lang
                })
            except Exception:
                raise RuntimeError('could not send request to get node map') from None
            if req.status_code != requests.codes.ok:
                raise RuntimeError('could not initialize node map') from None
            data = json.loads(req.text)
        else:
            data = json.load(open(filename))

        import tempfile
        f = open(tempfile.gettempdir()+"/nodemap.json", "w")
        f.write(json.dumps(data, indent=4))
        f.close()

        return geojson.Object.marshal_dict(data)

    def get_features(self, filename=None):
        """get features (links and pois)"""
        if not self._initialized:
            raise RuntimeError("server is not initialized")

        if filename is None:
            try:
                req = requests.post(self.get_search_url(), data={
                    'action': 'features',
                    'user': self._user,
                    'lang': self._lang
                })
            except Exception:
                raise RuntimeError('could not send request to get features') from None
            if req.status_code != requests.codes.ok:
                raise RuntimeError('could not initialize features') from None
            data = json.loads(req.text)
        else:
            data = json.load(open(filename))

        import tempfile

        f = open(tempfile.gettempdir()+"/features.json", "w")
        f.write(json.dumps(data, indent=4))
        f.close()

        return geojson.Object.marshal_list(data)

    def analyze_features(self):
        CaBotRclpyUtil.info(F"analyze_features {self.is_ready}, {self.is_analyzed}")
        if not self.is_ready:
            return
        if self.is_analyzed:
            return
        CaBotRclpyUtil.info("analyzing features")
        # links = geojson.Object.get_objects_by_type(geojson.Link)
        doors = geojson.Object.get_objects_by_type(geojson.DoorPOI)
        infos = geojson.Object.get_objects_by_type(geojson.InfoPOI)
        speeds = geojson.Object.get_objects_by_type(geojson.SpeedPOI)

        for poi in doors+infos+speeds:
            min_link = geojson.Object.get_nearest_link(poi)
            if min_link is None:
                CaBotRclpyUtil.debug(F"poi {poi._id} ({poi.floor}) is not registered.")
                continue
            min_dist = min_link.geometry.distance_to(poi.geometry)

            if min_dist < 5:
                min_link.register_poi(poi)
            else:
                CaBotRclpyUtil.debug(
                    F"poi {poi._id} ({poi.floor}) is not registered. "
                    F"min_link._id = {min_link._id}, min_link.floor = {min_link.floor}")

        elevator_cabs = geojson.Object.get_objects_by_type(geojson.ElevatorCabPOI)
        for poi in elevator_cabs:
            min_link = geojson.Object.get_nearest_link(poi, exclude=lambda x: x.is_elevator)
            min_dist = min_link.geometry.distance_to(poi.geometry)

            if min_dist < 5:
                min_link.register_poi(poi)
            else:
                CaBotRclpyUtil.debug(
                    F"poi {poi._id} ({poi.floor}) is not registered. "
                    F"min_link._id = {min_link._id}, min_link.floor = {min_link.floor}")

        queue_waits = geojson.Object.get_objects_by_type(geojson.QueueWaitPOI)
        queue_targets = geojson.Object.get_objects_by_type(geojson.QueueTargetPOI)
        for poi in queue_waits+queue_targets:
            min_link = geojson.Object.get_nearest_link(poi)
            if min_link is None:
                CaBotRclpyUtil.debug(
                    F"poi {poi._id} ({poi.floor}) is not registered. ")
                continue
            min_dist = min_link.geometry.distance_to(poi.geometry)

            if min_dist < 5:
                min_link.register_poi(poi)
            else:
                CaBotRclpyUtil.debug(
                    F"poi {poi._id} ({poi.floor}) is not registered. "
                    F"min_link._id = {min_link._id}, min_link.floor = {min_link.floor}")

        self.is_analyzed = True

    def _update_anchor(self):
        for obj in geojson.Object.get_all_objects():
            obj.update_anchor(self._anchor)

    def clear_route(self):
        self.current_route = None

    def get_route(self, from_id, to_id):
        """get NavCog route from 'from_id' to 'to_id'"""
        if from_id is None or to_id is None:
            raise RuntimeError("from_id and to_id should not be None")
        CaBotRclpyUtil.info(F"request route from {from_id} to {to_id}")
        req = requests.post(self.get_search_url(), data={
            'action': 'search',
            'user': self._user,
            'lang': self._lang,
            'from': from_id,
            'to': to_id,
            'preferences': json.dumps({
                'preset': '9',
                'min_width': '8',
                'dist': 2000,
                'stairs': '9',
                'tactile_paving': '1',  # not prefer tactile paving
                'mvw': '1',             # do not use moving walkway
                'slope': '9',
                'deff_LV': '9',
                'elv': '9',
                'road_condition': '9',
                'esc': '1'              # do not use escalator
            })
        })
        CaBotRclpyUtil.info(F"get data {len(req.text)} bytes")

        jfeatures = json.loads(req.text)
        import tempfile
        f = open(F"{tempfile.gettempdir()}/route_{from_id}_{to_id}", "w")
        f.write(json.dumps(jfeatures, indent=4))
        f.close()

        geojson.Object.reset_all_objects()

        self.current_route = geojson.Object.marshal_list(jfeatures)
        for obj in self.current_route:
            obj.update_anchor(self._anchor)

        return self.current_route


_instance = None


def getInstance(node=None):
    global _instance
    if _instance is None:
        if node is None:
            raise RuntimeError("Need to pass node for the first getInstance")
        _instance = DataUtil(node)
    return _instance
