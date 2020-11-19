# Copyright (c) 2020 Carnegie Mellon University
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

import sys
import json

import rospy
import requests
import geojson
import geoutil

class DataUtil(object):
    """Data Utility class"""

    SEARCH_API = "routesearch"
    
    def __init__(self):
        self._protocol = 'https'
        self._hostname = 'cmu-map.mybluemix.net/map'
        self._user = 'Cabot'
        self._lang = 'en'
        self._dist = 1000
        self._latitude = 40.4432778
        self._longitude = -79.9456066
        self._initialized = False
        self._anchor = geoutil.Anchor(lat=self._latitude, lng=self._longitude, rotate=0)
        # public data
        self.landmarks = None
        self.node_map = None
        self.features = None
        self.current_route = None
        self.is_ready = False
        self.is_analyzed = False

    def set_anchor(self, anchor):
        self._anchor = anchor
        geojson.Object.update_anchor_all(self._anchor)
    
    def get_search_url(self):
        """get the URL for search api"""
        return "%s://%s/%s" % (self._protocol, self._hostname, self.SEARCH_API)

    def init_by_server(self):
        """initialize server state for a user"""
        if self.is_ready:
            return
        rospy.loginfo("init server")
        self.init_by_data(landmarks = self.get_landmarks(),
                          node_map = self.get_node_map(),
                          features = self.get_features())

    def init_by_data(self, landmarks=[], node_map={}, features=[]):
        """initialize datautil with data"""
        self.landmarks = landmarks
        self.node_map = node_map
        self.features = features
        self.is_ready = True

    def get_landmarks(self, filename=None):
        """get landmarks"""
        if filename is None:
            req = requests.post(self.get_search_url(), data={
                'action': 'start',
                'user': self._user,
                'lang': self._lang,
                'lat': self._latitude,
                'lng': self._longitude,
                'dist': self._dist,
            })
            if req.status_code != requests.codes.ok:
                rospy.signal_shutdown('could not initialized')
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
            req = requests.post(self.get_search_url(), data={
                'action': 'nodemap',
                'user': self._user,
                'lang': self._lang
            })
            if req.status_code != requests.codes.ok:
                rospy.signal_shutdown('could not initialized')
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
            req = requests.post(self.get_search_url(), data={
                'action': 'features',
                'user': self._user,
                'lang': self._lang
            })
            if req.status_code != requests.codes.ok:
                rospy.signal_shutdown('could not initialized')
            data = json.loads(req.text)
        else:
            data = json.load(open(filename))

        import tempfile

        f = open(tempfile.gettempdir()+"/features.json", "w")
        f.write(json.dumps(data, indent=4))
        f.close()
        
        return geojson.Object.marshal_list(data)


    def analyze_features(self):
        if self.is_analyzed:
            return
        links = geojson.Object.get_objects_by_type(geojson.Link)
        doors = geojson.Object.get_objects_by_type(geojson.DoorPOI)
        infos = geojson.Object.get_objects_by_type(geojson.InfoPOI)
        speeds = geojson.Object.get_objects_by_type(geojson.SpeedPOI)

        for poi in doors+infos+speeds:
            min_link = geojson.Object.get_nearest_link(poi)
            if min_link is None:
                print ("poi %s (%f) is not registered. could not find a link" \
                       % (poi._id, poi.floor))
                continue
            min_dist = min_link.geometry.distance_to(poi.geometry)
            
            if min_dist < 5:
                min_link.register_poi(poi)
                #print "poi %s (%f) is registered to %s (%f) %f" % \
                #    (poi._id, poi.floor, min_link._id, min_link.floor, min_dist)
            else:
                print ("poi %s (%f) is not registered. " \
                    + "min_link._id = %s, min_link.floor = %f") \
                    % (poi._id, poi.floor, min_link._id, min_link.floor)
                print (poi._id, poi.floor, min_link._id, min_link.floor)
                
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
        rospy.loginfo("request route from %s to %s", from_id, to_id)
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
                'tactile_paving': '1', ## not prefer tactile paving
                'mvw': '1', ## do not use moving walkway
                'slope': '9',
                'deff_LV': '9',
                'elv': '9',
                'road_condition': '9',
                'esc': '1' ## do not use escalator
            })
        })
        rospy.loginfo("get data %d bytes", len(req.text))
        #rospy.loginfo(json.dumps(json.loads(req.text), indent=4))

        jfeatures = json.loads(req.text)
        import tempfile
        f = open(tempfile.gettempdir()+"/route_%s_%s"%(from_id,to_id), "w")
        f.write(json.dumps(jfeatures, indent=4))
        f.close()
        
        geojson.Object.reset_all_objects()

        self.current_route = geojson.Object.marshal_list(jfeatures)
        
        return self.current_route


instance = DataUtil()

if __name__ == '__main__':
    map = {}

    for f in instance.features:
        if type(f) not in map:
            map[type(f)] = True
            print(f)
    
    for n in instance.node_map.values():
        if type(n) not in map:
            map[type(n)] = True
            print(n)

    for l in instance.landmarks:
        if type(l) not in map:
            map[type(l)] = True
            print(l)

    routes = instance.get_route('EDITOR_node_1490021920691','EDITOR_node_1498748753059')
    for r in routes:
        print r
    
