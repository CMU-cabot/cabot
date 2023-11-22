#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

import gzip

import numpy as np

from . import geometry_utils


class CartographerSubmap:

    def __init__(self, submap_entry, submap_texture):
        self._submap_entry = submap_entry
        self._submap_texture = submap_texture

        self._submap_transform = None
        self._submap_slice_transform = None
        self._submap_texture_intensity = None
        self._submap_texture_alpha = None


    def _unzip_submap_texture(self, submap_texture):
        decompressed_submap_texture_cells = gzip.decompress(submap_texture.cells)

        submap_texture_cells = [int(cell) for cell in decompressed_submap_texture_cells]

        submap_intensity = np.array(submap_texture_cells[0:submap_texture.height*submap_texture.width*2:2], np.uint8).reshape(submap_texture.height, submap_texture.width)
        submap_alpha = np.array(submap_texture_cells[1:submap_texture.height*submap_texture.width*2+1:2], np.uint8).reshape(submap_texture.height, submap_texture.width)
        return submap_intensity, submap_alpha


    @property
    def trajectory_id(self):
        return self._submap_entry.trajectory_id


    @property
    def submap_index(self):
        return self._submap_entry.submap_index


    @property
    def submap_version(self):
        return self._submap_entry.submap_version


    @property
    def pose(self):
        return self._submap_entry.pose


    @property
    def transform(self):
        if self._submap_transform is None:
            self._submap_transform = geometry_utils.pose2transform(self._submap_entry.pose)
        return self._submap_transform


    @property
    def width(self):
        return self._submap_texture.width


    @property
    def height(self):
        return self._submap_texture.height


    @property
    def resolution(self):
        return self._submap_texture.resolution


    @property
    def slice_pose(self):
        return self._submap_texture.slice_pose


    @property
    def slice_transform(self):
        if self._submap_slice_transform is None:
            self._submap_slice_transform = geometry_utils.pose2transform(self._submap_texture.slice_pose)
        return self._submap_slice_transform


    @property
    def texture_intensity(self):
        if self._submap_texture_intensity is None:
            self._submap_texture_intensity, self._submap_texture_alpha = self._unzip_submap_texture(self._submap_texture)
        return self._submap_texture_intensity


    @property
    def texture_alpha(self):
        if self._submap_texture_alpha is None:
            self._submap_texture_intensity, self._submap_texture_alpha = self._unzip_submap_texture(self._submap_texture)
        return self._submap_texture_alpha


    def global_to_local_pose(self, global_pose):
        global_transform = geometry_utils.pose2transform(global_pose)
        local_transform = np.dot(global_transform, np.linalg.inv(self.transform))
        local_pose = geometry_utils.transform2pose(local_transform)
        return local_pose


    def local_to_global_pose(self, local_pose):
        local_transform = geometry_utils.pose2transform(local_pose)
        global_transform = np.dot(local_transform, self.transform)
        global_pose = geometry_utils.transform2pose(global_transform)
        return global_pose