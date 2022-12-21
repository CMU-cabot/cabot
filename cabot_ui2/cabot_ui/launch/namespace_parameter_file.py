# Copyright (c) 2022  Carnegie Mellon University
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

from typing import Union
from typing import Text
import yaml

from pathlib import Path
from tempfile import NamedTemporaryFile

from launch import SomeSubstitutionsType, LaunchContext
from launch.utilities import ensure_argument_type
from launch.utilities import perform_substitutions
from launch.utilities.typing_file_path import FilePath
from launch_ros.descriptions import ParameterFile


class NamespaceParameterFile(ParameterFile):
    """
    Describes a ROS parameter file where parameters are written in the top level
    and generate a new parameter file that has the top parameters are under the specified namespace
    """

    def __init__(
        self,
        namespace: Text,
        param_file: Union[FilePath, SomeSubstitutionsType],
        *,
        allow_substs: Union[bool, SomeSubstitutionsType] = False
    ) -> None:
        super().__init__(param_file, allow_substs=allow_substs)

        ensure_argument_type(
            namespace,
            Text,
            'namespace',
            'NamespaceParameterFile()'
        )

        self.__namespace: Text = namespace

    @property
    def namespace(self) -> Text:
        return self.__namespace

    def __str__(self) -> Text:
        return (
            'cabot_util.launch.NamespaceParameterFile'
            f'(namespace={self.namespace}, param_file={self.param_file}, allow_substs={self.allow_substs})'
        )

    def evaluate(self, context: LaunchContext) -> Path:
        """Evaluate and return a parameter file path."""
        if self._ParameterFile__evaluated_param_file is not None:
            return self._ParameterFile__evaluated_param_file

        param_file = self._ParameterFile__param_file
        if isinstance(param_file, list):
            # list of substitutions
            param_file = perform_substitutions(context, self._ParameterFile__param_file)

        param_file_path: Path = Path(param_file)

        with open(param_file_path, 'r') as f, NamedTemporaryFile(
            mode='w', prefix='launch_params_', delete=False
        ) as h:
            data = yaml.safe_load(f)
            namespaced_data = {
                self.__namespace: {
                    'ros__parameters': data
                }
            }
            h.write(yaml.dump(namespaced_data))
            param_file_path = Path(h.name)

        self._ParameterFile__evaluated_param_file = param_file_path
        return param_file_path
