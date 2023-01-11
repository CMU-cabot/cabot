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

import collections.abc
from typing import Iterable, List, Text

from ament_index_python.packages import get_package_share_directory
from launch.some_substitutions_type import SomeSubstitutionsType
from launch import LaunchContext
from launch.substitution import Substitution
from launch.utilities import (ensure_argument_type,
                              normalize_to_list_of_substitutions,
                              perform_substitutions)


class GetPackageShareDirectory(Substitution):
    def __init__(self, expression: SomeSubstitutionsType) -> None:
        """Create a GetPackageShareDirectory substitution."""
        super().__init__()
        ensure_argument_type(
            expression,
            (str, Substitution, collections.abc.Iterable),
            'expression',
            'GetPackageShareDirectory')
        self.__expression = normalize_to_list_of_substitutions(expression)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `GetPackageShareDirectory` substitution."""
        if len(data) != 1:
            raise TypeError('eval substitution expects 1 argument')
        return cls, {'expression': data[0]}

    @property
    def expression(self) -> List[Substitution]:
        """Getter for expression."""
        return self.__expression

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'PythonExpr({})'.format(' + '.join([sub.describe() for sub in self.expression]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        return get_package_share_directory(perform_substitutions(context, self.expression))
