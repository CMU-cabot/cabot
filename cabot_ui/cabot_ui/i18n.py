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

import sys
import os
import os.path
import yaml
import traceback

from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

from ament_index_python.packages import get_package_share_directory


_i18n_table = {}
_lang = "en"


def set_language(language):
    global _lang
    _lang = language


def localized_string(identifier, *args, **kwargs):
    lang = kwargs["lang"] if "lang" in kwargs else _lang  # default is en

    # CaBotRclPyUtil.debug("lang=%s, identifier=%s", lang, identifier)
    try:
        format_str = _i18n_table[lang][identifier]
    except:  # noqa: E722
        format_str = identifier

    if isinstance(format_str, str):
        format_str = format_str

    if args:
        print(args)
        try:
            return format_str.format(*args)
        except:  # noqa: E722
            traceback.print_exc(file=sys.stdout)
            pass
    return format_str


def localized_attr(properties, key, **kwargs):
    lang = kwargs["lang"] if "lang" in kwargs else _lang  # default is en
    if 'only_if' in kwargs and kwargs['only_if'] != lang:
        return None
    lang_key = "_".join([key, lang])

    if hasattr(properties, lang_key):
        return getattr(properties, lang_key)
    else:
        if hasattr(properties, key):
            return getattr(properties, key)
    return None


def localized_value(dictionary, key, **kwargs):
    lang = kwargs["lang"] if "lang" in kwargs else _lang  # default is en
    lang_key = "_".join([key, lang])
    if lang_key in dictionary:
        return dictionary[lang_key]
    else:
        if key in dictionary:
            return dictionary[key]
    return None


def init(directory):
    for filename in os.listdir(directory):
        if "yaml" not in filename:
            continue
        with open(os.path.join(directory, filename), "rb") as stream:
            text = stream.read().decode(encoding="utf-8")
            data = yaml.safe_load(text)
            CaBotRclpyUtil.debug(str(data))
            basename, _ = os.path.splitext(filename)
            if basename not in _i18n_table:
                _i18n_table[basename] = {}
            _i18n_table[basename].update(data)

    CaBotRclpyUtil.info(str(_i18n_table))


def load_from_packages(packages):
    for package in packages:
        if packages:
            init(os.path.join(get_package_share_directory(package), "i18n"))
