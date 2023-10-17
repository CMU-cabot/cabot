#!/bin/bash

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

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

HOST=http://map_server:9090/map
data_dir=/server_data
admin=hulopadmin
pass=please+change+password
editor=editor


count=0
echo "waiting server is up"
while [ "$(curl -I $HOST/login.jsp 2>/dev/null | head -n 1 | cut -d' ' -f2)" != "200" ];
do
    snore 1
    UPLINE=$(tput cuu1)
    ERASELINE=$(tput el)
    echo -n "$UPLINE$ERASELINE"
    echo "waiting server is up ($count)"
    count=$((count+1))
done

pushd $temp_dir

blue "adding editor user"
curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp
curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp
curl -b admin-cookie.txt -d "user=$editor&password=$editor&password2=$edito&role=editor" "$HOST/api/user?action=add-user"

blue "importing attachments.zip"
if [ ! -e $data_dir/attachments.zip ]; then
    if [ -e $data_dir/attachments ]; then
	pushd $data_dir/attachments
	zip -r ../attachments.zip .
	popd
    else
	red "[WARNING] there is not attachments directory or attachments.zip"
    fi
fi
if [ -e $data_dir/attachments.zip ]; then
    curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp
    curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp
    curl -b admin-cookie.txt -F file=@$data_dir/attachments.zip "$HOST/api/admin?action=import&type=attachment.zip"
fi
