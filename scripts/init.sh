#!/bin/bash
# Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
# 
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# 
# 
#  init.sh
# 
#  Created on: Oct 13, 2021
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#

set -euo pipefail

IFS=$'\n' MODULES=$(python3 -c \
"import mrob
for x in dir(mrob):
     if not (len(x) > 2 and x[0] == '_' and x[1] == '_'):
         print(x)")

echo -e "from mrob import mrob\n" >> __init__.py
for module in $MODULES
do
    if ! [[ $module == *"."* ]]; then
        echo $module = mrob.$module >> __init__.py
    fi
done
echo -e "\ndel(mrob)" >> __init__.py
