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
#  __init__.py
# 
#  Created on: Nov 11, 2020
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#

try:
    from . import mrob

    from pkg_resources import get_distribution

    __version__ = get_distribution('mrob').version

    for module in dir(mrob):
        n = len(module) - 1
        if not (module[:2] == '__' and module[n:n-2:-1] == '__') and module.count('.') == 0:
            globals()[module] = getattr(mrob, module)

    del mrob
except ImportError:
    import platform
    
    if platform.system() == "Windows":
        import sys
        
        sys.tracebacklimit = 0
        raise ImportError(
            "Maybe you don't have Microsoft Visual C++ Redistributable package installed. " + 
            "Please follow the link and install redistributable " +
            "package: https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-160" +
            "#visual-studio-2015-2017-2019-and-2022") from None
    
    raise
