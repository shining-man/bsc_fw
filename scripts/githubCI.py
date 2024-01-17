# Copyright (c) 2024 Tobias Himmler
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os

env_file = os.getenv('GITHUB_ENV')


datei = open('./include/defines.h','r')
for zeile in datei:
    defFound = zeile.find("#define ")
    if defFound >= 0:
        defs = zeile.strip().split(' ',2)
        if len(defs) == 3:
            defName = defs[1].strip()
            defValue = defs[2].strip().replace('"', '')

            if(defName == "BSC_SW_VERSION"):
                defValue=defValue.lower()
                print(defName+"="+defValue)
                with open(env_file, "a") as myfile:
                    myfile.write(defName+"="+defValue)

datei.close()