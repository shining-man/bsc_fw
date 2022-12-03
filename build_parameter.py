defines_dict = dict()

datei = open('./include/defines.h','r')
for zeile in datei:
    defFound = zeile.find("#define ")
    if defFound >= 0:
        defs = zeile.strip().split(' ',2)
        if len(defs) == 3:
            defines_dict.update({defs[1].strip(): defs[2].strip().replace('"', '')})
datei.close()

datei = open('./include/WebSettings.h','r')
for zeile in datei:
    defFound = zeile.find("#define ")
    if defFound >= 0:
        defs = zeile.strip().split(' ',2)
        if len(defs) == 3:
            defines_dict.update({defs[1].strip(): defs[2].strip().replace('"', '')})
datei.close()

dateiOut = open('./include/params.h','w')
datei = open('./include/params_py.h','r')
pyVar = 0
pyVarVarName=""
pyVarValue=""
for zeile in datei:
    zeileNeu = ""

    defFoundStart = zeile.find("//PY_VAR_ANF")
    if defFoundStart >= 0:
        pyVar=1
        pyVarVarName=""
        pyVarValue=""

    defFoundStart = zeile.find("//PY_VAR_END")
    if defFoundStart >= 0:
        pyVar=4
        defines_dict.update({pyVarVarName:pyVarValue})
        pyVarVarName=""
        pyVarValue=""
    
    if pyVar == 3:
        pyVarValue += zeile.strip().strip("\"").rstrip(";").rstrip("\"")
            
    if pyVar == 2:
        pyVar=3
        pyVarVarName = zeile.strip().split(' ',3)[2].strip()

    if pyVar == 0:
        defFoundStart = zeile.find("const String ")
        if defFoundStart >= 0:
            zeileNeu = zeile.replace("const String ", "const char ")
            varName = zeileNeu.strip().split(' ',3)
            zeileNeu = zeileNeu.replace(varName[2], varName[2]+"[]")
        else:
            defFoundStart = zeile.find("+String(")
            if defFoundStart >= 0:
                defFoundEnd = zeile.find(")")
                if defFoundEnd >= 0:
                    defn = zeile[defFoundStart+8:defFoundEnd]
                    if defn in defines_dict:
                        defNr = defines_dict.get(defn)
                        zeileNeu = zeile.replace(zeile[defFoundStart-1:defFoundEnd+3],defNr);
                    else:
                        zeileNeu=zeile
                else:
                    zeileNeu=zeile
            else:
                defFoundStart = zeile.find("+")
                if defFoundStart >= 0:
                    varName2 = zeile.split("+")[1]
                    if varName2 in defines_dict:
                        zeileNeu = zeile.replace("+"+varName2+"+","\""+defines_dict.get(varName2)+"\"");

                else:
                    zeileNeu=zeile

        dateiOut.write(zeileNeu)
 
    if pyVar == 1:
        pyVar=2

    if pyVar == 4:
        pyVar=0

datei.close()
dateiOut.close()
