Import("env")
import os
import re
import shutil
from datetime import datetime

def after_build(target, source, env):
    print("Linking abgeschlossen!")
      
    code_file = "include/defines.h"
    platformio_ini_file = "platformio.ini" 

    pattern = r'#define\s+BSC_SW_VERSION\s+"(.+?)"'
    spezial_flag_pattern = r'^\s*-DBSC_SW_SPEZIAL=(.*)"'

    with open(code_file, 'r') as file:
        content = file.read()
        match = re.search(pattern, content)
        if match:
            base_firmware_name = match.group(1) 
        else:
            raise ValueError("Firmware-Name nicht gefunden.")

    # Wert des -DBSC_SW_SPEZIAL Flags aus platformio.ini lesen
    # Alle Umgebungen in platformio.ini suchen
    with open(platformio_ini_file, 'r') as file:
        ini_content = file.read()

    # Aktive Umgebung
    current_env = env["PIOENV"]

    # Wert des -DBSC_SW_SPEZIAL Flags aus der aktuellen Umgebung lesen
    env_pattern = rf'\[env:{re.escape(current_env)}\](.*?)\n\[env:'

    # Suche nach der Umgebung
    env_block = re.search(env_pattern, ini_content, re.DOTALL)

    if env_block:
        env_content = env_block.group(1)
        spezial_match = re.search(spezial_flag_pattern, env_content, re.MULTILINE)
        
        if spezial_match:
            bsc_sw_spezial = spezial_match.group(1).replace("\\","").replace("\"","")  # Wert von BSC_SW_SPEZIAL extrahieren
            if bsc_sw_spezial != "":
                bsc_sw_spezial = "_" + bsc_sw_spezial
        else:
            bsc_sw_spezial = ""
    else:
        raise ValueError(f"Keine Informationen für die Umgebung '{current_env}' gefunden.")


    # Datum/Uhrzeit
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    firmwareName_bin = f"firmware_{timestamp}_{base_firmware_name}{bsc_sw_spezial}.bin" 
    firmwareName_elf = f"firmware_{timestamp}_{base_firmware_name}{bsc_sw_spezial}.elf" 

    # Pfade
    build_dir = os.path.join(".pio", "build", current_env)  
    dest_dir = os.path.join("..", "bsc_fw_bin", current_env)
    aFirmware_bin = os.path.join(build_dir, "firmware.bin")
    nFirmware_bin = os.path.join(dest_dir, firmwareName_bin)
    aFirmware_elf = os.path.join(build_dir, "firmware.elf")
    nFirmware_elf = os.path.join(dest_dir, firmwareName_elf)

    os.makedirs(dest_dir, exist_ok=True)

    # Kopieren
    if os.path.exists(aFirmware_elf):
        shutil.copy(aFirmware_elf, nFirmware_elf)
        print(f"Kopiere: {nFirmware_elf}")
    else:
        print("Firmware-Datei nicht gefunden.")

    
    if os.path.exists(aFirmware_bin):
        shutil.copy(aFirmware_bin, nFirmware_bin)
        print(f"Kopiere: {nFirmware_bin}")
    else:
        print("Firmware-Datei nicht gefunden.")



env.AddPostAction("buildprog", after_build)
