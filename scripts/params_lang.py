import subprocess
from pathlib import Path
import sys
from os.path import join
import shutil
import os
import json  
import re  
import argparse  
from typing import Dict, Any  
from SCons.Script import Import


#log_file = join("./build.log")
#sys.stdout = open(log_file, "a")  # Append mode to add to the PlatformIO log
#sys.stderr = sys.stdout


# Importiere die SCons-Umgebung  
Import("env")
  
def extract_all_json_blocks(header_content: str) -> Dict[str, str]:  
    """  
    Extrahiert alle JSON-Blöcke aus dem Header-File.  
    Rückgabe ist ein Dictionary mit dem Namen der JSON-Variable als Schlüssel und dem JSON-String als Wert.  
    Beispiel:  
    {  
        "paramSystem": "{...}",  
        "paramBluetooth": "{...}",  
        ...  
    }  
    """  
    pattern = re.compile(r'const\s+char\s+(\w+)\s*\[\]\s*PROGMEM\s*=\s*R"rawliteral\(\s*(\{.*?\})\s*\)rawliteral";', re.DOTALL)  
    matches = pattern.findall(header_content)  
    if not matches:  
        raise ValueError("Keine JSON-Blöcke im Headerfile gefunden.")  
    return {name: json_str for name, json_str in matches}  
  
  
def clean_json_string(json_str: str) -> str:  
    """  
    Bereinigt den JSON-String von C-spezifischen Syntaxelementen.  
    """  
    # 1. Ersetzen von +String(...) + durch einen Platzhalter, z.B. einen leeren String ""  
    # json_str = re.sub(r'\+String\([^)]*\)\+', '""', json_str)  
  
    # 2. Ersetzen von +...+ Ausdrücken, die nicht durch String ersetzt wurden  
    # json_str = re.sub(r'\+[^+]+\+', '""', json_str)  
  
    # 3. Entfernen von definierten Makros wie ID_PARAM_...  
    # json_str = re.sub(r'ID_PARAM_[A-Z0-9_]+', '""', json_str)  
  
    # 4. Entfernen von C-Kommentaren (falls vorhanden)  
    # json_str = re.sub(r'//.*?\n|/\*.*?\*/', '', json_str, flags=re.DOTALL)  
  
    # 5. Entfernen von trailing commas vor schließenden Klammern  
    # json_str = re.sub(r',\s*([}\]])', r'\1', json_str)  
  
    # 6. Entfernen von Heksadezimalwerten (falls erforderlich)  
    # Beispiel: Ersetzen von 0xFFFF durch 0  
    # json_str = re.sub(r'0x[0-9A-Fa-f]+', '0', json_str)  
  
    return json_str  
  
  
def embed_json_into_header(header_content: str, json_blocks: Dict[str, str]) -> str:  
    """  
    Ersetzt die alten JSON-Blöcke im Header-File mit den neuen JSON-Blöcken.  
    """  
    for name, new_json in json_blocks.items():  
        escaped_json = new_json

        # Escape für C-String: Backslashes und Anführungszeichen  
        #escaped_json = new_json.replace('\\', '\\\\').replace('"', '\\"')  

        # Da der JSON in einer einzigen Zeile sein soll, entfernen wir Zeilenumbrüche  
        escaped_json = escaped_json.replace('\\\\n', '<br>')  
        escaped_json = escaped_json.replace('\\\\\\"', '\'')  
        escaped_json = escaped_json.replace('\", \"', '\",\"')  
        escaped_json = escaped_json.replace('\\', '')  

        # Ersetzen des alten JSON-Blöcke  
        pattern = re.compile(  
            rf'const\s+char\s+{re.escape(name)}\s*\[\]\s*PROGMEM\s*=\s*R"rawliteral\(\s*\{{.*?\}}\s*\)rawliteral";',  
            re.DOTALL  
        )  
        replacement = f'const char {name}[] PROGMEM = R"rawliteral( {escaped_json} )rawliteral";'  
        header_content, count = pattern.subn(replacement, header_content)  
        if count == 0:  
            print(f"Warnung: JSON-Block für '{name}' nicht im Headerfile gefunden.")  
    return header_content  
  
  
def update_fields_in_json(header_json: Dict[str, Any], external_settings: Dict[str, Any]):  
    """  
    Aktualisiert die Felder im header_json basierend auf external_settings.  
    Die external_settings sollten unter dem Schlüssel "texts-settings" organisiert sein.  
    """  
    texts_settings = external_settings.get("texts-settings", {})  
  
    def traverse_and_update(obj):  
        if isinstance(obj, dict):  
            if "lid" in obj:  
                lid = str(obj["lid"])  
                if lid in texts_settings:  
                    for key, value in texts_settings[lid].items():  
                        if key == "label":                    
                          # Überprüfen, ob "label" oder "l" in den params vorhanden sind
                          if "label" in obj:
                            obj["label"] = value 
                          elif "l" in obj:
                            obj["l"] = value
                        else:
                          obj[key] = value  
            # Rekursiv durch alle Schlüssel  
            for key in obj:  
                traverse_and_update(obj[key])  
        elif isinstance(obj, list):  
            for item in obj:  
                traverse_and_update(item)  
  
    traverse_and_update(header_json)  
  
  
def process_header(header_file_path: str, external_json_path: str, output_file_path: str = None):  
    # Lesen des Header-Files  
    with open(header_file_path, 'r', encoding='utf-8') as f:  
        header_content = f.read()  
  
    # Extrahieren aller JSON-Blöcke  
    try:  
        json_blocks = extract_all_json_blocks(header_content)  
    except ValueError as e:  
        print(f"Fehler beim Extrahieren der JSON-Blöcke: {e}")  
        return  
  
    if not json_blocks:  
        print("Keine JSON-Blöcke gefunden, nichts zu aktualisieren.")  
        return  
  
    # Laden des externen JSON-Files  
    with open(external_json_path, 'r', encoding='utf-8') as f:  
        external_settings = json.load(f)  
  
    # Aktualisieren jedes JSON-Blocks  
    updated_json_blocks = {}  
    for name, json_str in json_blocks.items():  
        # Bereinigung des JSON-Strings  
        cleaned_json_str = clean_json_string(json_str)  
  
        # Parsen des JSON  
        try:  
            header_json = json.loads(cleaned_json_str)  
        except json.JSONDecodeError as e:  
            print(f"Fehler beim Parsen des JSON für '{name}': {e}")  
            print("Orginaler JSON-String:")  
            print(json_str)
            print("Bereinigter JSON-String:")  
            print(cleaned_json_str)
            with open('cleaned_json_debug.txt', 'w', encoding='utf-8') as debug_file:  
                debug_file.write(cleaned_json_str)  
            print("Der bereinigte JSON-String wurde in 'cleaned_json_debug.txt' gespeichert zur Überprüfung.")  
            continue  
  
        # Aktualisieren der Felder  
        update_fields_in_json(header_json, external_settings)  
  
        # Serialisieren des aktualisierten JSON ohne Einrückungen, alles in einer Zeile  
        updated_json_str = json.dumps(header_json, ensure_ascii=False, separators=(',', ':'))  
        #updated_json_str = json.dumps(header_json, ensure_ascii=False, indent=4) # Debug
  
        # Speichern des aktualisierten JSON in den Dictionary  
        updated_json_blocks[name] = updated_json_str  
  
        # Debugging: Speichern des aktualisierten JSON-Strings zur Überprüfung  
        #with open(f'updated_json_debug_{name}.json', 'w', encoding='utf-8') as debug_file:  
        #    debug_file.write(updated_json_str)  
        #print(f"Aktualisierter JSON-String für '{name}' wurde in 'updated_json_debug_{name}.json' gespeichert zur Überprüfung.")  
  
    # Einbetten der aktualisierten JSON-Blöcke in den Header  
    new_header_content = embed_json_into_header(header_content, updated_json_blocks)  
  
    # Schreiben in die Ausgabedatei oder Überschreiben des Original-Headerfiles  
    output_path = output_file_path if output_file_path else header_file_path  
    with open(output_path, 'w', encoding='utf-8') as f:  
        f.write(new_header_content)  
  
    print(f"Headerfile erfolgreich aktualisiert und gespeichert unter: {output_path}")  




for flag in env['BUILD_FLAGS']:
    if 'BUILD_LANG' in flag:
        build_lang = flag.split('=')[-1].strip('"')
        print(f"BUILD_LANG: {build_lang}")

# Kopiere Originale params.h nach params_de.h  
shutil.copy("./include/params.h", "./include/params_de.h")  

# Verarbeite das Header-File mit den gegebenen Parametern  
process_header("./include/params_de.h", f"./lang/{build_lang}.json", "./include/params.h")  

# Kopiere zurück und bereinige  
# shutil.copy("./include/params_de.h", "./include/params.h")    
os.remove("./include/params_de.h")  



"""
if __name__ == "__main__":  
    print("__main__") 

    parser = argparse.ArgumentParser(description="Aktualisiert Felder in einem Header-File basierend auf einem externen JSON-File.")    
    parser.add_argument('-l', '--lang', help='Sprache in der params.h erstellt werden soll.', default='de_DE')    
  
    args = parser.parse_args()    
    language = args.lang  
  
    # Kopiere Originale params.h nach params_de.h  
    shutil.copy("../include/params.h", "../include/params_de.h")    
  
    # Verarbeite das Header-File mit den gegebenen Parametern  
    process_header("../include/params_de.h", f"../lang/{language}.json", "../include/params.h")  
  
    # Kopiere zurück und bereinige  
    # shutil.copy("../include/params_de.h", "../include/params.h")    
    # os.remove("../include/params_de.h")  
"""
