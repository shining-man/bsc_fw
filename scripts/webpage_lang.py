import re
import json
from SCons.Script import Import

Import("env")

def replace_lid_texts(header_file, json_file, output_file):
    # Headerfile und JSON-Datei einlesen
    with open(header_file, 'r', encoding='utf-8') as hf:
        header_content = hf.read()
    
    with open(json_file, 'r', encoding='utf-8') as jf:
        json_content = json.load(jf)

    # Die Texte aus der JSON-Datei extrahieren
    texts_menu = json_content.get("texts-menu", {})

    # Regex zum Finden der Tags mit lid-Attribut
    tag_pattern = re.compile(r"""(<(?P<tag>p|div|span|a)[^>]*\slid=['"](?P<lid>\d+)['"][^>]*>)(?P<content>.*?)(</(?P=tag)>)""", re.DOTALL)

    def replace_text(match):
        if len(match.groups()) != 5:
            print("Fehler: Group anzahl ist falsch; groups:", match.groups())
        opening_tag = match.group(1)  # Öffnendes Tag inklusive Attribute
        lid = match.group('lid')      # lid-Wert
        content = match.group('content')  # Der Textinhalt, der ersetzt werden soll
        closing_tag = match.group(5)  # Schließendes Tag (jetzt korrekt)

        # Neuen Inhalt aus der JSON-Datei holen
        new_text = texts_menu.get(lid, {}).get('label', content)  # Fallback auf den alten Inhalt, falls nicht gefunden

        # Debugging-Ausgabe
        #print(f"Processing lid={lid}, original content='{content}', new content='{new_text}'")

        # Rückgabe mit neuem Text. Der alte Inhalt wird vollständig durch den neuen ersetzt.
        return f"{opening_tag}{new_text}{closing_tag}"

    # Ersetze die Inhalte der Tags
    updated_content = re.sub(tag_pattern, replace_text, header_content)

    # Aktualisierten Inhalt in eine neue Datei schreiben
    with open(output_file, 'w', encoding='utf-8') as of:
        of.write(updated_content)


for flag in env['BUILD_FLAGS']:
    if 'BUILD_LANG' in flag:
        build_lang = flag.split('=')[-1].strip('"')
        print(f"BUILD_LANG: {build_lang}")


# Dateien definieren
header_file = "./include/webpages.h"
json_file = f"./lang/{build_lang}.json"
#output_file = "./include/webpages_en.h"

replace_lid_texts(header_file, json_file, header_file)
