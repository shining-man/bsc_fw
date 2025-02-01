import os
import requests
from collections import defaultdict

github_token = os.environ.get("GITHUB_TOKEN")
if not github_token:
    raise ValueError("Bitte setze den GITHUB_TOKEN als Umgebungsvariable")


# GraphQL Query mit Spendengröße
query = """
{
  viewer {
    sponsorshipsAsMaintainer(first: 100) {
      nodes {
        tier {
          monthlyPriceInCents
        }
        sponsor {
          login
          avatarUrl
          url
        }
      }
    }
  }
}
"""

headers = {
    "Authorization": f"Bearer {github_token}",
    "Content-Type": "application/json",
}

response = requests.post(
    "https://api.github.com/graphql",
    json={"query": query},
    headers=headers
)

if response.status_code != 200:
    raise Exception(f"API Fehler: {response.status_code} {response.text}")

data = response.json()

# Fest definierte Sponsoring-Gruppen
fixed_tiers = {
    5: "Supporter",
    10: "Bronze",
    20: "Silver",
    50: "Gold"
}

ordered_tiers = [
    "Gold",
    "Silver",
    "Bronze",
    "Supporter",
    "Individual"
]

sponsors_by_tier = defaultdict(list)

try:
    nodes = data["data"]["viewer"]["sponsorshipsAsMaintainer"]["nodes"]
    for sponsorship in nodes:
        tier = sponsorship["tier"]
        sponsor = sponsorship["sponsor"]

        # Betrag berechnen und auf Ganzzahl prüfen
        amount = tier["monthlyPriceInCents"] / 100
        if amount.is_integer():
            amount = int(amount)

        # Filter für Beträge unter $5
        if amount < 5:
            continue  # Sponsor wird übersprungen

        login = sponsor["login"]
        avatar_url = sponsor["avatarUrl"]
        profile_url = f"https://github.com/{login}"

        sponsor_html = f'<a href="{profile_url}" target=_blank><img src="{avatar_url}" height="58"/></a>'

        # Gruppe zuweisen
        tier_name = fixed_tiers.get(amount, "Individual")
        sponsors_by_tier[tier_name].append(sponsor_html)

except KeyError:
    raise Exception("Ungültige API Response: ", data)

# README.md bearbeiten
with open("README.md", "r") as f:
    content = f.read()

start_marker = "<!-- SPONSORS_START -->"
end_marker = "<!-- SPONSORS_ENDE -->"

start = content.find(start_marker) + len(start_marker)
end = content.find(end_marker)

if start == -1 or end == -1:
    raise ValueError("Markierungen nicht gefunden")

# Sponsoren in festgelegter Reihenfolge einfügen
sponsor_sections = []
sponsor_sections_without_group = []
for tier_name in ordered_tiers:
    sponsors = sponsors_by_tier.get(tier_name, [])
    if sponsors:
        sponsor_sections.append(f"<p align=\"center\"><strong>{tier_name}</strong></p><p align=\"center\">")
        sponsor_sections.extend(sponsors)
        sponsor_sections.append(f"</p>")

        sponsor_sections_without_group.extend(sponsors)

new_content = (
    content[:start]
    + "\n"
    + "\n".join(sponsor_sections)
    + "\n"
    + content[end:]
)
print(new_content)

new_content_without_group = (
    "<p align=\"center\">\n"
    + "\n".join(sponsor_sections_without_group)
    + "\n</p>"
)
print(new_content_without_group)

with open("README.md", "w") as f:
    f.write(new_content)

with open("spons", "w") as f:
    f.write(new_content_without_group)

print("Sponsoren erfolgreich gruppiert aktualisiert!")
