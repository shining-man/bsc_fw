import os

Import("env")
  

def log_output(source, target, env):
    # Löschen der bestehenden build.log-Datei, falls sie existiert
    if os.path.exists("build.log"):
        os.remove("build.log")

    with open("build.log", "w") as f:
        f.write(env.Dump())

env.AddPostAction("buildprog", log_output)
