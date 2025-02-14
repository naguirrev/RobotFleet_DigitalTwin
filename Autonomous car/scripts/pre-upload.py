import os
Import("env")

def before_uploadfs(source, target, env):
    print("Executing 'pio run --target uploadfs' before upload...")
    os.system("pio run --target uploadfs")

env.AddPreAction("upload", before_uploadfs)
