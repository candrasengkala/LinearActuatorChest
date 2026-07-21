Import("env")
import subprocess

def after_upload(source, target, env):
    print("Upload complete — starting monitor capture to r_measurement.csv")
    subprocess.run(
        'pio device monitor -b 115200 | tee r_measurement.csv',
        shell=True
    )

env.AddPostAction("upload", after_upload)