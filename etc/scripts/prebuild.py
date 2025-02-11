import os
import subprocess
import sys

Import("env")

# retrieve build flags
my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}

version_string = defines.get("VERSION_STRING")  # e.g. "1.2.2"
board_name = env["BOARD"]  # e.g. "esp01m"

# replace dots in version if linker can't find the path
#version_string = version_string.replace(".","_")

# set board and version in firmware name
env.Replace(PROGNAME="mavesp-{}-{}".format(board_name, version_string))

# run Kconfig to generate config.h
build_dir = env.subst('$BUILD_DIR')
subprocess.run([
    sys.executable, "lib/kconfig/genconfig.py", "Kconfig",
    "--header-path", "{}/config.h".format(build_dir)
], check=True, env=dict(os.environ, PYTHONPATH="lib/kconfig"))

# Add build folder to path
env.Append(CPPPATH=[build_dir])
env.BuildSources(build_dir, build_dir)
