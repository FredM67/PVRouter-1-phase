from datetime import datetime
import os
import subprocess
from subprocess import CalledProcessError

def get_git_revision_short_hash() -> str:
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], shell=True).decode('utf-8').strip()
    except CalledProcessError:
        return "N/A"

def get_git_current_branch() -> str:
    try:
        return subprocess.check_output(['git', 'branch', '--show-current'], shell=True).decode('utf-8').strip()
    except CalledProcessError:
        return "N/A"

Import("env")
proj_path = os.path.join(env["PROJECT_DIR"], "dummy")

macro_value = f"\"{os.path.split(os.path.dirname(proj_path))[1]}\""
tz_dt = datetime.now().replace(microsecond=0).astimezone().isoformat(' ')

env.Append(CPPDEFINES=[
    ("PROJECT_PATH", macro_value),
    ("CURRENT_TIME", f"\"{tz_dt}\""),
    ("BRANCH_NAME", f"\"{get_git_current_branch()}\""),
    ("COMMIT_HASH", f"\"{get_git_revision_short_hash()}\"")
])
