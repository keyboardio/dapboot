Import("env")
import os
import subprocess
import shutil

# original Makefile does
# GIT_BUILD_SHA := $(shell git rev-parse HEAD |cut -c 1-6)
# which is, ahem, Unix specific due to the cut.
# let's properly call into git and inject the macro.
def get_git_hash():
    if not os.path.isdir(".git"):
        print(
            "Aborting creation of version.c since this project was not cloned via `git`...")
        return None
    # sanity check: do we have git?
    if shutil.which("git") is None:
        print("Command `git` is not available, aborting creation of src/version.c..")
        return None
    try:
        git_tag_cmd = ["git", "rev-parse", "HEAD"]
        git_tag = subprocess.check_output(
            git_tag_cmd).decode('utf-8').strip()[0:6]
        print("Got tag: %s" % git_tag)
        return git_tag
    except Exception as exc:
        print("Failed to do get git hash: " + str(exc))
        return None

git_hash = get_git_hash()
if git_hash is None: 
    env.Exit(-1)
# inject into build system as direct macro. explicitly no string-quoting.
env.Append(CCFLAGS=["-DGIT_BUILD_SHA=%s" % str(git_hash)])
