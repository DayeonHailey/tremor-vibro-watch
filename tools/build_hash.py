# For PlatformIO build_flags: prints the first 8 hex digits of git HEAD as -DBUILD_HASH.
# Sets the top bit if there are uncommitted changes (distinguishable via GET_INFO build_hash).
import subprocess

try:
    h = int(subprocess.check_output(["git", "rev-parse", "--short=8", "HEAD"],
                                    stderr=subprocess.DEVNULL).decode().strip(), 16)
    dirty = subprocess.call(["git", "diff", "--quiet", "HEAD"], stderr=subprocess.DEVNULL) != 0
    if dirty:
        h |= 0x80000000
except Exception:
    h = 0
print("-DBUILD_HASH=0x%08Xu" % h)
