# For PlatformIO build_flags: prints the first 8 hex digits of git HEAD as -DBUILD_HASH.
# Sets bit 31 if there are uncommitted changes; the hash itself is 31 bits (distinguishable via GET_INFO build_hash).
import subprocess

try:
    h = int(subprocess.check_output(["git", "rev-parse", "--short=8", "HEAD"],
                                    stderr=subprocess.DEVNULL).decode().strip(), 16)
    dirty = subprocess.call(["git", "diff", "--quiet", "HEAD"], stderr=subprocess.DEVNULL) != 0
    h &= 0x7FFFFFFF   # bit 31 is reserved for the dirty flag, so a hash with that bit set cannot be mistaken for dirty
    if dirty:
        h |= 0x80000000
except Exception:
    h = 0
print("-DBUILD_HASH=0x%08Xu" % h)
