"""Local process ownership and atomic status support for read-only Task6 tools."""

import fcntl
import json
import os
from pathlib import Path


def singleton(name):
    """Hold one instance per user/tool/ROS domain until the file handle closes."""
    directory = Path.home() / '.cache' / 'task6_ops'
    directory.mkdir(parents=True, exist_ok=True)
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdecimal() or not 0 <= int(domain) <= 232:
        raise ValueError('ROS_DOMAIN_ID must be an integer in 0..232')
    domain = str(int(domain))
    handle = (directory / f'{name}_{domain}.lock').open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError(f'{name}: another instance owns this ROS domain') from None
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


def write_json(path, value):
    """Encode before opening a temporary file, then replace status atomically."""
    text = json.dumps(value, indent=2)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(text + '\n')
    temporary.replace(path)
