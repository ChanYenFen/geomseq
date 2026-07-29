"""
Path Resolver Component for GeomSeq
GH entry point: locates src/ module folders relative to the saved .gh file and adds them to sys.path, so other components can import geomseq_core/rhino_utils regardless of machine.
"""

import os
import sys


def _find_src_dir(start_dir):
    """Walk up from start_dir looking for a directory containing geomseq/."""
    current = os.path.abspath(start_dir)
    while True:
        if os.path.isdir(os.path.join(current, "geomseq")):
            return current
        parent = os.path.dirname(current)
        if parent == current:  # hit filesystem root, not found
            return None
        current = parent


def _find_module_path(search_root, module):
    """Search under search_root for a file or folder named exactly `module`, at any depth."""
    for root, dirs, files in os.walk(search_root):
        if module in dirs or module in files:
            return os.path.join(root, module)
    return None


gh_doc = ghenv.Component.OnPingDocument()  # type: ignore
script_dirs = []

if not gh_doc.FilePath:
    print("[geomseq] can't resolve project src/ -- .gh document hasn't been saved yet")
else:
    src_dir = _find_src_dir(os.path.dirname(gh_doc.FilePath))
    if src_dir is None:
        print("[geomseq] can't find a 'geomseq' folder walking up from the .gh document's location")
    else:
        search_root = os.path.join(src_dir, "geomseq", "src")
        for module in modules:  # type: ignore
            found = _find_module_path(search_root, module)
            if found is None:
                print("[geomseq] module not found under geomseq/src/: %s" % module)
            else:
                script_dirs.append(os.path.normpath(found))

for module_dir in script_dirs:
    parent_dir = os.path.dirname(module_dir)
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)