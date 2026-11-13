"""The ABI version guard: the binary, the Python wrapper and the C# wrapper must
name the same interface, and a binary that names another one must be refused.
Plain CPython, no Rhino -- the C# side is checked by reading its source, since
nothing here can load the .gha."""

import os
import re
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "src"))

from geomseq_core import native_bridge

CS_LOADER = os.path.join(_ROOT, "src", "gha", "Native", "NativeLibraryLoader.cs")
CPP_SOURCE = os.path.join(_ROOT, "src", "geomseq_core", "native", "abi_version.cpp")


def _only_int(path, pattern):
    with open(path, encoding="utf-8") as fh:
        found = re.findall(pattern, fh.read())
    assert len(found) == 1, "%s: expected one match for %r, got %r" % (path, pattern, found)
    return int(found[0])


def test_binary_reports_the_version_python_expects():
    lib = native_bridge.load_dll()
    assert lib.geomseq_abi_version() == native_bridge.ABI_VERSION


def test_source_matches_python():
    # Catches a bump in the .cpp that was never compiled into the committed binary.
    declared = _only_int(CPP_SOURCE, r"geomseq_abi_version\(\)\s*\{\s*return\s+(\d+)\s*;")
    assert declared == native_bridge.ABI_VERSION


def test_csharp_matches_python():
    declared = _only_int(CS_LOADER, r"const\s+int\s+AbiVersion\s*=\s*(\d+)\s*;")
    assert declared == native_bridge.ABI_VERSION


def test_mismatched_binary_is_refused(monkeypatch):
    # The guard must fail when the versions differ, not only pass when they agree.
    monkeypatch.setattr(native_bridge, "_DLL", None)
    monkeypatch.setattr(native_bridge, "ABI_VERSION", native_bridge.ABI_VERSION + 1)
    with pytest.raises(RuntimeError, match="ABI version"):
        native_bridge.load_dll()
    assert native_bridge._DLL is None  # a refused binary is not cached
