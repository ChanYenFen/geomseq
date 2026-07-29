import sys


def unload_modules(top_level_module_name):
    """Unloads all modules starting with `top_level_module_name` (e.g. 'geomseq_core'),
    adapted from COMPAS to support dynamic reloading in Rhino/GH."""
    # 1. Identify all related modules
    modules_to_remove = [
        m for m in sys.modules.keys()
        if m.startswith(top_level_module_name)
    ]

    # 2. Remove them from memory
    for module_name in modules_to_remove:
        del sys.modules[module_name]

    # 3. Return list of unloaded modules (useful for debugging)
    return modules_to_remove
