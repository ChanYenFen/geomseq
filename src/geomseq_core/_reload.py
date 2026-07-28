import sys


def unload_modules(top_level_module_name):
    """
    Unloads all modules named starting with the specified string.

    This is adapted from COMPAS to facilitate dynamic reloading in Rhino/GH.

    Args:
        top_level_module_name (str): The name of the library (e.g., 'geomseq_core')
    """
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
