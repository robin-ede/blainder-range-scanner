# context.area: VIEW_3D

# Range Scanner Extension for Blender 5.0+
# Manifest-based extension format (blender_manifest.toml)

from .ui import user_interface

# register all needed classes on startup
def register():
    user_interface.register()

# delete all classes on shutdown
def unregister():
    user_interface.unregister()