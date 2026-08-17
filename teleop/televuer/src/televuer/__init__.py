# unitree_televuer/__init__.py
from .vuer_compat import apply_vuer_video_texture_patch

apply_vuer_video_texture_patch()

from .televuer import TeleVuer
from .tv_wrapper import TeleVuerWrapper, TeleData
