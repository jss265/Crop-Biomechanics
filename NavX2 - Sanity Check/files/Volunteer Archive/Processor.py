"""
Simple processor module for parsed NavX frames.
Keep processing here minimal; other modules can import `process`.
"""
from navx_types import NavXFrame

def process(frame: NavXFrame):
	"""
	Handle a parsed `NavXFrame`.
	"""
	print(frame)