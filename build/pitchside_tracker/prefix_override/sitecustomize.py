import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jota/Desktop/uni/magistrale/droni/pitchside_tracker/install/pitchside_tracker'
