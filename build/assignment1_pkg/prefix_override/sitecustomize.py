import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/daimen/Documents/TEL280_assignments/install/assignment1_pkg'
