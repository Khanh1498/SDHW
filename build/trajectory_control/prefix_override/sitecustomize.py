import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jonny/github-classroom/ITU-EMAV/trajectory-tracking-control-homework-2-team-jh/install/trajectory_control'
