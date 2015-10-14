
topic_list = ['/imu/data', '/joy', '/odom', '/rotunit_joint_states', '/scan360', '/volksbot_joint_states', '/velodyne_packets']

from subprocess import call
import sys

expression = 'topic in ' + str(topic_list)
print 'filtering bag files with expression:', expression

try:
  for file in sys.argv[1:]:
    if not file.endswith('.bag'):
      print file, 'is not a bag file!'
      continue

    name = file.split('.bag')[0]
    out_bag_name = name + '_filtered.bag'
    print 'filter bag', file, 'to', out_bag_name
    call(['rosbag', 'filter', file, out_bag_name, expression])
except:
  print '... aborted!'
