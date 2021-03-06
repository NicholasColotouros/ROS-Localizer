## *********************************************************
## 
## File autogenerated for the aquajoy package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Deadzone range on individual joypad analog axes', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'joy_axis_deadzone', 'edit_method': '', 'default': 0.15, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum speed command magnitude [for all modes]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_speed_cmd', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum heave command magnitude [for non-depth-reg. modes]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_heave_cmd', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum roll command magnitude [for non-AP mode]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_roll_cmd', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum pitch command magnitude [for non-AP mode]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_pitch_cmd', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum yaw command magnitude [for non-AP mode]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_yaw_cmd', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to roll position ratio (deg) [for AP-pos. modes]', 'max': 180.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_roll_pos', 'edit_method': '', 'default': 90.0, 'level': 0, 'min': -180.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to pitch position ratio (deg) [for AP-pos. modes]', 'max': 180.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_pitch_pos', 'edit_method': '', 'default': 90.0, 'level': 0, 'min': -180.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to yaw position ratio (deg) [UNUSED]', 'max': 180.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_yaw_pos', 'edit_method': '', 'default': 180.0, 'level': 0, 'min': -180.0, 'type': 'double'}, {'srcline': 259, 'description': 'Minimum depth (m) [for AP-depth-reg. modes]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'min_depth', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -2.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum depth (m) [for AP-depth-reg. modes]', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_depth', 'edit_method': '', 'default': 100.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Use fixed initial depth upon entering depth-related modes [if < 0, then initial depth is current robot depth]', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'default_fixed_depth', 'edit_method': '', 'default': -1.0, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to roll velocity ratio (deg/s) [for AP-pos. modes]', 'max': 1800.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_roll_vel', 'edit_method': '', 'default': 450.0, 'level': 0, 'min': -1800.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to pitch velocity ratio (deg/s) [for AP-pos. modes]', 'max': 1800.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_pitch_vel', 'edit_method': '', 'default': 450.0, 'level': 0, 'min': -1800.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to yaw velocity ratio (deg/s) [for AP-pos. modes]', 'max': 1800.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_yaw_vel', 'edit_method': '', 'default': 900.0, 'level': 0, 'min': -1800.0, 'type': 'double'}, {'srcline': 259, 'description': 'Analog stick to depth velocity ratio (m/s) [for AP-pos. modes]', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_depth_vel', 'edit_method': '', 'default': 3.0, 'level': 0, 'min': -10.0, 'type': 'double'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

