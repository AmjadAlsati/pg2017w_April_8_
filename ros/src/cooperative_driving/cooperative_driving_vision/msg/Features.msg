# Exchange format for extracted features

# Standard ROS message header
Header header
# Width of the image the features where extracted from
uint16 image_width
# Height of the image the features where extracted from
uint16 image_height
# List of points in the image at which a vertical line
# (the lane marker) has been extracted. The list is ordered
# inversely by the y-coordinate, i.e. from bottom of the
# image to the top.
geometry_msgs/Point[] Hlines # z-coordinate unused
geometry_msgs/Point[] Vlines
# The processed frames per second
uint16 PFPS
# List of colored regions found in the image
Region[] regions
uint16 box_width
uint16 box_height