# Note: corners (of the platform) are hard-coded inside the node (see the macros in both iirob_led_cross.h and iirob_led_rectangle.h)

#geometry_msgs/Vector3 force		# X,Y - used for the direction on the XY plane; Z along with X and Y - used for caluclating the force (lenght of the vector)
#geometry_msgs/TransformStamped tf_stamped

# Lower bound for force: 0
# Upper bound for force: max_force; if provided as additional parameter we need to check if the given value can be "applied" to the LEDs we have the cross)

# NEW FORMAT
geometry_msgs/WrenchStamped force
std_msgs/ColorRGBA color
