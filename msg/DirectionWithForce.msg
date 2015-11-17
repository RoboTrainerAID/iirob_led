# Lower bound for force: 0
# Upper bound for force: currently set to 10; if provided as additional parameter we need to check if the given value can be "applied" to the LEDs we have
# Note: corners (of the platform) are hard-coded inside the node
geometry_msgs/Vector3 force		# X,Y - used for the direction on the XY plane; Z along with X and Y - used for caluclating the force (lenght of the vector)
geometry_msgs/TransformStamped tf_stamped
