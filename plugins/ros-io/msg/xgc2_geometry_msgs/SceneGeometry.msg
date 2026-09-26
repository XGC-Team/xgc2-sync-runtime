# Geometry in the part frame, metres. Cylinder/capsule axis is local Z.
# box: size is full side length; sphere: radius; cylinder: radius and full height.
# capsule: radius and straight cylindrical height; convex: vertices and triangle indices.
string type
geometry_msgs/Vector3 size
float64 radius
float64 height
geometry_msgs/Point[] vertices
uint32[] triangles
