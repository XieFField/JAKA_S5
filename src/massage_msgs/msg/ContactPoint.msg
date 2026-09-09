string body_a
string body_b
string geom_a
string geom_b
# World frame. Normal points from geom_a toward geom_b.
geometry_msgs/Point position
geometry_msgs/Vector3 normal
# m, positive only for penetration; N, compressive force magnitude.
float64 penetration
float64 normal_force
bool massage_pair
