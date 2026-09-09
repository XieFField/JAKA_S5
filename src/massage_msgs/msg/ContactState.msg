std_msgs/Header header
massage_msgs/ContactPoint[] contacts
# Designated tool/target pair AND normal force threshold.
bool in_contact
bool over_force
bool impact
bool unexpected_contact
float64 normal_force
float64 force_rate
# Edges only: CONTACT_STARTED, CONTACT_ENDED, OVER_FORCE, IMPACT,
# UNEXPECTED_CONTACT, and the corresponding *_CLEARED events.
string[] events
