# Current exploration state
std_msgs/Header header

uint8 IDLE = 0
uint8 SEARCHING = 1
uint8 PLANNING = 2
uint8 NAVIGATING = 3
uint8 GOAL_REACHED = 4
uint8 STUCK = 5
uint8 COMPLETE = 6

uint8 state                         # Current state
uint32 current_goal_id              # Active goal cluster ID
geometry_msgs/Point current_goal    # Active goal position
float64 distance_to_goal            # Distance remaining
float64 exploration_progress        # 0-100 percentage explored
string status_message               # Human readable status
