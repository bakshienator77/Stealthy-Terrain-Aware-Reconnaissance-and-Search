# Diagnostic data presented in safety manager's display

# Potential risks ahead
float32 distance_to_obstacle
float32 time_to_obstacle

# Current action taken by safety manager
uint8 state

uint8 NONE=0
uint8 SLOWING_DOWN=1
uint8 STOPPING=2
