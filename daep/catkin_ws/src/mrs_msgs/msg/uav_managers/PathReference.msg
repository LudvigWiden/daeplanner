# The Path reference

Header header

# True if the heading part should be tracked, otherwise the tracker should maintain the current heading.
bool use_heading

# True if a planner should start being tracked immediately after receiving it.
# Beware the header time stamp matters in this case.
bool fly_now

# True if the path should be looped.
bool loop

# The list of points in the path.
Reference[] points 
