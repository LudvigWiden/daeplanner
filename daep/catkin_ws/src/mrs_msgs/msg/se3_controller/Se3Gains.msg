# This a set of gains for the SE(3) controller.

# Horizontal position gain.
float64 kpxy

# Horizontal velocity gain.
float64 kvxy

# Horizontal acceleration gain.
float64 kaxy

# Vertical position gain.
float64 kpz

# Vertical velocity gain.
float64 kvz

# Vertical acceleration gain.
float64 kaz

# Body disturbance gain.
float64 kibxy

# Body disturbance estimator limit ([N]).
float64 kibxy_lim

# World disturbance gain.
float64 kiwxy

# World disturbance estimator limit  ([N]).
float64 kiwxy_lim

# Roll+pitch attitude gain.
float64 kqrp

# Yaw attitude gain.
float64 kqy

# Mass estimator gain.
float64 km

# Mass estimator limit ([m]).
float64 km_lim
