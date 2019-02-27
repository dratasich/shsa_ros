Header header
# outputs of the substitutions (these are intervals)
# intervals are transformed to mean=midpoint and variance=size(interval)/2
float32[] mean
float32[] variance
# difference between intervals
float32[] error
# index of substitution failed
int8 failed
