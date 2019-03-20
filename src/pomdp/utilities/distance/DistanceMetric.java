package pomdp.utilities.distance;

import pomdp.utilities.BeliefState;

public interface DistanceMetric 
{
    double distance(BeliefState bs1, BeliefState bs2);
}
