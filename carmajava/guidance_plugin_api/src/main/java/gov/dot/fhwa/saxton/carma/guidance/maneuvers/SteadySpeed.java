/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

/**
 * Represents a steady-speed longitudinal maneuver.
 * NOTE:  even this maneuver requires a target speed to be specified. Since the vehicle's actual speed may never
 * be suitably close to the desired speed from the previous maneuver, we can't rely on simply continuing with the
 * current actual speed.
 */
public class SteadySpeed extends LongitudinalManeuver {
 
    protected static final double SPEED_EPSILON = 0.05;
    
    public SteadySpeed(IPlugin planner) {
        super(planner);
    }

    /**
     * Since steady speed is intended to continue the current speed, the required distance to complete the maneuver
     * is zero.  Therefore, the end distance will be set to the start distance, and the caller will have the option
     * to set it to whatever larger distance is desired to fill up space in a trajectory.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        //check that both the beginning and end speeds are the same
        if (Math.abs(startSpeed_ - endSpeed_) > SPEED_EPSILON) {
            throw new IllegalStateException("SteadySpeed maneuver being planned with start speed = " + startSpeed_
                                            + ", target speed = " + endSpeed_);
        }

        //set end distance to start distance
        endDist_ = startDist;
        workingAccel_ = 0.5 * maxAccel_;
    }


    @Override
    public double planToTargetDistance(IManeuverInputs inputs, IGuidanceCommands commands, double startDist, double endDist) {
        super.planToTargetDistance(inputs, commands, startDist, endDist);

        //check that both the beginning and end speeds are the same
        if (Math.abs(startSpeed_ - endSpeed_) > SPEED_EPSILON) {
            throw new IllegalStateException("SteadySpeed maneuver being planned with start speed = " + startSpeed_
                                            + ", target speed = " + endSpeed_);
        }

        //set end distance to start distance
        endDist_ = endDist;
        workingAccel_ = 0.5 * maxAccel_;
        return endSpeed_;
    }


    /**
     * Used only for SteadySpeed maneuver, this allows the caller to set the length of the maneuver to whatever is
     * desired.
     * @param endDist - the desired end distance from the beginning of the planned route, m
     */
    public void overrideEndDistance(double endDist) {
        endDist_ = endDist;
    }

    @Override
    public double generateSpeedCommand() throws IllegalStateException {
        //if we are within a slow time step of the maneuver's end, mark it as completed
        double location = inputs_.getDistanceFromRouteStart();
        double prettyClose = 0.2*endSpeed_;
        if (endDist_ - location < prettyClose) {
            completed = true;
        }

        return endSpeed_;
    }


    @Override
    public boolean canPlan(IManeuverInputs inputs, double startDist, double endDist) {
        return true;
    }
}
