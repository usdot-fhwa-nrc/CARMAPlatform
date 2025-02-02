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

#include "autoware_msgs/Lane.h"
#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/TrajectoryPlan.h>

namespace pure_pursuit_wrapper {

class PurePursuitWrapperWorker {
    public:
        // Convert TrajectoryPlanPoint to Waypoint. This is used by TrajectoryPlanHandler.
        autoware_msgs::Waypoint TrajectoryPlanPointToWaypointConverter(double current_time, geometry_msgs::PoseStamped pose, cav_msgs::TrajectoryPlanPoint tpp);
};
}
