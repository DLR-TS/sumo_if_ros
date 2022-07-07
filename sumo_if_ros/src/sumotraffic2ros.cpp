/********************************************************************************
 * Copyright (C) 2017-2022 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *   Daniel Heß - initial API and implementation
 *   Matthias Nichting
 ********************************************************************************/
#include <sumo_if_ros/sumotraffic2ros.h>


int main(int argc, char** argv)
{
    adore::sumo_if_ros::SUMOTrafficToROS sumo_traffic_to_ros_node;
    sumo_traffic_to_ros_node.init_rosconnection(argc, argv, 100.0, "sumo_traffic_to_ros_node");
    sumo_traffic_to_ros_node.init_sumo();
    sumo_traffic_to_ros_node.run();
    sumo_traffic_to_ros_node.closeSumo();
    return 0;
}
