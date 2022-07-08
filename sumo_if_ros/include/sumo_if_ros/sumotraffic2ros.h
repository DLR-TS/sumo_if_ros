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
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include "sumotls2ros.h"
#include <iostream>
#include <libsumo/libsumo.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <adore_if_ros_msg/TrafficParticipantSetSimulation.h>
#include <adore_if_ros_msg/TrafficParticipantSimulation.h>
#include <ros/console.h>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <unordered_map>

namespace adore
{
    namespace sumo_if_ros
    {
        struct Timer
        {
          public:
            double tUTC_;
            Timer() : tUTC_(0.0)
            {
            }
            void receive(const std_msgs::Float64ConstPtr& msg)
            {
                tUTC_ = msg->data;
            }
        };

        struct ROSVehicleSet
        {
          public:
            std::unordered_map<int, adore_if_ros_msg::TrafficParticipantSimulation> data_;  ///<- a mapping from vehicle
                                                                                            ///< id to
                                                                                            ///< latest message
            void receive(adore_if_ros_msg::TrafficParticipantSimulationConstPtr msg)
            {
                // data_.push_back(*msg);
                if (data_.find(msg->simulationID) == data_.end())
                {
                    data_.emplace(msg->simulationID, *msg);
                }
                else
                {
                    data_[msg->simulationID] = *msg;
                }
            }
        };

        class SUMOTrafficToROS
        {
          public:
            SUMOTrafficToROS()
            {
                delta_t_ = 0;
                min_update_period_ = 0.01;
                min_tl_update_period_ = 1;
                last_tl_update_time_ = 0;
            }
            ~SUMOTrafficToROS()
            {
                delete nh_;
            }

          protected:
            ROSVehicleSet rosVehicleSet_;
            ros::NodeHandle* nh_;
            ros::Publisher publisher_;
            ros::Publisher mapem_publisher_;
            ros::Publisher spatem_publisher_;
            SumoTLs2Ros sumotls2ros;
            double last_tl_update_time_;
            ros::Subscriber subscriber1_;
            ros::Subscriber subscriber2_;
            std::string sumo_rosveh_prefix_;
            std::string sumo_rosped_prefix_;
            std::vector<ros::Timer> rostimers_;

            std::unordered_map<std::string, int> sumovehid2int_;
            std::unordered_map<std::string, int> sumopedid2int_;
            int last_assigned_int_id_;
            std::map<int, std::string> replacement_ids_;

            std::vector<std::string> vehidlist_;
            std::vector<std::string> pedidlist_;
            std::vector<std::string> sumo_to_ros_ignore_list_;
            std::vector<int> ros_to_sumo_ignore_list_;

            Timer timer_;
            double tSUMO0;
            double tSUMO;
            double delta_t_;
            double min_update_period_;
            double min_tl_update_period_;

          public:
            ros::NodeHandle* getRosNodeHandle()
            {
                return nh_;
            }
            void run()
            {
                while (nh_->ok())
                {
                    ros::spin();
                }
            }

          protected:
            int getNewIntID()
            {
                if (last_assigned_int_id_ > std::numeric_limits<int>::max() - 2)
                {
                    last_assigned_int_id_ = 1000;
                }
                return ++last_assigned_int_id_;
            }
            void removeVehicle(std::string& id)
            {
                try
                {
                    libsumo::Vehicle::remove(id);
                }
                catch (...)
                {
                    std::cout << "Error: removal of sumo vehicle failed" << std::endl;
                }
            }
            void addVehicle(std::string& id)
            {
                try
                {
                    libsumo::Vehicle::add(id, "");
                }
                catch (...)
                {
                    std::cout << "Error: adding a sumo vehicle failed" << std::endl;
                }
            }
            void setMaxSpeed(std::string& id, double val)
            {
                try
                {
                    libsumo::Vehicle::setMaxSpeed(id, val);
                }
                catch (...)
                {
                    std::cout << "Error: setMaxSpeed for sumo vehicle failed" << std::endl;
                }
            }
            void setSpeed(std::string& id, double val)
            {
                try
                {
                    libsumo::Vehicle::setSpeed(id, val);
                }
                catch (...)
                {
                    std::cout << "Error: setSpeed for sumo vehicle failed" << std::endl;
                }
            }
            void moveToXY(std::string& id, std::string z, int a, double x, double y, double heading, int b)
            {
                try
                {
                    libsumo::Vehicle::moveToXY(id, z, a, x, y, heading, b);
                }
                catch (...)
                {
                    std::cout << "Error: moveToXY for sumo vehicle failed" << std::endl;
                }
            }

          public:
            void runCallback(const ros::TimerEvent& te)
            {
                if (newStep())
                {
                    transferDataSumoToRos();
                    transferDataRosToSumo();
                }
            }

            bool newStep()
            {
                // synchronize SUMO:
                delta_t_ = timer_.tUTC_ - tSUMO + tSUMO0;
                if (delta_t_ >= min_update_period_)
                {
                    libsumo::Simulation::step(timer_.tUTC_ + tSUMO0);
                    double tSUMO_new = libsumo::Simulation::getTime();

                    if (tSUMO_new <= tSUMO)
                    {
                        return false;
                    }
                    tSUMO = tSUMO_new;
                    // get vehicles from sumo
                    vehidlist_ = libsumo::Vehicle::getIDList();
                    pedidlist_ = libsumo::Person::getIDList();
                    return true;
                }
                return false;
            }
            void transferDataSumoToRos()
            {
                // traffic light information
                if (timer_.tUTC_ - last_tl_update_time_ > min_tl_update_period_)
                {
                    auto v2x_mapem = sumotls2ros.getMAPEMFromSUMO(timer_.tUTC_);
                    auto spatems = sumotls2ros.getSPATEMFromSUMO(timer_.tUTC_);

                    // resend static mapem data
                    for (auto&& mapem_item : v2x_mapem)
                        mapem_publisher_.publish(mapem_item.second);

                    // send non-static spatem data
                    for (auto&& spat : spatems)
                        spatem_publisher_.publish(spat);

                    last_tl_update_time_ = timer_.tUTC_;
                }
                // traffic participant information
                if (vehidlist_.size() > 0 || pedidlist_.size() > 0)
                {
                    // message for set of traffic participants
                    adore_if_ros_msg::TrafficParticipantSetSimulation tpset;
                    tpset.simulator = "SUMO";

                    for (auto& id : vehidlist_)
                    {
                        if (id.find(sumo_rosveh_prefix_) == std::string::npos)
                        {
                            // id translation
                            int intid = 0;
                            auto idtranslation = sumovehid2int_.find(id);
                            if (idtranslation == sumovehid2int_.end())
                            {
                                intid = getNewIntID();
                                sumovehid2int_.emplace(id, intid);
                            }
                            else
                            {
                                intid = idtranslation->second;
                            }
                            if (std::find(sumo_to_ros_ignore_list_.begin(), sumo_to_ros_ignore_list_.end(), id) !=
                                sumo_to_ros_ignore_list_.end())
                            {
                                continue;
                            }
                            try
                            {
                                libsumo::TraCIPosition tracipos = libsumo::Vehicle::getPosition(id);
                                double heading = M_PI * 0.5 - libsumo::Vehicle::getAngle(id) / 180.0 * M_PI;
                                double v = libsumo::Vehicle::getSpeed(id);
                                std::string type = libsumo::Vehicle::getTypeID(id);
                                double L = libsumo::Vehicle::getLength(id);
                                double w = libsumo::Vehicle::getWidth(id);
                                double H = libsumo::Vehicle::getHeight(id);
                                int signals = libsumo::Vehicle::getSignals(id);  ///<- bit array! see TraciAPI:669,
                                                                                 ///< VehicleSignal

                                // ros message for single traffic participant
                                adore_if_ros_msg::TrafficParticipantSimulation tp;
                                tp.simulationID = intid;
                                tp.data.v2xStationID = intid;
                                tp.data.time = tSUMO - tSUMO0;
                                tp.data.classification.type_id =
                                    adore_if_ros_msg::TrafficClassification::CAR;  //@TODO: parse sumo type string
                                tp.data.shape.type = 1;
                                tp.data.shape.dimensions.push_back(L);
                                tp.data.shape.dimensions.push_back(w);
                                tp.data.shape.dimensions.push_back(H);
                                auto geopos = libsumo::Simulation::convertGeo(tracipos.x, tracipos.y, false);
                                tp.data.motion_state.pose.pose.position.x = geopos.x;
                                tp.data.motion_state.pose.pose.position.y = geopos.y;
                                tp.data.motion_state.pose.pose.position.z = 0;
                                tf2::Quaternion q;
                                q.setRPY(0.0, 0.0, heading);
                                tp.data.motion_state.pose.pose.orientation.x = q.getX();
                                tp.data.motion_state.pose.pose.orientation.y = q.getY();
                                tp.data.motion_state.pose.pose.orientation.z = q.getZ();
                                tp.data.motion_state.pose.pose.orientation.w = q.getW();
                                tp.data.motion_state.twist.twist.linear.x = v;
                                // add the traffic participant to the set
                                tpset.data.push_back(tp);
                            }
                            catch (...)
                            {
                                std::cout << "Error: failed to get information with libsumo::Vehicle" << std::endl;
                            }
                        }
                    }

                    for (auto& id : pedidlist_)
                    {
                        // ignore pedestrians controlled by other simulator
                        if (id.find(sumo_rosped_prefix_) == std::string::npos)
                        {
                            // id translation
                            int intid = 0;
                            auto idtranslation = sumopedid2int_.find(id);
                            if (idtranslation == sumopedid2int_.end())
                            {
                                intid = getNewIntID();
                                sumopedid2int_.emplace(id, intid);
                            }
                            else
                            {
                                intid = idtranslation->second;
                            }

                            // retrieve person data
                            libsumo::TraCIPosition tracipos = libsumo::Person::getPosition(id);
                            double heading = M_PI * 0.5 - libsumo::Person::getAngle(id) / 180.0 * M_PI;
                            double v = libsumo::Person::getSpeed(id);
                            std::string type = libsumo::Person::getTypeID(id);
                            double L = libsumo::Person::getLength(id);
                            double w = libsumo::Person::getLength(id);
                            double H = 1.75;
                            // not in libsumo?  int signals = libtraci::Person::getSignals(id); ///<- bit array!
                            // see TraciAPI:669, todo int signals = libsumo::Person::getSignals(id);  ///<- bit
                            // array! see TraciAPI:669, VehicleSignal

                            // ros message for single traffic participant
                            adore_if_ros_msg::TrafficParticipantSimulation tp;
                            tp.simulationID = intid;
                            tp.data.time = tSUMO - tSUMO0;
                            tp.data.classification.type_id = adore_if_ros_msg::TrafficClassification::PEDESTRIAN;
                            tp.data.shape.type = 1;
                            tp.data.shape.dimensions.push_back(L);
                            tp.data.shape.dimensions.push_back(w);
                            tp.data.shape.dimensions.push_back(H);
                            auto geopos = libsumo::Simulation::convertGeo(tracipos.x, tracipos.y, false);
                            tp.data.motion_state.pose.pose.position.x = geopos.x;
                            tp.data.motion_state.pose.pose.position.y = geopos.y;
                            tp.data.motion_state.pose.pose.position.z = 0;
                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, heading);
                            tp.data.motion_state.pose.pose.orientation.x = q.getX();
                            tp.data.motion_state.pose.pose.orientation.y = q.getY();
                            tp.data.motion_state.pose.pose.orientation.z = q.getZ();
                            tp.data.motion_state.pose.pose.orientation.w = q.getW();
                            tp.data.motion_state.twist.twist.linear.x = v;
                            // add the traffic participant to the set
                            tpset.data.push_back(tp);
                        }
                    }

                    // write the message
                    publisher_.publish(tpset);
                }
            }
            void transferDataRosToSumo()
            {
                // update sumo with new information from ros
                // TODO std::vector<int> delete_from_rosvehicleset;
                for (auto pair : rosVehicleSet_.data_)
                {
                    auto msg = pair.second;
                    if (std::find(ros_to_sumo_ignore_list_.begin(), ros_to_sumo_ignore_list_.end(), msg.simulationID) !=
                        ros_to_sumo_ignore_list_.end())
                    {
                        continue;
                    }
                    std::string sumoid;
                    auto replacement_id = replacement_ids_.find(msg.simulationID);
                    if (replacement_id == replacement_ids_.end())
                    {
                        std::stringstream ss;
                        ss << sumo_rosveh_prefix_ << msg.simulationID;
                        sumoid = ss.str();
                    }
                    else
                    {
                        sumoid = replacement_id->second;
                    }
                    if (std::find(vehidlist_.begin(), vehidlist_.end(), sumoid) == vehidlist_.end())
                    {
                        addVehicle(sumoid);
                        setMaxSpeed(sumoid, 100.0);
                    }

                    auto p = msg.data.motion_state.pose.pose;
                    tf2::Quaternion q;
                    q.setValue(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    const double heading = (M_PI * 0.5 - yaw) * 180.0 / M_PI;

                    auto sumopos = libsumo::Simulation::convertGeo(p.position.x, p.position.y, true);
                    // following line: keepRoute=2. see
                    // https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#move_to_xy_0xb4 for details
                    moveToXY(sumoid, "", 0, sumopos.x, sumopos.y, heading, 2);
                    setSpeed(sumoid, msg.data.motion_state.twist.twist.linear.x);
                }
            }

            void init_rosconnection(int argc, char** argv, double rate, std::string nodename)
            {
                ros::init(argc, argv, nodename);
                nh_ = new ros::NodeHandle();
                ros::Timer rostimer = nh_->createTimer(ros::Duration(1.0 / rate), &SUMOTrafficToROS::runCallback, this);
                rostimers_.push_back(rostimer);
            }
            void init_sumo()
            {
                int port = -1;
                double step_length = 0.05;
                std::string cfg_file;
                nh_->getParam("/sumo/port", port); //relevant only for libtraci
                nh_->getParam("/sumo/cfg_file", cfg_file);
                nh_->getParam("/sumo/step_length", step_length);
                if (cfg_file.empty())
                {
                    throw std::runtime_error("Error: No config file for sumo provided.");
                }
                // cfg_file_ = "/home/fascar/catkin_ws/src/adore/adore_if_ros_demos/demo005.sumocfg";
                std::vector<std::string> sumoargs;
                sumoargs.push_back("-c");
                sumoargs.push_back(cfg_file);
                sumoargs.push_back("--step-length");
                sumoargs.push_back(std::to_string(step_length));
                if (port > -1)
                {
                    sumoargs.push_back("--remote-port");
                    sumoargs.push_back(std::to_string(port));
                }

                while (nh_->ok())
                {
                    try
                    {
                        std::cout << "load sumo ..." << std::flush;
                        libsumo::Simulation::load(sumoargs);
                        std::cout << " done." << std::endl;
                        break;
                    }
                    catch (const std::exception& exc)
                    {
                        std::cout << exc.what() << std::endl;
                        std::cout << "Arguments for sumo:" << std::endl;
                        for (auto a : sumoargs)
                        {
                            std::cout << a << std::endl;
                        }
                        std::cout << "try again ..." << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                std::cout << "started sumo with config file " << cfg_file << " and step length " << step_length
                          << std::endl;
                tSUMO0 = libsumo::Simulation::getTime();
                tSUMO = tSUMO0;
                publisher_ = nh_->advertise<adore_if_ros_msg::TrafficParticipantSetSimulation>("/SIM/"
                                                                                               "traffic/"
                                                                                               "agg",
                                                                                               5);
                nh_->getParam("PARAMS/V2X_TL/UTMZone", sumotls2ros.utm_zone_);
                nh_->getParam("PARAMS/V2X_TL/SouthHemi", sumotls2ros.is_south_hemi_);
                nh_->getParam("PARAMS/V2X_TL/UseSystemTime", sumotls2ros._use_system_time);
                nh_->getParam("PARAMS/V2X_TL/EnableSPATTiming", sumotls2ros._generate_spat_timing);
                ROS_INFO_STREAM_ONCE("SPAT/MAP Generation Parameters");
                ROS_INFO_STREAM_ONCE("-- UTM-Zone: " << sumotls2ros.utm_zone_);
                ROS_INFO_STREAM_ONCE("-- South. Hemi.: " << sumotls2ros.is_south_hemi_);
                ROS_INFO_STREAM_ONCE("-- use system time: " << sumotls2ros._use_system_time);
                ROS_INFO_STREAM_ONCE("-- enable spat timing: " << sumotls2ros._generate_spat_timing);
                ROS_INFO_STREAM_ONCE("-------------------------------");
                mapem_publisher_ =
                    nh_->advertise<adore_v2x_sim::SimMAPEM>("/SIM/v2x/MAPEM",100);
                spatem_publisher_ =
                    nh_->advertise<adore_v2x_sim::SimSPATEM>("/SIM/v2x/SPATEM",100);
                subscriber1_ = nh_->subscribe<std_msgs::Float64>("/SIM/utc", 1, &Timer::receive, &timer_);
                subscriber2_ = nh_->subscribe<adore_if_ros_msg::TrafficParticipantSimulationConstPtr>(
                    "/SIM/traffic", 100, &ROSVehicleSet::receive, &rosVehicleSet_);
                sumo_rosveh_prefix_ = "rosvehicle";
                sumo_rosped_prefix_ = "rospedestrian";
                last_assigned_int_id_ = 1000;
            }
            void setRosNodeHandle(ros::NodeHandle* nh)
            {
                nh_ = nh;
            }

            void closeSumo()
            {
                libsumo::Simulation::close();
            }
        };
    }  // namespace sumo_if_ros
}  // namespace adore