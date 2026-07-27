#pragma once

//  #include "AaronLib/AaronLib_API.hpp"
//  using namespace AARONLIB;

#include "AaronLib/Util/Util.hpp"
#include "AaronLib/Util/pid.hpp"
#include "AaronLib/Util/exitcondition.hpp"
#include "AaronLib/Util/FastTrig.hpp"
#include "AaronLib/Util/ScaledIMU.hpp"
#include "AaronLib/Util/Jam_Detector.hpp"
#include "AaronLib/Util/Warnings.hpp"

#include "AaronLib/Drivetrain.hpp"

#include "AaronLib/Tracking/Odom_Tracking.hpp"
#include "AaronLib/Tracking/RCL_Tracking.hpp"
#include "AaronLib/Tracking/MCL_Tracking.hpp"

#include "AaronLib/Movement_Control/MoveToPoint.hpp"
#include "AaronLib/Movement_Control/TurnToPoint.hpp"
#include "AaronLib/Movement_Control/Arc.hpp"
#include "AaronLib/Movement_Control/Boomerang.hpp"
#include "AaronLib/Movement_Control/RAMSETE.hpp"
#include "AaronLib/Movement_Control/Velocity_Profiles.hpp"
#include "AaronLib/Movement_Control/Feedforward.hpp"

#include "AaronLib/Chassis.hpp"
#include "AaronLib/Util/Sensor_Calibrator.hpp"
#include "AaronLib/Movement_Control/VelocityController.hpp"
#include "AaronLib/Movement_Control/LTV.hpp"

#include "AaronLib/Auton_Selector/AutonSelector.hpp"
