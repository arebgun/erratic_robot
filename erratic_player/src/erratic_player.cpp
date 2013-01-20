/*
 * erratic_player
 * Copyright (c) 2010, Antons Rebguns.
 * Copyright (c) 2010, Pablo Inigo Blasco. RTC Group - Universidad de Sevilla.
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b erratic_player is a driver for the Erratic mobile robot, available from
<a href="http://www.videredesign.com">Videre Design</a>.

This node wraps up the Player @b erratic driver.  For detailed documentation,
consult <a href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__erratic.html">Player erratic documentation</a>.

<hr>

@section usage Usage
@verbatim
$ erratic_player [standard ROS args]
@endverbatim

@par Example

@verbatim
$ erratic_player
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b "cmd_vision_tilt"/Float64 : angle command to the vision tilt servo actuator (rads, from -1.5 to 1.5, home 0.2).
- @b "cmd_vision_pan"/Float64 : angle command to the vision pan servo actuator (rads, from -1.45 to 1.45, home 0).
- @b "cmd_ranger_tilt"/Float64 : angle command to the ranger tilt servo actuator (rads, from -1.5 to 0.95, home 0).

Publishes to (name / type):
- @b "odom"/Odometry : odometry data from the robot.
- @b "battery_state"/PowerState : battery data. Since the robot does not have any charge detector, it uses the empirical result that the robot is charging if V>12.98
- @b "erratic_player/RangeArray": Sonar Data.

<hr>

@section parameters ROS parameters

- @b port_name (string) : Port that the robot base is connected to.
- @b max_trans_vel (string) : Maximum translational velocity. Default: 0.5 m/s.
- @b max_rot_vel (string) : Maximum rotational velocity. Default: 100 deg/s.
- @b trans_acc (string) : Maximum translational acceleration, in length/sec/sec; nonnegative.
    Zero means use the robot's default value. Default: 0.
- @b trans_decel (string) : Maximum translational deceleration, in length/sec/sec; nonpositive.
    Zero means use the robot's default value. Default: trans_acc.
- @b rot_acc (string) : Maximum rotational acceleration, in angle/sec/sec; nonnegative.
    Zero means use the robot's default value. Default: 0.
- @b rot_decel (string) : Maximum rotational deceleration, in angle/sec/sec; nonpositive.
    Zero means use the robot's default value. Default: rot_acc.
- @b enable_ir (bool) : Whether to enable IR sensors. Default: false.
- @b enable_sonar (bool)  : Whether to enable Sonar sensors. Default: false.
- @b enable_vision_pan_tilt (bool)  : Whether to enable vision servo motors on erratic. Default: false.
- @b enable_ranger_tilt (bool)  : Whether to enable ranger servo motor on erratic. Default: false.
- @b odometry_frame_id (string) : Frame name which will be used to send stamped transforms
    through the TF mechanism. Default: odom.

 **/

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#include <erratic_player/BatteryState.h>
#include <erratic_player/RangeArray.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* Erratic_Init(ConfigFile* cf, int section);

class ErraticNode
{
public:
    QueuePointer q;
    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher odom_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher ir_pub_;
    ros::Publisher sonar_pub;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_ranger_tilt_sub_;
    ros::Subscriber cmd_vision_tilt_sub_;
    ros::Subscriber cmd_vision_pan_sub_;
    std::string tf_prefix_;

    bool publish_tf_;
    double sigma_x_;
    double sigma_y_;
    double sigma_theta_;

    ErraticNode() : watts_charging_(10), watts_unplugged_(-10), charging_threshold_(12.98)
    {
        ros::NodeHandle private_nh("~");

        std::string port = "/dev/ttyUSB0";
        std::string max_trans_vel = "0.5";
        std::string max_rot_vel = "100";
        std::string trans_acc = "0";   // use robot's default value
        std::string trans_decel = trans_acc;
        std::string rot_acc = "0";     // use robot's default value
        std::string rot_decel = rot_acc;

        private_nh.getParam("port_name", port);
        private_nh.getParam("max_trans_vel", max_trans_vel);
        private_nh.getParam("max_rot_vel", max_rot_vel);
        private_nh.getParam("trans_acc", trans_acc);
        private_nh.getParam("trans_decel", trans_decel);
        private_nh.getParam("rot_acc", rot_acc);
        private_nh.getParam("rot_decel", rot_decel);

        private_nh.param("enable_ir", enable_ir, false);
        private_nh.param("enable_sonar", enable_sonar, false);
        private_nh.param("enable_vision_pan_tilt", enable_vision_pan_tilt, false);
        private_nh.param("enable_ranger_tilt", enable_ranger_tilt, false);
        private_nh.param("publish_tf", publish_tf_, true);
        private_nh.param("odometry_frame_id", odom_frame_id, std::string("odom"));

        private_nh.param<double>("x_stddev", sigma_x_, 0.006);
        private_nh.param<double>("y_stddev", sigma_y_, 0.006);
        private_nh.param<double>("rotation_stddev", sigma_theta_, 0.051);

        tf_prefix_ = tf::getPrefixParam(node_);

        // libplayercore boiler plate
        player_globals_init();
        itable_init();

        // TODO: remove XDR dependency
        playerxdr_ftable_init();

        //create publishers for odometry and battery information
        odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1);
        battery_pub_ = node_.advertise<erratic_player::BatteryState>("battery_state", 1);
        if (enable_ir) { ir_pub_ = node_.advertise<erratic_player::RangeArray>("ir", 1); }
        if (enable_sonar) { sonar_pub = node_.advertise<erratic_player::RangeArray>("sonar",1); }

        // The Player address that will be assigned to this device.  The format
        // is interface:index.  The interface must match what the driver is
        // expecting to provide.  The value of the index doesn't really matter,
        // but 0 is most common.
        const char* player_addr_pos = "position2d:0";
        const char* player_addr_power = "power:0";
        const char* player_addr_ir = "ir:0";
        const char* player_addr_sonar="sonar:0";
        const char* player_addr_vision_ptz="ptz:0";
        const char* player_addr_ranger_tilt="tilt:::ptz:1";

        // Create a ConfigFile object, into which we'll stuff parameters.
        // Drivers assume that this object will persist throughout execution
        // (e.g., they store pointers to data inside it).  So it must NOT be
        // deleted until after the driver is shut down.
        this->cf = new ConfigFile();

        // Insert (name,value) pairs into the ConfigFile object.  These would
        // presumably come from the param server
        int field_id = 0;

        this->cf->InsertFieldValue(field_id++, "provides", player_addr_pos);
        this->cf->InsertFieldValue(field_id++, "provides", player_addr_power);

        // optional erratic accessories
        if (enable_ir) { this->cf->InsertFieldValue(field_id++, "provides", player_addr_ir); }
        if (enable_sonar) {this->cf->InsertFieldValue(field_id++,"provides", player_addr_sonar);}
        if (enable_vision_pan_tilt) {this->cf->InsertFieldValue(field_id++,"provides", player_addr_vision_ptz);}
        if (enable_ranger_tilt) {this->cf->InsertFieldValue(field_id++,"provides", player_addr_ranger_tilt);}

        this->cf->InsertFieldValue(0, "port", port.c_str());
        this->cf->InsertFieldValue(0, "max_trans_vel", max_trans_vel.c_str());
        this->cf->InsertFieldValue(0, "max_rot_vel", max_rot_vel.c_str());
        this->cf->InsertFieldValue(0, "trans_acc", trans_acc.c_str());
        this->cf->InsertFieldValue(0, "trans_decel", trans_decel.c_str());
        this->cf->InsertFieldValue(0, "rot_acc", rot_acc.c_str());
        this->cf->InsertFieldValue(0, "rot_decel", rot_decel.c_str());

        // Create an instance of the driver, passing it the ConfigFile object.
        // The -1 tells it to look into the "global" section of the ConfigFile,
        // which is where ConfigFile::InsertFieldValue() put the parameters.
        assert((this->driver = Erratic_Init(cf, -1)));

        // Print out warnings about parameters that were set, but which the
        // driver never looked at.
        cf->WarnUnused();

        // Grab from the global deviceTable a pointer to the Device that was
        // created as part of the driver's initialization.
        assert((this->pos_device = deviceTable->GetDevice(player_addr_pos, false)));
        assert((this->power_device = deviceTable->GetDevice(player_addr_power, false)));

        if (enable_ir) { assert((this->ir_device = deviceTable->GetDevice(player_addr_ir, false))); }
        if (enable_sonar) { assert((this->sonar_device = deviceTable->GetDevice(player_addr_sonar,false))); }
        if (enable_vision_pan_tilt) { assert((this->vision_ptz_device = deviceTable->GetDevice(player_addr_vision_ptz,false))); }
        if (enable_ranger_tilt) { assert((this->ranger_tilt_device = deviceTable->GetDevice(player_addr_ranger_tilt,false))); }

        // Create a message queue
        this->q = QueuePointer(false, PLAYER_QUEUE_LEN);
    }

    ~ErraticNode()
    {
        delete cf;
        player_globals_fini();
    }

    int start()
    {
        // Subscribe to device, which causes it to startup
        if (this->pos_device->Subscribe(this->q) != 0)
        {
            ROS_ERROR("Failed to start the odometry device");
            return(-1);
        }

        if (this->power_device->Subscribe(this->q) != 0)
        {
            ROS_ERROR("Failed to start the power device");
            return(-1);
        }

        if (enable_ir && (this->ir_device->Subscribe(this->q) != 0))
        {
            ROS_ERROR("Failed to start the ir device");
            return(-1);
        }

        if (enable_sonar && (this->sonar_device->Subscribe(this->q) != 0))
        {
            ROS_ERROR("Failed to start the sonar device");
            return(-1);
        }

        cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ErraticNode::cmdVelReceived, this, _1));

        if (enable_vision_pan_tilt)
        {
            cmd_vision_pan_sub_ = node_.subscribe<std_msgs::Float64>("cmd_vision_pan", 1, boost::bind(&ErraticNode::cmdVisionPanReceived,this,_1));
            cmd_vision_tilt_sub_ = node_.subscribe<std_msgs::Float64>("cmd_vision_tilt", 1, boost::bind(&ErraticNode::cmdVisionTiltReceived,this,_1));
        }

        if (enable_ranger_tilt) { cmd_ranger_tilt_sub_=node_.subscribe<std_msgs::Float64>("cmd_ranger_tilt", 1, boost::bind(&ErraticNode::cmdRangerTiltReceived,this,_1)); }

        return(0);
    }

    int stop()
    {
        int status = 0;

        // Unsubscribe from the device, which causes it to shutdown
        if (this->pos_device->Unsubscribe(this->q) != 0)
        {
            ROS_WARN("Failed to stop the odometry device");
            status = -1;
        }

        if (this->power_device->Unsubscribe(this->q) != 0)
        {
            ROS_WARN("Failed to stop the power device");
            status = -1;
        }

        if (enable_ir && (this->ir_device->Unsubscribe(this->q) != 0))
        {
            ROS_WARN("Failed to stop the ir device");
            status = -1;
        }

        if (enable_sonar && (this->sonar_device->Unsubscribe(this->q) != 0))
        {
            ROS_WARN("Failed to stop the sonar device");
            status = -1;
        }

        if (enable_vision_pan_tilt && (this->vision_ptz_device->Unsubscribe(this->q) != 0))
        {
            ROS_WARN("Failed to stop the vision pan/tilt device");
            status = -1;
        }

        if (enable_ranger_tilt && (this->ranger_tilt_device->Unsubscribe(this->q) != 0))
        {
            ROS_WARN("Failed to stop the ranger tilt device");
            status = -1;
        }

        // Give the driver a chance to shutdown.  Wish there were a way to
        // detect when that happened.
        usleep(1000000);
        return(status);
    }

    int setMotorState(uint8_t state)
    {
        Message* msg;

        // Enable the motors
        player_position2d_power_config_t motorconfig;
        motorconfig.state = state;

        if (!(msg = this->pos_device->Request(this->q,
                                              PLAYER_MSGTYPE_REQ,
                                              PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                              (void*) &motorconfig,
                                              sizeof(motorconfig), NULL, false)))
        {
            return(-1);
        }
        else
        {
            delete msg;
            return(0);
        }
    }


    void getCenter()
    {
        Message* msg = NULL;

        //Wait until there is a message for geometry
        while (!(msg = this->pos_device->Request(this->q, PLAYER_MSGTYPE_REQ,
                                                 PLAYER_POSITION2D_REQ_GET_GEOM,
                                                 NULL, 0, NULL, false)))
        {
            ROS_ERROR("No geom for the robot from player (yet). Waiting one second and trying again.");
            ros::Duration(1, 0).sleep();
        }

        player_position2d_geom_t* geomconfig = (player_position2d_geom_t*) msg->GetPayload();
        center_x_ = geomconfig->pose.px;
        center_y_ = geomconfig->pose.py;
        center_yaw_ = geomconfig->pose.pyaw;
        ROS_INFO("Robot center at (%fm %fm), yaw %f radians. Width %fm, length %fm", center_x_, center_y_, center_yaw_, geomconfig->size.sw, geomconfig->size.sl);
        delete msg;
    }

    void cmdRangerTiltReceived (const std_msgs::Float64::ConstPtr& tilt)
    {
        player_ptz_cmd req;
        memset(&req, 0, sizeof(req));

        req.tilt=tilt->data;

        this->ranger_tilt_device->PutMsg(this->q,
                                         PLAYER_MSGTYPE_CMD,
                                         PLAYER_PTZ_CMD_STATE,
                                         (void*) &req,
                                         sizeof(req),NULL);
    }

    void updateVisionPanTilt()
    {
        player_ptz_cmd req;
        memset(&req, 0, sizeof(req));

        req.pan = vision_pan_rads;
        req.tilt = vision_tilt_rads;

        this->vision_ptz_device->PutMsg(this->q,
                                        PLAYER_MSGTYPE_CMD,
                                        PLAYER_PTZ_CMD_STATE,
                                        (void*) &req,
                                        sizeof(req),NULL);
    }

    void cmdVisionTiltReceived(const std_msgs::Float64::ConstPtr& tilt)
    {
        vision_tilt_rads = tilt->data;
        updateVisionPanTilt();
    }

    void cmdVisionPanReceived(const std_msgs::Float64::ConstPtr& pan)
    {
        vision_pan_rads = pan->data;
        updateVisionPanTilt();
    }

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        player_position2d_cmd_vel_t cmd;
        memset(&cmd, 0, sizeof(cmd));

        cmd.vel.px = cmd_vel->linear.x;
        cmd.vel.py = 0.0;
        cmd.vel.pa = cmd_vel->angular.z;
        cmd.state = 1;

        this->pos_device->PutMsg(this->q,
                                 PLAYER_MSGTYPE_CMD,
                                 PLAYER_POSITION2D_CMD_VEL,
                                 (void*) &cmd,sizeof(cmd), NULL);
    }

    void populateCovariance(nav_msgs::Odometry &msg)
    {
        if (fabs(msg.twist.twist.linear.x) <= 1e-8 &&
            fabs(msg.twist.twist.linear.y) <= 1e-8 &&
            fabs(msg.twist.twist.linear.z) <= 1e-8)
        {
            //nav_msgs::Odometry has a 6x6 covariance matrix
            msg.pose.covariance[0] = 1e-12;
            msg.pose.covariance[7] = 1e-12;
            msg.pose.covariance[35] = 1e-12;
        }
        else
        {
            // nav_msgs::Odometry has a 6x6 covariance matrix
            msg.pose.covariance[0] = pow(sigma_x_, 2);
            msg.pose.covariance[7] = pow(sigma_y_, 2);
            msg.pose.covariance[35] = pow(sigma_theta_, 2);
        }

        msg.pose.covariance[14] = DBL_MAX;
        msg.pose.covariance[21] = DBL_MAX;
        msg.pose.covariance[28] = DBL_MAX;

        msg.twist.covariance = msg.pose.covariance;
    }

    void doUpdate()
    {
        Message* msg = NULL;

        // Block until there's a message on the queue
        q->Wait();

        // Pop off one message (we own the resulting memory)
        if (!(msg = q->Pop()))
        {
            ROS_WARN("Empty message queue, no messages to pop.");
            return;
        }

        // Is the message one we care about?
        player_msghdr_t* hdr = msg->GetHeader();

        if ((hdr->type == PLAYER_MSGTYPE_DATA) &&
            (hdr->subtype == PLAYER_POSITION2D_DATA_STATE) &&
            (hdr->addr.interf == PLAYER_POSITION2D_CODE))
        {
            // Cast the message payload appropriately
            player_position2d_data_t* pdata = (player_position2d_data_t*) msg->GetPayload();

            nav_msgs::Odometry odom;

            // Translate from Player data to ROS data
            odom.pose.pose.position.x = pdata->pos.px;
            odom.pose.pose.position.y = pdata->pos.py;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pdata->pos.pa);

            odom.twist.twist.linear.x = pdata->vel.px;
            odom.twist.twist.linear.y = pdata->vel.py;
            odom.twist.twist.angular.z = pdata->vel.pa;

            //@todo TODO: Think about publishing stall information with odometry or on a separate topic
            //odom.stall = pdata->stall;

            odom.header.frame_id = tf::resolve(tf_prefix_, odom_frame_id);
            odom.header.stamp.sec = (long long unsigned int) floor(hdr->timestamp);
            odom.header.stamp.nsec = (long long unsigned int) ((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL);

            populateCovariance(odom);

            // Publish the new data
            odom_pub_.publish(odom);

            if (publish_tf_)
            {
                tf_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(pdata->pos.pa),
                                                                     tf::Point(pdata->pos.px,
                                                                               pdata->pos.py,
                                                                               0.0)),
                                                       ros::Time(odom.header.stamp.sec, odom.header.stamp.nsec),
                                                       odom_frame_id,
                                                       "base_footprint"));
           }

            //printf("Published new odom: (%.3f,%.3f,%.3f)\n",
            //odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.th);
        }
        else if ((hdr->type == PLAYER_MSGTYPE_DATA) &&
                    (hdr->subtype == PLAYER_POWER_DATA_STATE) &&
                    (hdr->addr.interf == PLAYER_POWER_CODE))
        {
            player_power_data_t* pdata = (player_power_data_t*) msg->GetPayload();
            erratic_player::BatteryState state;

            if (pdata->percent < 25) { ROS_WARN("Battery capacity is at %f%% (%fV), please recharge", pdata->percent, pdata->volts); }

            state.header.stamp = ros::Time::now();
            state.capacity = pdata->percent;
            state.charge = pdata->volts;

            ROS_DEBUG("percent = %f, volts = %f", pdata->percent, pdata->volts);
            //state.time_remaining.fromSec((pdata->volts < 11.5) ? 0 : 3600); //need to calculate the remaing runtime based on the batteries discharge curve, for now stop when voltage is below 11.5. -Curt
            //state.power_consumption = (pdata->volts > charging_threshold_) ? watts_charging_ : watts_unplugged_; //Does not work, as they don't publish this
            //state.AC_present = (pdata->volts > charging_threshold_) ? 1 : 0; // are we charging?

            battery_pub_.publish(state);
        }
        else if ((hdr->type == PLAYER_MSGTYPE_DATA) &&
                 (hdr->subtype == PLAYER_SONAR_DATA_RANGES) &&
                 (hdr->addr.interf == PLAYER_SONAR_CODE))
        {
            //ROS_INFO("Received Sonar msg type-> %d",hdr->addr.interf);
            player_sonar_data_t* pdata = (player_sonar_data_t*) msg->GetPayload();

            erratic_player::RangeArray rangerArray;
            rangerArray.ranges.resize(pdata->ranges_count);

            for (uint32_t i = 0 ; i < pdata->ranges_count; ++i)
            {
                //ROS_INFO("Sonar(%d) = %f m", i, pdata->ranges[i]);
                rangerArray.ranges[i].header.stamp = ros::Time::now();
                rangerArray.ranges[i].header.frame_id = std::string("erratic_sonar_");
                rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
                rangerArray.ranges[i].range = pdata->ranges[i];

                // 30 degrees following the erratic manual
                rangerArray.ranges[i].field_of_view = 0.5236085;
                rangerArray.ranges[i].max_range = 5;
                rangerArray.ranges[i].min_range = 0;
                rangerArray.ranges[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
            }

            sonar_pub.publish(rangerArray);
        }
        else if ((hdr->type == PLAYER_MSGTYPE_DATA) &&
                 (hdr->subtype == PLAYER_IR_DATA_RANGES) &&
                 (hdr->addr.interf == PLAYER_IR_CODE))
        {
            player_ir_data_t* pdata = (player_ir_data_t*) msg->GetPayload();

            ros::Time timestamp = ros::Time::now();
            erratic_player::RangeArray rangerArray;
            rangerArray.ranges.resize(pdata->ranges_count);

            ROS_DEBUG("Received IR msg type, ranges_count -> %d, voltages_count: %d", pdata->ranges_count, pdata->voltages_count);
            // TODO: figure out why this segfaults

            for (uint32_t i = 0 ; i < pdata->ranges_count; ++i)
            {
                rangerArray.ranges[i].header.stamp = timestamp;
                rangerArray.ranges[i].header.frame_id = std::string("erratic_ir_");
                rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
                rangerArray.ranges[i].range = pdata->ranges[i];
                rangerArray.ranges[i].radiation_type = sensor_msgs::Range::INFRARED;
            }

            ir_pub_.publish(rangerArray);
        }
        else
        {
            ROS_WARN("Unhandled Player message %d:%d:%d:%d",
                     hdr->type,
                     hdr->subtype,
                     hdr->addr.interf,
                     hdr->addr.index);
        }

        // We're done with the message now
        delete msg;
    }

private:
    double center_x_;
    double center_y_;
    double center_yaw_;

    double watts_charging_;
    double watts_unplugged_;
    double charging_threshold_;

    float vision_pan_rads;
    float vision_tilt_rads;

    Driver* driver;
    Device* pos_device;
    Device* power_device;
    Device* sonar_device;
    Device* ir_device;
    Device* ranger_tilt_device;
    Device* vision_ptz_device;

    ConfigFile* cf;
    std::string odom_frame_id;

    bool enable_ir;
    bool enable_sonar;
    bool enable_ranger_tilt;
    bool enable_vision_pan_tilt;
};

void spinThread()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "erratic_player");

    ErraticNode erratic;

    ros::NodeHandle n;
    boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

    // Start up the robot
    if (erratic.start() != 0)
    {
        exit(-1);
    }

    // Enable the motors
    if (erratic.setMotorState(1) < 0)
    {
        ROS_ERROR("Failed to enable motors");
    }

    erratic.getCenter();

    /////////////////////////////////////////////////////////////////
    // Main loop; grab messages off our queue and republish them via ROS
    while (n.ok())
    {
        erratic.doUpdate();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    erratic.stop();

    spin_thread.join();

    // To quote Morgan, Hooray!
    return(0);
}
