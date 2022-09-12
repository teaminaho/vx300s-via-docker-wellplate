#pragma once

#include <QPushButton>
#include <QComboBox>
#include <QMap>
#include <QTimer>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <std_srvs/Trigger.h>
#include <std_msgs/Time.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_ctrl_msgs/ArmControlAction.h>

namespace arm_ctrl_rviz
{
    typedef actionlib::SimpleActionClient<arm_ctrl_msgs::ArmControlAction> Client;
    class ArmControlPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        ArmControlPanel(QWidget *parent = 0);

        virtual void load(const rviz::Config &config);
        virtual void save(rviz::Config config) const;

        void doneCallback(const actionlib::SimpleClientGoalState &, const arm_ctrl_msgs::ArmControlResultConstPtr &);
        void activeCallback();
        void feedbackCallback(const arm_ctrl_msgs::ArmControlFeedbackConstPtr &);
        void ctrlHealthCallback(const std_msgs::TimeConstPtr &);
        void ctrlWatchdogCallback(const ros::TimerEvent &);

    public Q_SLOTS:
        void run();
        void cancel();
        void cleanup();

    protected:
        ros::NodeHandle nh_;

    private:
        ros::ServiceClient run_srv_client_;
        ros::Subscriber ctrl_health_;
        ros::Time ctrl_last_health_;
        ros::Timer ctrl_watchdog_;

        Client *action_client_;

        QPushButton *run_button_;
        QPushButton *cleanup_button_;
        QPushButton *cancel_button_;
        QComboBox *motion_combo_box_;

        QMap<QString, uint8_t> *motion_item_map_;
    };
}