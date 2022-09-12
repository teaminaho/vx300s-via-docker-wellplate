#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>

#include <QString>
#include <QMap>

#include "arm_ctrl_rviz.hpp"

namespace arm_ctrl_rviz
{

    typedef std::tuple<QString, uint8_t> QStringTuple;

    ArmControlPanel::ArmControlPanel(QWidget *parent) : rviz::Panel(parent)
    {
        QVBoxLayout *v_layout = new QVBoxLayout();
        QHBoxLayout *row_1 = new QHBoxLayout();
        QHBoxLayout *row_2 = new QHBoxLayout();

        run_button_ = new QPushButton(tr("Run"));
        cancel_button_ = new QPushButton(tr("Cancel"));
        cleanup_button_ = new QPushButton(tr("Cleanup"));

        motion_combo_box_ = new QComboBox();
        motion_item_map_ = new QMap<QString, uint8_t>();

        {
            std::vector<QStringTuple> table;
            table.push_back({tr("Incubator->Stage"), 11});
            table.push_back({tr("Stage->Incubator"), 12});
            table.push_back({"-----", 255});
            table.push_back({tr("Home"), 0});
            table.push_back({tr("Home and Sleep"), 100});
            table.push_back({tr("Gripper Open"), 101});
            table.push_back({tr("Gripper Close"), 102});
            table.push_back({"-----", 255});
            table.push_back({tr("Open Incubator Door"), 1});
            table.push_back({tr("Close Incubator Door"), 2});
            table.push_back({tr("Picking from Incubator"), 3});
            table.push_back({tr("Picking from Stage"), 4});

            for (auto itr = table.begin(); itr != table.end(); itr++)
            {
                auto [name, id] = *itr;
                if (name == "-----")
                {
                    motion_combo_box_->insertSeparator(motion_combo_box_->count());
                }
                else
                {
                    motion_item_map_->insert(name, id);
                    motion_combo_box_->addItem(name);
                }
            }
        }

        row_1->addWidget(new QLabel(tr("Motion:")));
        row_1->addWidget(motion_combo_box_);
        row_2->addWidget(run_button_);
        row_2->addWidget(cancel_button_);
        row_2->addWidget(cleanup_button_);

        v_layout->addLayout(row_1);
        v_layout->addLayout(row_2);
        setLayout(v_layout);

        QObject::connect(run_button_, SIGNAL(clicked()), SLOT(run()));
        QObject::connect(cancel_button_, SIGNAL(clicked()), SLOT(cancel()));
        QObject::connect(cleanup_button_, SIGNAL(clicked()), SLOT(cleanup()));

        ctrl_health_ = nh_.subscribe<std_msgs::Time>("/arm_ctrl/health", 1,
                                                     boost::bind(&ArmControlPanel::ctrlHealthCallback, this, _1));
        ctrl_watchdog_ = nh_.createTimer(ros::Duration(0.2),
                                         boost::bind(&ArmControlPanel::ctrlWatchdogCallback, this, _1));

        action_client_ = new Client("/arm_ctrl/run_arm");

        run_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/arm_ctrl/clear_latch");
    }

    void ArmControlPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void ArmControlPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }

    void ArmControlPanel::run()
    {
        uint8_t motion_id = motion_item_map_->value(motion_combo_box_->currentText());
        if (motion_id == 255)
        {
            return;
        }
        else
        {
            run_button_->setEnabled(false);
            update();

            arm_ctrl_msgs::ArmControlGoal goal;
            goal.motion_id = motion_id;
            action_client_->sendGoal(goal,
                                     boost::bind(&ArmControlPanel::doneCallback, this, _1, _2),
                                     boost::bind(&ArmControlPanel::activeCallback, this),
                                     boost::bind(&ArmControlPanel::feedbackCallback, this, _1));
            ROS_INFO("RUN sendGoal");
        }
    }

    void ArmControlPanel::doneCallback(const actionlib::SimpleClientGoalState &state, const arm_ctrl_msgs::ArmControlResultConstPtr &result)
    {
        ROS_INFO("%s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
        {
            run_button_->setEnabled(true);
            update();
        }
    }

    void ArmControlPanel::activeCallback()
    {
        ROS_INFO("Active");
    }

    void ArmControlPanel::feedbackCallback(const arm_ctrl_msgs::ArmControlFeedbackConstPtr &feedback)
    {
        ROS_INFO("Feedback");
        ROS_INFO("%d", feedback->step);
    }

    void ArmControlPanel::ctrlHealthCallback(const std_msgs::TimeConstPtr &msg)
    {
        ctrl_last_health_ = msg->data;
    }

    void ArmControlPanel::ctrlWatchdogCallback(const ros::TimerEvent &evt)
    {
        static uint8_t count = 5;
        static bool initial = true;

        if (initial && count > 0)
        {
            count--;
            return;
        }
        else
        {
            initial = false;
        }

        ros::Duration d = ros::Time::now() - ctrl_last_health_;
        if (d.toSec() > 0.5)
        {
            QMessageBox::warning(this,
                                 tr("Warning"),
                                 tr("Control Service will be down.\nMake sure actual robot arm status,\nthen restart the service manually."));
        }
    }

    void ArmControlPanel::cancel()
    {
        action_client_->cancelGoal();
    }

    void ArmControlPanel::cleanup()
    {
        ROS_INFO("Cleanup");
        if (run_srv_client_.exists())
        {
            std_srvs::Trigger srv = std_srvs::Trigger();
            if (run_srv_client_.call(srv) && srv.response.success)
            {
                run_button_->setEnabled(true);
                update();
            }
            else
            {
                QMessageBox::warning(this, tr("Warning"), tr("Failed to perform cleanup. Please try restarting applications."));
            }
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arm_ctrl_rviz::ArmControlPanel, rviz::Panel)
