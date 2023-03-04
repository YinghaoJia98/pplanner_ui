#include "pplanner_ui.h"
// pci_initialization_trigger
namespace pplanner_ui
{

  pplanner_panel::pplanner_panel(QWidget *parent) : rviz::Panel(parent)
  {

    global_planner_by_id_client_ =
        nh.serviceClient<pplanner_msgs::pplanner_pub_target>("planner_global");

    Pause_planner_client = nh.serviceClient<std_srvs::Trigger>("pause_tracker");
    Continue_planner_client = nh.serviceClient<std_srvs::Trigger>("continue_tracker");
    Stop_planner_client = nh.serviceClient<std_srvs::Trigger>("stop_tracker");

    QVBoxLayout *v_box_layout = new QVBoxLayout;

    button_global_planner_by_id_ = new QPushButton;
    button_pause_planner = new QPushButton;
    button_continue_planner = new QPushButton;
    button_stop_planner = new QPushButton;

    button_global_planner_by_id_->setText("Run Global Planner By Id");
    button_pause_planner->setText("Pause the tracker");
    button_continue_planner->setText("Continue the tracker");
    button_stop_planner->setText("Stop the tracker");

    QVBoxLayout *global_vbox_layout = new QVBoxLayout;
    QHBoxLayout *global_hbox_layout = new QHBoxLayout;

    QLabel *text_label_ptr = new QLabel("Target ID:");

    global_planner_id_line_edit_ = new QLineEdit();

    global_hbox_layout->addWidget(text_label_ptr);

    global_hbox_layout->addWidget(global_planner_id_line_edit_);

    global_hbox_layout->addWidget(button_global_planner_by_id_);
    global_vbox_layout->addLayout(global_hbox_layout);
    v_box_layout->addLayout(global_vbox_layout);
    v_box_layout->addWidget(button_pause_planner);
    v_box_layout->addWidget(button_continue_planner);
    v_box_layout->addWidget(button_stop_planner);

    setLayout(v_box_layout);

    connect(button_global_planner_by_id_, SIGNAL(clicked()), this,
            SLOT(on_global_planner_by_id_click()));

    connect(button_pause_planner, SIGNAL(clicked()), this,
            SLOT(on_pause_click()));

    connect(button_continue_planner, SIGNAL(clicked()), this,
            SLOT(on_continue_click()));

    connect(button_stop_planner, SIGNAL(clicked()), this,
            SLOT(on_stop_click()));
  }

  void pplanner_panel::on_global_planner_by_id_click()
  {
    // retrieve ID as a string
    std::string in_string = global_planner_id_line_edit_->text().toStdString();
    // global_planner_id_line_edit_->clear();
    int id = -1;
    if (in_string.empty())
      id = 0;
    else
    {
      // try to convert to an integer
      try
      {
        id = std::stoi(in_string);
      }
      catch (const std::out_of_range &exc)
      {
        ROS_ERROR("[PPLANNER UI] - Invalid ID: %s", in_string.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[PPLANNER UI] - Invalid ID: %s", in_string.c_str());
        return;
      }
    }
    // check bounds on integer
    if (id < 0)
    {
      ROS_ERROR("[PPLANNER UI] - In valid ID, must be non-negative");
      return;
    }
    // we got an ID!!!!!!!!!
    ROS_INFO("PPLANNER UI :Global Planner found ID : %i", id);

    pplanner_msgs::pplanner_pub_target plan_srv;
    plan_srv.request.id = id;
    if (!global_planner_by_id_client_.call(plan_srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                global_planner_by_id_client_.getService().c_str());
    }
  }

  void pplanner_panel::on_pause_click()
  {
    std_srvs::Trigger srv;
    if (!Pause_planner_client.call(srv))
    {
      ROS_ERROR("[PPLANNER-UI] Service call failed: %s",
                Pause_planner_client.getService().c_str());
    }
  }

  void pplanner_panel::on_continue_click()
  {
    std_srvs::Trigger srv;
    if (!Continue_planner_client.call(srv))
    {
      ROS_ERROR("[PPLANNER-UI] Service call failed: %s",
                Continue_planner_client.getService().c_str());
    }
  }

  void pplanner_panel::on_stop_click()
  {
    std_srvs::Trigger srv;
    if (!Stop_planner_client.call(srv))
    {
      ROS_ERROR("[PPLANNER-UI] Service call failed: %s",
                Stop_planner_client.getService().c_str());
    }
  }

  void pplanner_panel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }
  void pplanner_panel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pplanner_ui::pplanner_panel, rviz::Panel)
