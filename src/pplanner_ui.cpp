#include "pplanner_ui.h"
// pci_initialization_trigger
namespace pplanner_ui
{

  pplanner_panel::pplanner_panel(QWidget *parent) : rviz::Panel(parent)
  {

    global_planner_by_id_client =
        nh.serviceClient<pplanner_msgs::pplanner_pub_target>("planner_global");

    QVBoxLayout *v_box_layout = new QVBoxLayout;

    button_global_planner_by_id = new QPushButton;

    button_global_planner_by_id->setText("Run Global Planner By Id");

    QVBoxLayout *global_vbox_layout = new QVBoxLayout;
    QHBoxLayout *global_hbox_layout = new QHBoxLayout;

    QLabel *text_label_ptr = new QLabel("Target ID:");

    global_planner_id_line_edit = new QLineEdit();

    global_hbox_layout->addWidget(text_label_ptr);

    global_hbox_layout->addWidget(global_planner_id_line_edit);

    global_hbox_layout->addWidget(button_global_planner_by_id);
    global_vbox_layout->addLayout(global_hbox_layout);
    v_box_layout->addLayout(global_vbox_layout);

    setLayout(v_box_layout);

    connect(button_global_planner_by_id, SIGNAL(clicked()), this,
            SLOT(on_global_planner_by_id_click()));
  }

  void pplanner_panel::on_global_planner_by_id_click()
  {
    // retrieve ID as a string
    std::string in_string = global_planner_id_line_edit->text().toStdString();
    // global_planner_id_line_edit->clear();
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
    if (!global_planner_by_id_client.call(plan_srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                global_planner_by_id_client.getService().c_str());
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
