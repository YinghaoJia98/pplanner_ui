#ifndef PPLANNER_UI_H
#define PPLANNER_UI_H

#include <stdio.h>

#include <pplanner_msgs/pplanner_pub_target.h>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace pplanner_ui
{
  class pplanner_panel : public rviz::Panel
  {
    Q_OBJECT
  public:
    pplanner_panel(QWidget *parent = 0);
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

  public Q_SLOTS:

    void on_global_planner_by_id_click();
    void on_pause_click();
    void on_continue_click();
    void on_stop_click();
    void on_StartExplorer_click();
    void on_StopExplorer_click();
  protected Q_SLOTS:

  protected:
    QPushButton *button_global_planner_by_id_;
    QLineEdit *global_planner_id_line_edit_;
    ros::ServiceClient global_planner_by_id_client_;

    QPushButton *button_pause_planner;
    ros::ServiceClient Pause_planner_client;

    QPushButton *button_continue_planner;
    ros::ServiceClient Continue_planner_client;

    QPushButton *button_stop_planner;
    ros::ServiceClient Stop_planner_client;

    QPushButton *button_start_explorer;
    ros::ServiceClient Start_explorer_client;

    QPushButton *button_stop_explorer;
    ros::ServiceClient Stop_explorer_client;

    ros::NodeHandle nh;
  };

} // namespace pplanner_ui

#endif // PPLANNER_UI_H
