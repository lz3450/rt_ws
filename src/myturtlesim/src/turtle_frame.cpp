#include <cstdlib>
#include <ctime>

#include <QPointF>

#include "myturtlesim/turtle_frame.h"

#define DEFAULT_BG_R 0x33
#define DEFAULT_BG_G 0x77
#define DEFAULT_BG_B 0xff

namespace myturtlesim
{

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f)
  : QFrame(parent, f)
  , path_image_(500, 500, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
  , frame_count_(0)
  , id_counter_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("MyTurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;
  rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
  background_r_descriptor.description = "Red channel of the background color";
  background_r_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
  background_g_descriptor.description = "Green channel of the background color";
  background_g_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
  background_b_descriptor.description = "Blue channel of the background color";
  background_b_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
  nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
  nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

  QVector<QString> turtles;
  turtles.append("ardent.png");
  turtles.append("bouncy.png");
  turtles.append("crystal.png");
  turtles.append("dashing.png");
  turtles.append("eloquent.png");
  turtles.append("foxy.png");
  turtles.append("galactic.png");
  turtles.append("humble.png");
  turtles.append("rolling.png");

  QString images_path = (ament_index_cpp::get_package_share_directory("myturtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear();

  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>(
      "clear", std::bind(&TurtleFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>(
      "reset", std::bind(&TurtleFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  spawn_srv_ = nh_->create_service<myturtlesim::srv::Spawn>(
      "spawn", std::bind(&TurtleFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  kill_srv_ = nh_->create_service<myturtlesim::srv::Kill>(
      "kill", std::bind(&TurtleFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", qos, std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(), "Starting myturtlesim with node name %s", nh_->get_fully_qualified_name());

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

  // spawn all available turtle types
  if (false)
  {
    for (int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * ((float)index / 7),
                  static_cast<float>(PI) / 2.0f, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(const myturtlesim::srv::Spawn::Request::SharedPtr req,
                                myturtlesim::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool TurtleFrame::killCallback(const myturtlesim::srv::Kill::Request::SharedPtr req,
                               myturtlesim::srv::Kill::Response::SharedPtr)
{
  M_Turtle::iterator it = turtles_.find(req->name);
  if (it == turtles_.end())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

void TurtleFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name())
  {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t = std::make_shared<Turtle>(nh_, real_name, turtle_images_[static_cast<int>(index)],
                                         QPointF(x, height_in_meters_ - y), angle);
  turtles_[real_name] = t;
  update();

  RCLCPP_INFO(nh_->get_logger(), "Spawning turtle [%s] at x=%f y=%f theta=%f", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok())
  {
    close();
    return;
  }

  rclcpp::spin_some(nh_);

  updateTurtles();
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0)
  {
    last_turtle_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
                                   height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}

bool TurtleFrame::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                                std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing myturtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                                std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting myturtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}  // namespace myturtlesim
