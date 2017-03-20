## Static transforms of flatfish ##

### Sensors

## imu pose in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.29424, 0.0, 0.02863 ),
    "imu" => "body"

## gps receiver pose in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "gps_receiver" => "body"

## pressure sensor in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.0, 0.0, -0.03 ),
    "pressure_sensor" => "body"

## dvl in body frame
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(22.5 / 180.0 * Math::PI, 0, 0), 2, 1, 0),
    Eigen::Vector3.new( 0.659, 0.141, -0.206 ),
    "dvl" => "body"

## usbl mobile in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.286, 0.126, 0.316 ),
    "usbl_mobile" => "body"

## camera front left in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 1.0, 0.13, -0.085 ),
    "camera_front_left" => "body"

## camera front right in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 1.0, -0.13, -0.085 ),
    "camera_front_right" => "body"

## camera bottom left in body frame
static_transform Eigen::Quaternion.from_angle_axis( 90.0 / 180.0 * Math::PI, Eigen::Vector3.UnitY ),
    Eigen::Vector3.new( 0.469, 0.13, -0.16 ),
    "camera_bottom_left" => "body"

## camera bottom right in body frame
static_transform Eigen::Quaternion.from_angle_axis( 90.0 / 180.0 * Math::PI, Eigen::Vector3.UnitY ),
    Eigen::Vector3.new( 0.469, -0.13, -0.16 ),
    "camera_bottom_right" => "body"

## front sonar pose in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.814, 0.0, 0.3 ),
    "sonar_tritech_front" => "body"

## rear sonar pose in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( -1.04, 0.145, -0.008 ),
    "sonar_tritech_rear" => "body"

## sonar gemini in body frame
static_transform Eigen::Quaternion.from_angle_axis( 30.0 / 180.0 * Math::PI, Eigen::Vector3.UnitY ),
    Eigen::Vector3.new( 0.0, 0.0, -0.206 ),
    "sonar_gemini" => "body"

## sonar blueview in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 1.1, 0.0, 0.035 ),
    "sonar_blueview" => "body"


### Map

## map_usbl in map
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "local_map" => "world"

static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 11.29, 0.22, -0.3 ),
    "map_usbl" => "dfki_basin"

static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 11.5, 9.5, 0.0 ),
    "map_localization" => "dfki_basin"

## map in world orientation at the dfki main tank
static_transform Eigen::Quaternion.from_angle_axis( -116.56 / 180.0 * Math::PI, Eigen::Vector3.UnitZ ),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "dfki_basin" => "world_orientation"

## angle of the reference wall in the world frame
## this is the angle of the northern wall in the DFKI testbed, based on google earth gps coordinates
static_transform Eigen::Quaternion.from_angle_axis( -26.56 / 180.0 * Math::PI, Eigen::Vector3.UnitZ ),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "reference_wall_dfki_basin" => "world_orientation"

static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "world_orientation" => "world"
