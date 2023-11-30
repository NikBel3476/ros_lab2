use once_cell::sync::Lazy;
use rosrust::ros_info;
use rosrust_msg::geometry_msgs;
use rosrust_msg::{geometry_msgs::PoseWithCovariance, nav_msgs::Odometry};
use std::env;
use std::f64::consts::PI;
use std::sync::Mutex;

// static mut current_pose: Box<PoseWithCovariance> = Box::new(PoseWithCovariance::default());
static CURRENT_POSE: Lazy<Mutex<Option<PoseWithCovariance>>> = Lazy::new(|| Mutex::new(None));

fn main() {
    rosrust::init("lab2");

    // ros_info!("{:#?}", env::args());
    let x_arg = env::args().nth(1);
    let y_arg = env::args().nth(2);
    let (x, y): (f64, f64) = match (x_arg, y_arg) {
        (Some(x_str), Some(y_str)) => (
            x_str.parse().expect("Incorrect x coordinate"),
            y_str.parse().expect("Incorrect y coordinate"),
        ),
        _ => panic!("Expected x and y coordinates. Using: `cargo run <x> <y>`"),
    };

    ros_info!("x: {x}, y: {y}");

    let cmd_vel_pub = rosrust::publish("cmd_vel", 10).unwrap();
    cmd_vel_pub.wait_for_subscribers(None).unwrap();

    let pose_sub = rosrust::subscribe("odom", 10, |odometry: Odometry| {
        let mut pose = CURRENT_POSE.lock().unwrap();
        *pose = Some(odometry.pose);
    })
    .unwrap();

    loop {
        if CURRENT_POSE.lock().unwrap().is_some() {
            break;
        }
    }

    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    if let Some(pose) = pose {
        let robot_angle = get_angle_from_pose(&pose);
        ros_info!("Current angle: {robot_angle}");

        let angle_to_destination_point = (y - pose.pose.position.y).atan2(x - pose.pose.position.x);
        ros_info!("Angle to destination point: {angle_to_destination_point}");

        let angle_to_rotate = angle_to_destination_point - robot_angle;
        ros_info!("Angle to rotate: {angle_to_rotate}");

        let direction = angle_to_rotate.signum();
        ros_info!("Direction: {direction}");
        cmd_vel_pub
            .send(geometry_msgs::Twist {
                linear: Default::default(),
                angular: geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.2 * direction,
                },
            })
            .unwrap();

        let rate = rosrust::rate(10.0);
        loop {
            if let Some(robot_angle) = get_robot_angle() {
                // ros_info!("Robot angle: {robot_angle}");
                if (angle_to_destination_point - robot_angle).abs() < 0.01 {
                    cmd_vel_pub.send(Default::default()).unwrap();
                    ros_info!("Rotation completed. Current angle: {robot_angle}");
                    break;
                }
            }
            rate.sleep();
        }
    }

    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    if let Some(pose) = pose {
        let distance =
            ((x - pose.pose.position.x).powi(2) + (y - pose.pose.position.y).powi(2)).sqrt();
        let velocity = 0.2;
        let duration = rosrust::Duration::from_nanos((distance / velocity * 1e9) as i64);

        ros_info!("Distance: {distance}");
        ros_info!("Velocity: {velocity}");
        ros_info!("Duration: {duration}");

        cmd_vel_pub
            .send(geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 {
                    x: 0.2,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Default::default(),
            })
            .unwrap();
        rosrust::sleep(duration);
        cmd_vel_pub.send(Default::default()).unwrap();

        ros_info!("Destination point is reached");
    }

    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    if let Some(pose) = pose {
        ros_info!("Current robot position: {:#?}", pose.pose.position);
    }
}

fn get_robot_angle() -> Option<f64> {
    let pose = CURRENT_POSE.lock().unwrap();
    match (*pose).clone() {
        Some(pose) => Some(get_angle_from_pose(&pose)),
        _ => None,
    }
}

fn get_angle_from_pose(pose: &PoseWithCovariance) -> f64 {
    (pose.pose.orientation.z.asin() * 2.0 * 180.0 / PI).to_radians()
    // (pose.pose.orientation.z).atan2(pose.pose.orientation.w)
}
