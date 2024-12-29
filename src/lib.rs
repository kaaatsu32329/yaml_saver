pub fn laser_scan_to_yaml(scan: r2r::sensor_msgs::msg::LaserScan) -> String {
    let mut yaml = String::new();
    yaml.push_str("---\n");
    yaml.push_str("header:\n");
    yaml.push_str(&format!("  stamp:\n"));
    yaml.push_str(&format!("    sec: {}\n", scan.header.stamp.sec));
    yaml.push_str(&format!("    nanosec: {}\n", scan.header.stamp.nanosec));
    yaml.push_str(&format!("  frame_id: {}\n", scan.header.frame_id));
    yaml.push_str(&format!("angle_min: {}\n", scan.angle_min));
    yaml.push_str(&format!("angle_max: {}\n", scan.angle_max));
    yaml.push_str(&format!("angle_increment: {}\n", scan.angle_increment));
    yaml.push_str(&format!("time_increment: {}\n", scan.time_increment));
    yaml.push_str(&format!("scan_time: {}\n", scan.scan_time));
    yaml.push_str(&format!("range_min: {}\n", scan.range_min));
    yaml.push_str(&format!("range_max: {}\n", scan.range_max));
    yaml.push_str(&format!("ranges:\n"));
    for r in scan.ranges.iter() {
        yaml.push_str(&format!("  - {}\n", r));
    }
    yaml.push_str(&format!("intensities:\n"));
    for i in scan.intensities.iter() {
        yaml.push_str(&format!("  - {}\n", i));
    }
    yaml
}

pub fn odometry_to_yaml(odom: r2r::nav_msgs::msg::Odometry) -> String {
    let mut yaml = String::new();
    yaml.push_str("---\n");
    yaml.push_str("header:\n");
    yaml.push_str(&format!("  stamp:\n"));
    yaml.push_str(&format!("    sec: {}\n", odom.header.stamp.sec));
    yaml.push_str(&format!("    nanosec: {}\n", odom.header.stamp.nanosec));
    yaml.push_str(&format!("  frame_id: {}\n", odom.header.frame_id));
    yaml.push_str(&format!("child_frame_id: {}\n", odom.child_frame_id));
    yaml.push_str(&format!("pose:\n"));
    yaml.push_str(&format!("  pose:\n"));
    yaml.push_str(&format!("    position:\n"));
    yaml.push_str(&format!("      x: {}\n", odom.pose.pose.position.x));
    yaml.push_str(&format!("      y: {}\n", odom.pose.pose.position.y));
    yaml.push_str(&format!("      z: {}\n", odom.pose.pose.position.z));
    yaml.push_str(&format!("    orientation:\n"));
    yaml.push_str(&format!("      x: {}\n", odom.pose.pose.orientation.x));
    yaml.push_str(&format!("      y: {}\n", odom.pose.pose.orientation.y));
    yaml.push_str(&format!("      z: {}\n", odom.pose.pose.orientation.z));
    yaml.push_str(&format!("      w: {}\n", odom.pose.pose.orientation.w));
    yaml.push_str(&format!("  covariance:\n"));
    for c in odom.pose.covariance.iter() {
        yaml.push_str(&format!("    - {}\n", c));
    }
    yaml.push_str(&format!("twist:\n"));
    yaml.push_str(&format!("  twist:\n"));
    yaml.push_str(&format!("    linear:\n"));
    yaml.push_str(&format!("      x: {}\n", odom.twist.twist.linear.x));
    yaml.push_str(&format!("      y: {}\n", odom.twist.twist.linear.y));
    yaml.push_str(&format!("      z: {}\n", odom.twist.twist.linear.z));
    yaml.push_str(&format!("    angular:\n"));
    yaml.push_str(&format!("      x: {}\n", odom.twist.twist.angular.x));
    yaml.push_str(&format!("      y: {}\n", odom.twist.twist.angular.y));
    yaml.push_str(&format!("      z: {}\n", odom.twist.twist.angular.z));
    yaml.push_str(&format!("  covariance:\n"));
    for c in odom.twist.covariance.iter() {
        yaml.push_str(&format!("    - {}\n", c));
    }
    yaml
}
