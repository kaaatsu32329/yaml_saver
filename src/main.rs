use futures::{executor::LocalPool, stream::StreamExt, task::LocalSpawnExt};
use r2r::QosProfile;
use std::io::Write;
use yaml_saver::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "yaml_saver", "")?;

    let mut scan_subscriber =
        node.subscribe::<r2r::sensor_msgs::msg::LaserScan>("/scan", QosProfile::default())?;
    let mut odom_subscriber =
        node.subscribe::<r2r::nav_msgs::msg::Odometry>("/odom", QosProfile::default())?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let mut scan_writer = std::io::BufWriter::new(std::fs::File::create("log/scan.yaml").unwrap());
    let mut odom_writer = std::io::BufWriter::new(std::fs::File::create("log/odom.yaml").unwrap());

    spawner.spawn_local(async move {
        loop {
            match scan_subscriber.next().await {
                Some(scan_msg) => {
                    let scan_str = laser_scan_to_yaml(scan_msg);
                    scan_writer.write_all(scan_str.as_bytes()).unwrap();
                }
                None => {
                    print!(".");
                }
            }
        }
    })?;

    spawner.spawn_local(async move {
        loop {
            match odom_subscriber.next().await {
                Some(odom_msg) => {
                    let odom_str = odometry_to_yaml(odom_msg);
                    odom_writer.write_all(odom_str.as_bytes()).unwrap();
                }
                None => {
                    print!(".");
                }
            }
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
