use geometry_msgs::msg::PoseStamped;
use mavros_msgs::{
    msg::State,
    srv::{
        CommandBool, CommandBool_Request, CommandBool_Response, SetMode, SetMode_Request,
        SetMode_Response,
    },
};
use rclrs::*;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::sleep,
    time::{Duration, SystemTime},
};

#[derive(Debug, Default)]
struct SubscriptionData {
    current_state: State,
}

fn main() -> anyhow::Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("offb_node")?;
    let worker = node.create_worker(SubscriptionData::default());

    let _state_sub = worker.create_subscription(
        "mavros/state",
        move |sub_data: &mut SubscriptionData, msg: State| {
            sub_data.current_state = msg;
        },
    )?;

    let local_pos_pub = node.create_publisher("mavros/setpoint_position/local")?;
    let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming")?;
    let set_mode_client = node.create_client::<SetMode>("mavros/set_mode")?;

    let rate_time = Duration::from_millis(50);
    let connected = Arc::new(AtomicBool::new(false));
    let connected_clone = connected.clone();

    std::thread::spawn(move || {
        executor.spin(SpinOptions::default());
    });

    while !connected.clone().load(Ordering::Relaxed) {
        let _ = worker.run({
            let connected_clone = connected_clone.clone();
            move |sub_data: &mut SubscriptionData| {
                if sub_data.current_state.connected {
                    connected_clone.store(true, Ordering::Relaxed);
                }
            }
        });
        sleep(rate_time);
    }

    let mut pose = PoseStamped::default();
    pose.pose.position.z = 2.0;

    for _i in 0..100 {
        local_pos_pub.publish(&pose)?;
        sleep(rate_time);
    }

    let last_request = Arc::new(Mutex::new(SystemTime::now()));

    loop {
        let node = node.clone();
        let set_mode_client = set_mode_client.clone();
        let arming_client = arming_client.clone();
        let last_request = last_request.clone();
        let _ = worker.run(move |sub_data: &mut SubscriptionData| {
            let current_state = &sub_data.current_state;
            if current_state.mode != "OFFBOARD"
                && SystemTime::now() > *last_request.lock().unwrap() + Duration::from_secs(5)
            {
                let mut offb_set_mode = SetMode_Request::default();
                offb_set_mode.custom_mode = "OFFBOARD".to_string();
                let _ = set_mode_client.call_then(&offb_set_mode, move |res: SetMode_Response| {
                    if res.mode_sent {
                        log_info!(node.logger().once(), "Offboard enabled");
                    }
                });
                *last_request.lock().unwrap() = SystemTime::now();
            } else if !current_state.armed
                && SystemTime::now() > *last_request.lock().unwrap() + Duration::from_secs(5)
            {
                let mut arm_cmd = CommandBool_Request::default();
                arm_cmd.value = true;
                let _ = arming_client.call_then(&arm_cmd, move |res: CommandBool_Response| {
                    if res.success {
                        log_info!(node.logger().once(), "Vehicle armed");
                    }
                });
                *last_request.lock().unwrap() = SystemTime::now();
            }
        });
        local_pos_pub.publish(&pose).unwrap();
        sleep(rate_time);
    }
}
