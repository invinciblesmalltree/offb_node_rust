use geometry_msgs::msg::PoseStamped;
use mavros_msgs::{
    msg::State,
    srv::{CommandBool, CommandBool_Request, SetMode, SetMode_Request},
};
use rclrs::{create_node, spin_once, Context, QOS_PROFILE_DEFAULT};
use std::{
    env,
    mem::transmute,
    thread::sleep,
    time::{Duration, SystemTime},
};

fn main() -> anyhow::Result<()> {
    let context = Context::new(env::args())?;
    let mut node = create_node(&context, "offb_node")?;

    let mut current_state = State::default();
    let current_state_ref: &mut State = unsafe { transmute(&mut current_state) };

    let _state_sub =
        node.create_subscription("mavros/state", QOS_PROFILE_DEFAULT, |msg: State| {
            *current_state_ref = msg;
        })?;
    let local_pos_pub =
        node.create_publisher("mavros/setpoint_position/local", QOS_PROFILE_DEFAULT)?;
    let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming")?;
    let set_mode_client = node.create_client::<SetMode>("mavros/set_mode")?;

    let rate_time = Duration::from_millis(50);
    let spin_time = Some(Duration::from_secs(0));

    while context.ok() && !current_state.connected {
        spin_once(&node, spin_time).unwrap_or(());
        sleep(rate_time);
    }

    let mut pose = PoseStamped::default();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 2.0;

    for _i in 0..100 {
        local_pos_pub.publish(&pose)?;
        spin_once(&node, spin_time).unwrap_or(());
        sleep(rate_time);
    }

    let mut offb_set_mode = SetMode_Request::default();
    offb_set_mode.custom_mode = "OFFBOARD".to_string();

    let mut arm_cmd = CommandBool_Request::default();
    arm_cmd.value = true;

    let mut last_request = SystemTime::now();

    while context.ok() {
        if current_state.mode != "OFFBOARD"
            && SystemTime::now().duration_since(last_request)? > Duration::from_secs(5)
        {
            set_mode_client.async_send_request_with_callback(&offb_set_mode, |res| {
                if res.mode_sent {
                    println!("Offboard enabled");
                }
            })?;
            last_request = SystemTime::now();
        } else if !current_state.armed
            && SystemTime::now().duration_since(last_request)? > Duration::from_secs(5)
        {
            arming_client.async_send_request_with_callback(&arm_cmd, |res| {
                if res.success {
                    println!("Vehicle armed");
                }
            })?;
            last_request = SystemTime::now();
        }

        local_pos_pub.publish(&pose)?;

        spin_once(&node, spin_time).unwrap_or(());
        sleep(rate_time);
    }

    Ok(())
}
