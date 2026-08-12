#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ranger_msgs__msg__ActuatorState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motor: super::msg::MotorState,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver: super::msg::DriverState,

}



impl Default for ActuatorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ActuatorState::default())
  }
}

impl rosidl_runtime_rs::Message for ActuatorState {
  type RmwMsg = super::msg::rmw::ActuatorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        motor: super::msg::MotorState::into_rmw_message(std::borrow::Cow::Owned(msg.motor)).into_owned(),
        driver: super::msg::DriverState::into_rmw_message(std::borrow::Cow::Owned(msg.driver)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        motor: super::msg::MotorState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.motor)).into_owned(),
        driver: super::msg::DriverState::into_rmw_message(std::borrow::Cow::Borrowed(&msg.driver)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      motor: super::msg::MotorState::from_rmw_message(msg.motor),
      driver: super::msg::DriverState::from_rmw_message(msg.driver),
    }
  }
}


// Corresponds to ranger_msgs__msg__ActuatorStateArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorStateArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub states: Vec<super::msg::ActuatorState>,

}



impl Default for ActuatorStateArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ActuatorStateArray::default())
  }
}

impl rosidl_runtime_rs::Message for ActuatorStateArray {
  type RmwMsg = super::msg::rmw::ActuatorStateArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        states: msg.states
          .into_iter()
          .map(|elem| super::msg::ActuatorState::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        states: msg.states
          .iter()
          .map(|elem| super::msg::ActuatorState::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      states: msg.states
          .into_iter()
          .map(super::msg::ActuatorState::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ranger_msgs__msg__DriverState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DriverState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_voltage: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_temperature: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motor_temperature: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_state: u32,

}



impl Default for DriverState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DriverState::default())
  }
}

impl rosidl_runtime_rs::Message for DriverState {
  type RmwMsg = super::msg::rmw::DriverState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        driver_voltage: msg.driver_voltage,
        driver_temperature: msg.driver_temperature,
        motor_temperature: msg.motor_temperature,
        driver_state: msg.driver_state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      driver_voltage: msg.driver_voltage,
      driver_temperature: msg.driver_temperature,
      motor_temperature: msg.motor_temperature,
      driver_state: msg.driver_state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      driver_voltage: msg.driver_voltage,
      driver_temperature: msg.driver_temperature,
      motor_temperature: msg.motor_temperature,
      driver_state: msg.driver_state,
    }
  }
}


// Corresponds to ranger_msgs__msg__MotionState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// motion state
    pub motion_mode: u8,

}

impl MotionState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_DUAL_ACKERMAN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_PARALLEL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_SPINNING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_SIDE_SLIP: u8 = 3;

}


impl Default for MotionState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotionState::default())
  }
}

impl rosidl_runtime_rs::Message for MotionState {
  type RmwMsg = super::msg::rmw::MotionState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        motion_mode: msg.motion_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      motion_mode: msg.motion_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      motion_mode: msg.motion_mode,
    }
  }
}


// Corresponds to ranger_msgs__msg__MotorState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub rpm: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pulse_count: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motor_angles: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motor_speeds: f32,

}



impl Default for MotorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorState::default())
  }
}

impl rosidl_runtime_rs::Message for MotorState {
  type RmwMsg = super::msg::rmw::MotorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        rpm: msg.rpm,
        current: msg.current,
        pulse_count: msg.pulse_count,
        motor_angles: msg.motor_angles,
        motor_speeds: msg.motor_speeds,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      rpm: msg.rpm,
      current: msg.current,
      pulse_count: msg.pulse_count,
      motor_angles: msg.motor_angles,
      motor_speeds: msg.motor_speeds,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      rpm: msg.rpm,
      current: msg.current,
      pulse_count: msg.pulse_count,
      motor_angles: msg.motor_angles,
      motor_speeds: msg.motor_speeds,
    }
  }
}


// Corresponds to ranger_msgs__msg__SystemState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SystemState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// system state
    pub vehicle_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub battery_voltage: f64,

    /// motion state
    pub motion_mode: u8,

}

impl SystemState {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VEHICLE_STATE_NORMAL: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VEHICLE_STATE_ESTOP: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VEHICLE_STATE_EXCEPTION: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CONTROL_MODE_RC: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CONTROL_MODE_CAN: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_DUAL_ACKERMAN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_PARALLEL: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_SPINNING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_MODE_SIDE_SLIP: u8 = 3;

}


impl Default for SystemState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SystemState::default())
  }
}

impl rosidl_runtime_rs::Message for SystemState {
  type RmwMsg = super::msg::rmw::SystemState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        vehicle_state: msg.vehicle_state,
        control_mode: msg.control_mode,
        error_code: msg.error_code,
        battery_voltage: msg.battery_voltage,
        motion_mode: msg.motion_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      vehicle_state: msg.vehicle_state,
      control_mode: msg.control_mode,
      error_code: msg.error_code,
      battery_voltage: msg.battery_voltage,
      motion_mode: msg.motion_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      vehicle_state: msg.vehicle_state,
      control_mode: msg.control_mode,
      error_code: msg.error_code,
      battery_voltage: msg.battery_voltage,
      motion_mode: msg.motion_mode,
    }
  }
}


