#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__ActuatorState() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__ActuatorState__init(msg: *mut ActuatorState) -> bool;
    fn ranger_msgs__msg__ActuatorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ActuatorState>, size: usize) -> bool;
    fn ranger_msgs__msg__ActuatorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ActuatorState>);
    fn ranger_msgs__msg__ActuatorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ActuatorState>, out_seq: *mut rosidl_runtime_rs::Sequence<ActuatorState>) -> bool;
}

// Corresponds to ranger_msgs__msg__ActuatorState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motor: super::super::msg::rmw::MotorState,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver: super::super::msg::rmw::DriverState,

}



impl Default for ActuatorState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__ActuatorState__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__ActuatorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ActuatorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ActuatorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ActuatorState where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/ActuatorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__ActuatorState() }
  }
}


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__ActuatorStateArray() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__ActuatorStateArray__init(msg: *mut ActuatorStateArray) -> bool;
    fn ranger_msgs__msg__ActuatorStateArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ActuatorStateArray>, size: usize) -> bool;
    fn ranger_msgs__msg__ActuatorStateArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ActuatorStateArray>);
    fn ranger_msgs__msg__ActuatorStateArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ActuatorStateArray>, out_seq: *mut rosidl_runtime_rs::Sequence<ActuatorStateArray>) -> bool;
}

// Corresponds to ranger_msgs__msg__ActuatorStateArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorStateArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub states: rosidl_runtime_rs::Sequence<super::super::msg::rmw::ActuatorState>,

}



impl Default for ActuatorStateArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__ActuatorStateArray__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__ActuatorStateArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ActuatorStateArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorStateArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorStateArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__ActuatorStateArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ActuatorStateArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ActuatorStateArray where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/ActuatorStateArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__ActuatorStateArray() }
  }
}


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__DriverState() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__DriverState__init(msg: *mut DriverState) -> bool;
    fn ranger_msgs__msg__DriverState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DriverState>, size: usize) -> bool;
    fn ranger_msgs__msg__DriverState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DriverState>);
    fn ranger_msgs__msg__DriverState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DriverState>, out_seq: *mut rosidl_runtime_rs::Sequence<DriverState>) -> bool;
}

// Corresponds to ranger_msgs__msg__DriverState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__DriverState__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__DriverState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DriverState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__DriverState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__DriverState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__DriverState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DriverState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DriverState where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/DriverState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__DriverState() }
  }
}


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__MotionState() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__MotionState__init(msg: *mut MotionState) -> bool;
    fn ranger_msgs__msg__MotionState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotionState>, size: usize) -> bool;
    fn ranger_msgs__msg__MotionState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotionState>);
    fn ranger_msgs__msg__MotionState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotionState>, out_seq: *mut rosidl_runtime_rs::Sequence<MotionState>) -> bool;
}

// Corresponds to ranger_msgs__msg__MotionState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotionState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__MotionState__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__MotionState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotionState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotionState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotionState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotionState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotionState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotionState where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/MotionState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__MotionState() }
  }
}


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__MotorState() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__MotorState__init(msg: *mut MotorState) -> bool;
    fn ranger_msgs__msg__MotorState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorState>, size: usize) -> bool;
    fn ranger_msgs__msg__MotorState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorState>);
    fn ranger_msgs__msg__MotorState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorState>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorState>) -> bool;
}

// Corresponds to ranger_msgs__msg__MotorState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__MotorState__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__MotorState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotorState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotorState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__MotorState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorState where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/MotorState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__MotorState() }
  }
}


#[link(name = "ranger_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__SystemState() -> *const std::ffi::c_void;
}

#[link(name = "ranger_msgs__rosidl_generator_c")]
extern "C" {
    fn ranger_msgs__msg__SystemState__init(msg: *mut SystemState) -> bool;
    fn ranger_msgs__msg__SystemState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SystemState>, size: usize) -> bool;
    fn ranger_msgs__msg__SystemState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SystemState>);
    fn ranger_msgs__msg__SystemState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SystemState>, out_seq: *mut rosidl_runtime_rs::Sequence<SystemState>) -> bool;
}

// Corresponds to ranger_msgs__msg__SystemState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SystemState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ranger_msgs__msg__SystemState__init(&mut msg as *mut _) {
        panic!("Call to ranger_msgs__msg__SystemState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SystemState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__SystemState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__SystemState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ranger_msgs__msg__SystemState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SystemState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SystemState where Self: Sized {
  const TYPE_NAME: &'static str = "ranger_msgs/msg/SystemState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ranger_msgs__msg__SystemState() }
  }
}


