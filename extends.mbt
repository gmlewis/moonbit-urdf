///|
// Explicit trait-method promotion declarations.
//
// MoonBit no longer promotes a trait's methods to direct methods on a type
// implicitly: every promoted method needs an explicit `extend` declaration,
// otherwise moonc warns (implicit_impl_as_method). The primary method of each
// trait is promoted plainly. The defaulted derivative that the `!=` operator
// already covers (`Eq::not_equal`) is promoted as well - moonc otherwise warns
// about its implicit promotion - but deprecated and hidden, since there is no
// reason to call it directly.

///|
pub extend Actuator with Debug::{to_repr}

///|
pub extend Actuator with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Actuator with Eq::{not_equal}

///|
pub extend Collision with Debug::{to_repr}

///|
pub extend Collision with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Collision with Eq::{not_equal}

///|
pub extend Geometry with Debug::{to_repr}

///|
pub extend Geometry with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Geometry with Eq::{not_equal}

///|
pub extend Inertia with Debug::{to_repr}

///|
pub extend Inertia with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Inertia with Eq::{not_equal}

///|
pub extend Inertial with Debug::{to_repr}

///|
pub extend Inertial with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Inertial with Eq::{not_equal}

///|
pub extend Joint with Debug::{to_repr}

///|
pub extend Joint with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Joint with Eq::{not_equal}

///|
pub extend JointCalibration with Debug::{to_repr}

///|
pub extend JointCalibration with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointCalibration with Eq::{not_equal}

///|
pub extend JointDynamics with Debug::{to_repr}

///|
pub extend JointDynamics with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointDynamics with Eq::{not_equal}

///|
pub extend JointLimit with Debug::{to_repr}

///|
pub extend JointLimit with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointLimit with Eq::{not_equal}

///|
pub extend JointMimic with Debug::{to_repr}

///|
pub extend JointMimic with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointMimic with Eq::{not_equal}

///|
pub extend JointType with Debug::{to_repr}

///|
pub extend JointType with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointType with Eq::{not_equal}

///|
pub extend Link with Debug::{to_repr}

///|
pub extend Link with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Link with Eq::{not_equal}

///|
pub extend Material with Debug::{to_repr}

///|
pub extend Material with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Material with Eq::{not_equal}

///|
pub extend Origin with Debug::{to_repr}

///|
pub extend Origin with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Origin with Eq::{not_equal}

///|
pub extend RawXmlNode with Debug::{to_repr}

///|
pub extend RawXmlNode with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend RawXmlNode with Eq::{not_equal}

///|
pub extend Rgba with Debug::{to_repr}

///|
pub extend Rgba with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Rgba with Eq::{not_equal}

///|
pub extend Robot with Debug::{to_repr}

///|
pub extend Robot with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Robot with Eq::{not_equal}

///|
pub extend SafetyController with Debug::{to_repr}

///|
pub extend SafetyController with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend SafetyController with Eq::{not_equal}

///|
pub extend Transmission with Debug::{to_repr}

///|
pub extend Transmission with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Transmission with Eq::{not_equal}

///|
pub extend TransmissionJoint with Debug::{to_repr}

///|
pub extend TransmissionJoint with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend TransmissionJoint with Eq::{not_equal}

///|
pub extend Visual with Debug::{to_repr}

///|
pub extend Visual with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Visual with Eq::{not_equal}

///|
pub extend Angle with Add::{add}

///|
pub extend Angle with Sub::{sub}

///|
pub extend Angle with Neg::{neg}

///|
pub extend Length with Add::{add}

///|
pub extend Length with Sub::{sub}

///|
pub extend Length with Neg::{neg}

///|
pub extend Mass with Add::{add}

///|
pub extend Mass with Sub::{sub}

///|
pub extend UrdfParseErrorInfo with Debug::{to_repr}

///|
pub extend UrdfParseErrorInfo with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend UrdfParseErrorInfo with Eq::{not_equal}

///|
pub extend Diagnostic with Debug::{to_repr}

///|
pub extend Diagnostic with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Diagnostic with Eq::{not_equal}

///|
pub extend Severity with Debug::{to_repr}

///|
pub extend Severity with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Severity with Eq::{not_equal}

///|
pub extend MateConnector with Debug::{to_repr}

///|
pub extend MateConnector with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend MateConnector with Eq::{not_equal}

///|
///|
pub extend Matrix3x3 with Debug::{to_repr}

///|
pub extend Matrix3x3 with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Matrix3x3 with Eq::{not_equal}

///|
pub extend Transform with Debug::{to_repr}

///|
pub extend Transform with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Transform with Eq::{not_equal}
