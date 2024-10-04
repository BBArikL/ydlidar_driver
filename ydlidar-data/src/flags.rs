#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Interference flag corresponding to the scan signal.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum InterferenceFlag {
    /// The signal has the interference of specular reflection
    SpecularReflection,
    /// The signal is interfered by ambient light
    AmbientLight,
    /// Interference was not observed
    Nothing,
}
