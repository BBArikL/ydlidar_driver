/// Interference flag corresponding to the scan signal.
#[derive(Clone, Debug, PartialEq)]
pub enum InterferenceFlag {
    /// The signal has the interference of specular reflection
    SpecularReflection,
    /// The signal is interfered by ambient light
    AmbientLight,
    /// Interference was not observed
    Nothing,
}

pub(crate) fn to_flag(value: u8) -> InterferenceFlag {
    match value {
        2 => InterferenceFlag::SpecularReflection,
        3 => InterferenceFlag::AmbientLight,
        _ => InterferenceFlag::Nothing,
    }
}
