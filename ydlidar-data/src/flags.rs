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
