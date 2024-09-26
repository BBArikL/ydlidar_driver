use ydlidar_data::InterferenceFlag;

pub(crate) fn to_flag(value: u8) -> InterferenceFlag {
    match value {
        2 => InterferenceFlag::SpecularReflection,
        3 => InterferenceFlag::AmbientLight,
        _ => InterferenceFlag::Nothing,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_flag() {
        assert_eq!(to_flag(2), InterferenceFlag::SpecularReflection);
        assert_eq!(to_flag(3), InterferenceFlag::AmbientLight);
        assert_eq!(to_flag(1), InterferenceFlag::Nothing);
    }
}
