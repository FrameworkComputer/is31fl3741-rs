#![cfg(any(feature = "framework_ledmatrix", feature = "adafruit_rgb_13x9"))]

#[cfg(feature = "adafruit_rgb_13x9")]
mod support;

fn assert_mapping(fixture: &str, mapping: fn(u8, u8) -> (u8, u8), channels: usize) {
    let mut seen = std::collections::HashSet::new();
    let mut count = 0;
    for line in fixture.lines().filter(|line| !line.starts_with('#')) {
        let values: Vec<u8> = line.split(',').map(|v| v.parse().unwrap()).collect();
        let actual = mapping(values[0], values[1]);
        assert_eq!(
            actual,
            (values[2], values[3]),
            "coordinate {},{}",
            values[0],
            values[1]
        );
        assert!(actual.1 <= 1);
        assert!(actual.0 < if actual.1 == 0 { 180 } else { 171 });
        assert!(seen.insert(actual), "duplicate hardware channel");
        count += 1;
    }
    assert_eq!(count, channels);
}

#[cfg(feature = "framework_ledmatrix")]
#[test]
fn framework_mapping_matches_frozen_reference() {
    assert_mapping(
        include_str!("data/framework-mapping.csv"),
        is31fl3741::devices::CALC_PIXEL,
        306,
    );
}

#[cfg(feature = "adafruit_rgb_13x9")]
#[test]
fn adafruit_mapping_matches_frozen_reference() {
    let d = is31fl3741::devices::AdafruitRGB13x9::configure(support::MockI2c::default());
    assert_mapping(
        include_str!("data/adafruit-mapping.csv"),
        d.device.calc_pixel,
        351,
    );
}
