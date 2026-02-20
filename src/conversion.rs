use std::f64::consts::PI;

/// Conversion helpers for 12-bit Feetech encoder space (4096 ticks / revolution)
/// with configurable logical zero offset.
#[derive(Debug, Clone, Copy)]
pub struct EncoderConversion {
    pub ticks_per_rev: i32,
    pub zero_tick: i32,
}

impl Default for EncoderConversion {
    fn default() -> Self {
        Self {
            ticks_per_rev: 4096,
            zero_tick: 0,
        }
    }
}

impl EncoderConversion {
    pub fn new(zero_tick: i32) -> Self {
        Self {
            ticks_per_rev: 4096,
            zero_tick,
        }
    }

    /// Convert raw encoder tick to radians around logical zero.
    /// Result is normalized to [-pi, pi).
    pub fn tick_to_rad(&self, raw_tick: i32) -> f64 {
        let rel = wrap_tick(raw_tick - self.zero_tick, self.ticks_per_rev);
        (rel as f64) * (2.0 * PI) / (self.ticks_per_rev as f64)
    }

    /// Convert radians to raw encoder tick around configured logical zero.
    pub fn rad_to_tick(&self, rad: f64) -> i32 {
        let mut rel = (rad * (self.ticks_per_rev as f64) / (2.0 * PI)).round() as i32;
        rel = wrap_tick(rel, self.ticks_per_rev);
        wrap_tick(self.zero_tick + rel, self.ticks_per_rev)
    }
}

fn wrap_tick(v: i32, n: i32) -> i32 {
    let m = v % n;
    if m < 0 { m + n } else { m }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_with_offset() {
        let c = EncoderConversion::new(1234);
        let raw = 3500;
        let rad = c.tick_to_rad(raw);
        let out = c.rad_to_tick(rad);
        assert_eq!(out, raw);
    }

    #[test]
    fn zero_maps_to_zero_rad() {
        let c = EncoderConversion::new(777);
        assert!((c.tick_to_rad(777) - 0.0).abs() < 1e-9);
    }
}
