///|
/// A length stored internally in meters (the URDF/SI unit).
pub struct Length {
  meters : Double
}

///|
pub fn Length::m(value : Double) -> Length {
  { meters: value, }
}

///|
pub fn Length::cm(value : Double) -> Length {
  { meters: value * 0.01, }
}

///|
pub fn Length::mm(value : Double) -> Length {
  { meters: value * 0.001, }
}

///|
pub fn Length::inches(value : Double) -> Length {
  { meters: value * 0.0254, }
}

///|
pub fn Length::to_m(self : Length) -> Double {
  self.meters
}

///|
pub fn Length::to_mm(self : Length) -> Double {
  self.meters * 1000.0
}

///|
pub fn Length::scale(self : Length, factor : Double) -> Length {
  { meters: self.meters * factor, }
}

///|
pub impl Add for Length with fn add(self : Length, other : Length) -> Length {
  { meters: self.meters + other.meters, }
}

///|
pub impl Sub for Length with fn sub(self : Length, other : Length) -> Length {
  { meters: self.meters - other.meters, }
}

///|
pub impl Neg for Length with fn neg(self : Length) -> Length {
  { meters: -self.meters, }
}

///|
/// An angle stored internally in radians (the URDF/SI unit).
pub struct Angle {
  radians : Double
}

///|
pub fn Angle::rad(value : Double) -> Angle {
  { radians: value, }
}

///|
pub fn Angle::deg(value : Double) -> Angle {
  { radians: value * @math.PI / 180.0, }
}

///|
pub fn Angle::to_rad(self : Angle) -> Double {
  self.radians
}

///|
pub fn Angle::to_deg(self : Angle) -> Double {
  self.radians * 180.0 / @math.PI
}

///|
pub impl Add for Angle with fn add(self : Angle, other : Angle) -> Angle {
  { radians: self.radians + other.radians, }
}

///|
pub impl Sub for Angle with fn sub(self : Angle, other : Angle) -> Angle {
  { radians: self.radians - other.radians, }
}

///|
pub impl Neg for Angle with fn neg(self : Angle) -> Angle {
  { radians: -self.radians, }
}

///|
/// A mass stored internally in kilograms (the URDF/SI unit).
pub struct Mass {
  kilograms : Double
}

///|
pub fn Mass::kg(value : Double) -> Mass {
  { kilograms: value, }
}

///|
pub fn Mass::grams(value : Double) -> Mass {
  { kilograms: value * 0.001, }
}

///|
pub fn Mass::lbs(value : Double) -> Mass {
  { kilograms: value * 0.45359237, }
}

///|
pub fn Mass::to_kg(self : Mass) -> Double {
  self.kilograms
}

///|
pub fn Mass::to_grams(self : Mass) -> Double {
  self.kilograms * 1000.0
}

///|
pub impl Add for Mass with fn add(self : Mass, other : Mass) -> Mass {
  { kilograms: self.kilograms + other.kilograms, }
}

///|
pub impl Sub for Mass with fn sub(self : Mass, other : Mass) -> Mass {
  { kilograms: self.kilograms - other.kilograms, }
}

///|
/// A material density stored internally in kg/m³ (the URDF/SI unit).
pub struct Density {
  kg_per_m3 : Double
}

///|
pub fn Density::kg_per_m3(value : Double) -> Density {
  { kg_per_m3: value, }
}

///|
pub fn Density::g_per_cm3(value : Double) -> Density {
  { kg_per_m3: value * 1000.0, }
}

///|
pub fn Density::to_kg_per_m3(self : Density) -> Double {
  self.kg_per_m3
}

///|
/// Common FDM 3D-printing filament: PLA.
pub fn Density::pla() -> Density {
  { kg_per_m3: 1240.0, }
}

///|
/// Common FDM 3D-printing filament: PETG.
pub fn Density::petg() -> Density {
  { kg_per_m3: 1270.0, }
}

///|
/// Common FDM 3D-printing filament: ABS.
pub fn Density::abs() -> Density {
  { kg_per_m3: 1040.0, }
}

///|
/// Common FDM 3D-printing filament: TPU.
pub fn Density::tpu() -> Density {
  { kg_per_m3: 1210.0, }
}

///|
/// Carbon-fiber composite (approximate).
pub fn Density::carbon_fiber() -> Density {
  { kg_per_m3: 1600.0, }
}

///|
/// Aluminum 6061 alloy.
pub fn Density::aluminum_6061() -> Density {
  { kg_per_m3: 2700.0, }
}

///|
/// Mild steel.
pub fn Density::steel_mild() -> Density {
  { kg_per_m3: 7850.0, }
}

///|
/// Titanium alloy (approximate).
pub fn Density::titanium() -> Density {
  { kg_per_m3: 4430.0, }
}

///|
/// Short alias for mild steel (`steel_mild`), the material robotics builders
/// reach for most often.
pub fn Density::steel() -> Density {
  Density::steel_mild()
}

///|
/// Carbon-fiber reinforced nylon, typical of printed structural links.
pub fn Density::cf_nylon() -> Density {
  { kg_per_m3: 1250.0, }
}

///|
/// Rubber, typical of tires, feet, and vibration isolators.
pub fn Density::rubber() -> Density {
  { kg_per_m3: 1150.0, }
}

///|
/// Lithium-polymer battery pack, including cells, packaging, and wiring.
pub fn Density::lipo_battery() -> Density {
  { kg_per_m3: 1850.0, }
}

///|
/// FR-4 printed-circuit board (approximate, copper included).
pub fn Density::fr4_pcb() -> Density {
  { kg_per_m3: 1850.0, }
}

///|
/// Brass, typical of standoffs and bushings.
pub fn Density::brass() -> Density {
  { kg_per_m3: 8500.0, }
}

///|
/// Acetal (Delrin/POM), typical of low-friction bushings and gears.
pub fn Density::delrin() -> Density {
  { kg_per_m3: 1410.0, }
}
