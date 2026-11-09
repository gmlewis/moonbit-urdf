///|
fn approx(a : Double, b : Double) -> Bool {
  (a - b).abs() < 1.0e-9
}

///|
test "length converts between meters millimeters and inches" {
  assert_true(approx(@urdf.Length::mm(250.0).to_m(), 0.25))
  assert_true(approx(@urdf.Length::mm(250.0).to_mm(), 250.0))
  assert_true(approx(@urdf.Length::cm(12.5).to_m(), 0.125))
  assert_true(approx(@urdf.Length::inches(1.0).to_m(), 0.0254))
  assert_true(approx(@urdf.Length::inches(10.0).to_mm(), 254.0))
}

///|
test "length arithmetic" {
  let a = @urdf.Length::m(0.5)
  let b = @urdf.Length::cm(25.0)
  assert_true(approx((a + b).to_m(), 0.75))
  assert_true(approx((a - b).to_m(), 0.25))
  assert_true(approx((-a).to_m(), -0.5))
  assert_true(approx(a.scale(3.0).to_m(), 1.5))
}

///|
test "angle converts between radians and degrees" {
  assert_true(approx(@urdf.Angle::deg(180.0).to_rad(), @math.PI))
  assert_true(approx(@urdf.Angle::rad(@math.PI / 2.0).to_deg(), 90.0))
  assert_true(
    approx((@urdf.Angle::deg(30.0) + @urdf.Angle::deg(60.0)).to_deg(), 90.0),
  )
  assert_true(
    approx((@urdf.Angle::deg(90.0) - @urdf.Angle::deg(45.0)).to_deg(), 45.0),
  )
  assert_true(approx((-@urdf.Angle::deg(45.0)).to_deg(), -45.0))
}

///|
test "mass converts between kilograms grams and pounds" {
  assert_true(approx(@urdf.Mass::grams(500.0).to_kg(), 0.5))
  assert_true(approx(@urdf.Mass::lbs(1.0).to_kg(), 0.45359237))
  assert_true(approx(@urdf.Mass::kg(2.0).to_grams(), 2000.0))
  assert_true(
    approx((@urdf.Mass::kg(1.0) + @urdf.Mass::grams(250.0)).to_kg(), 1.25),
  )
  assert_true(
    approx((@urdf.Mass::kg(1.0) - @urdf.Mass::grams(250.0)).to_kg(), 0.75),
  )
}

///|
test "density conversions and material presets" {
  assert_true(approx(@urdf.Density::g_per_cm3(1.24).to_kg_per_m3(), 1240.0))
  assert_true(approx(@urdf.Density::pla().to_kg_per_m3(), 1240.0))
  assert_true(approx(@urdf.Density::petg().to_kg_per_m3(), 1270.0))
  assert_true(approx(@urdf.Density::abs().to_kg_per_m3(), 1040.0))
  assert_true(approx(@urdf.Density::tpu().to_kg_per_m3(), 1210.0))
  assert_true(approx(@urdf.Density::carbon_fiber().to_kg_per_m3(), 1600.0))
  assert_true(approx(@urdf.Density::aluminum_6061().to_kg_per_m3(), 2700.0))
  assert_true(approx(@urdf.Density::steel_mild().to_kg_per_m3(), 7850.0))
  assert_true(approx(@urdf.Density::titanium().to_kg_per_m3(), 4430.0))
}
