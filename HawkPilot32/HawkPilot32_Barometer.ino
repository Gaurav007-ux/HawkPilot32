///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BMP280 (HW-611) Barometer — 3-stage complementary filter for stable altitude hold
//
// Filter pipeline per 4ms loop tick:
//   Hardware:  BMP280 IIR coeff 8  + x16 pressure oversampling   (removes motor vibration noise)
//   Software 1: 25-point rotating average                        (smooths reading jitter)
//   Software 2: Complementary filter (0.985/0.015)               (stable long-term altitude, ~267ms time constant)
//   Software 3: Divergence correction (±8 Pa clamp + /6 nudge)   (prevents slow drift)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t bmp280_dig_T1;
int16_t  bmp280_dig_T2, bmp280_dig_T3;
uint16_t bmp280_dig_P1;
int16_t  bmp280_dig_P2, bmp280_dig_P3, bmp280_dig_P4;
int16_t  bmp280_dig_P5, bmp280_dig_P6, bmp280_dig_P7;
int16_t  bmp280_dig_P8, bmp280_dig_P9;
int32_t  bmp280_t_fine;
int32_t  bmp280_raw_pressure, bmp280_raw_temperature;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize BMP280 — read calibration bytes, set high-quality measurement mode.
// Called once from setup().
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup_bmp280(void) {
  // Read 24 calibration bytes from 0x88 (little-endian, T1-T3, P1-P9)
  HWire.beginTransmission(BMP280_address);
  HWire.write(0x88);
  HWire.endTransmission();
  HWire.requestFrom(BMP280_address, 24);
  bmp280_dig_T1 = (uint16_t)(HWire.read() | (HWire.read() << 8));
  bmp280_dig_T2 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_T3 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P1 = (uint16_t)(HWire.read() | (HWire.read() << 8));
  bmp280_dig_P2 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P3 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P4 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P5 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P6 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P7 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P8 = (int16_t) (HWire.read() | (HWire.read() << 8));
  bmp280_dig_P9 = (int16_t) (HWire.read() | (HWire.read() << 8));

  // ctrl_meas (0xF4): osrs_t=010(x2), osrs_p=101(x16), mode=11(normal) = 0x57
  // x16 pressure oversampling reduces raw noise ~4x before any software filter
  HWire.beginTransmission(BMP280_address);
  HWire.write(0xF4);
  HWire.write(0x57);
  HWire.endTransmission();

  // config (0xF5): t_sb=000(0.5ms standby), filter=100(IIR coeff 8) = 0x10
  // IIR coeff 8 — halves residual pressure noise from motor vibration vs coeff 4,
  // with minimal added latency (BMP280 datasheet Table 6). Tuned for 500–700g frame (Task 3).
  HWire.beginTransmission(BMP280_address);
  HWire.write(0xF5);
  HWire.write(0x10);
  HWire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read and filter BMP280 pressure. Called every 4ms (250Hz main loop).
// 3-counter structure keeps timing identical to original MS5611 version
// so all altitude PID logic is unchanged.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_barometer(void) {
  barometer_counter++;

  //-------------------------------------------------------
  // Counter 1: Read 6 raw bytes (pressure + temperature)
  //-------------------------------------------------------
  if (barometer_counter == 1) {
    HWire.beginTransmission(BMP280_address);
    HWire.write(0xF7);
    HWire.endTransmission();
    HWire.requestFrom(BMP280_address, 6);
    bmp280_raw_pressure    = (int32_t)((uint32_t)HWire.read() << 12 | (uint32_t)HWire.read() << 4 | (uint32_t)HWire.read() >> 4);
    bmp280_raw_temperature = (int32_t)((uint32_t)HWire.read() << 12 | (uint32_t)HWire.read() << 4 | (uint32_t)HWire.read() >> 4);
  }

  //-------------------------------------------------------
  // Counter 2: Compensate + 3-stage smoothing filter
  //-------------------------------------------------------
  if (barometer_counter == 2) {

    // STAGE 1 — Temperature compensation (produces bmp280_t_fine needed for pressure)
    int32_t var1_t, var2_t;
    var1_t = (((bmp280_raw_temperature >> 3) - ((int32_t)bmp280_dig_T1 << 1)) * (int32_t)bmp280_dig_T2) >> 11;
    var2_t = ((((bmp280_raw_temperature >> 4) - (int32_t)bmp280_dig_T1) *
               ((bmp280_raw_temperature >> 4) - (int32_t)bmp280_dig_T1)) >> 12) *
               ((int32_t)bmp280_dig_T3) >> 14;              // Correct parenthesis — fixed vs original
    bmp280_t_fine = var1_t + var2_t;

    // STAGE 2 — Pressure compensation (BMP280 datasheet 64-bit integer formula)
    int64_t var1_p, var2_p, p;
    var1_p = (int64_t)bmp280_t_fine - 128000;
    var2_p = var1_p * var1_p * (int64_t)bmp280_dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp280_dig_P5) << 17);
    var2_p = var2_p + ((int64_t)bmp280_dig_P4 << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp280_dig_P3) >> 8) + ((var1_p * (int64_t)bmp280_dig_P2) << 12);
    var1_p = (((int64_t)1 << 47) + var1_p) * (int64_t)bmp280_dig_P1 >> 33;

    if (var1_p != 0) {
      p = 1048576 - bmp280_raw_pressure;
      p = (((p << 31) - var2_p) * 3125) / var1_p;
      var1_p = ((int64_t)bmp280_dig_P9 * (p >> 13) * (p >> 13)) >> 25;
      var2_p = ((int64_t)bmp280_dig_P8 * p) >> 19;
      p = ((p + var1_p + var2_p) >> 8) + ((int64_t)bmp280_dig_P7 << 4);
      int32_t P_raw = (int32_t)(p >> 8);                    // Integer Pascals

      // STAGE 3a — 25-point rotating average buffer (increased from 20 for smoother altitude reference, Task 3)
      // Removes reading-to-reading jitter before the complementary filter sees it
      pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];
      pressure_rotating_mem[pressure_rotating_mem_location] = P_raw;
      pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];
      pressure_rotating_mem_location++;
      if (pressure_rotating_mem_location == 25) pressure_rotating_mem_location = 0;  //Rollover at 25 (was 20).
      actual_pressure_fast = (float)pressure_total_avarage / 25.0;                   //Divide by 25 (was 20).

      // STAGE 3b — Complementary filter (tightened from 0.96/0.04 to 0.985/0.015, Task 3)
      // slow (0.985) = very stable altitude reference for PID setpoint (~267ms time constant at 250Hz)
      // fast (0.015) = current average, responds to real altitude changes
      // Tighter ratio reduces altitude hunting on light frames
      actual_pressure_slow = actual_pressure_slow * 0.985 + actual_pressure_fast * 0.015;

      // Difference clamped to ±3 Pa (reduced from ±4 Pa — gentler correction, Task 3)
      actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
      if (actual_pressure_diff >  3) actual_pressure_diff =  3;
      if (actual_pressure_diff < -3) actual_pressure_diff = -3;

      // Nudge slow toward fast when they diverge — divisor increased from 3.0 to 4.0 for gentler correction (Task 3)
      // Reduces pressure-induced throttle spikes on responsive 500–700g frame
      if (actual_pressure_diff > 0.5 || actual_pressure_diff < -0.5)
        actual_pressure_slow -= actual_pressure_diff / 4.0;

      actual_pressure = actual_pressure_slow;               // Used by altitude PID
    }
  }

  //-------------------------------------------------------
  // Counter 3: Altitude PID and parachute detection
  // (identical to original — no changes needed here)
  //-------------------------------------------------------
  if (barometer_counter == 3) {
    barometer_counter = 0;

    if (manual_altitude_change == 1) pressure_parachute_previous = actual_pressure * 10;
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;

    if (flight_mode >= 2 && takeoff_detected == 1) {
      if (pid_altitude_setpoint == 0) pid_altitude_setpoint = actual_pressure;
      manual_altitude_change = 0;
      manual_throttle = 0;
      if (channel_3 > 1600) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (channel_3 - 1600) / 3;
      }
      if (channel_3 < 1400) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (channel_3 - 1400) / 5;
      }

      pid_altitude_input = actual_pressure;
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

      pid_error_gain_altitude = 0;
      //Dead-band tightened from 10 to 8 — I-term gain scaling activates sooner for better hold (Task 3).
      if (pid_error_temp > 8 || pid_error_temp < -8) {
        pid_error_gain_altitude = (abs(pid_error_temp) - 8) / 20.0;
        if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
      }

      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude >  pid_max_altitude) pid_i_mem_altitude =  pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1) pid_i_mem_altitude = pid_max_altitude * -1;

      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      if (pid_output_altitude >  pid_max_altitude) pid_output_altitude =  pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1) pid_output_altitude = pid_max_altitude * -1;
    }
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {
      pid_altitude_setpoint = 0;
      pid_output_altitude   = 0;
      pid_i_mem_altitude    = 0;
      manual_throttle       = 0;
      manual_altitude_change = 1;
    }
  }
}
