///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPS Diagnostic Tool v2 - Auto Baud Detection for BN-880 / M8N on STM32 BluePill
// 
// This sketch automatically scans common baud rates to find your GPS module.
// Open Serial Monitor at 57600 baud.
//
// Commands (type letter + Enter):
//   s = Scan all baud rates to find GPS (RUN THIS FIRST!)
//   r = Show raw NMEA for 5 seconds at current baud rate
//   g = Show parsed GPS data
//   d = Dump last GGA with index map
//   u = Try ublox 57600 switch (send ublox command then listen at 57600)
//   h = Help
//
// GPS is on Serial2 (PA3=RX, PA2=TX on BluePill)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define STM32_board_LED PC13

char incomming_message[100];
char last_gga[100];
uint16_t message_counter = 0;
uint8_t new_line_found = 0;
uint8_t has_gga = 0;
uint8_t number_used_sats = 0;
uint8_t fix_quality = 0;
uint8_t fix_type = 0;
uint8_t latitude_north = 0, longiude_east = 0;

long current_baud = 0;  // Currently active baud rate (0 = not found yet)
uint32_t led_timer = 0;
bool led_state = false;

// Common GPS baud rates to scan
const long baud_rates[] = {9600, 19200, 38400, 57600, 115200, 4800};
const uint8_t num_rates = 6;

void setup() {
  pinMode(STM32_board_LED, OUTPUT);
  digitalWrite(STM32_board_LED, HIGH);  // LED off

  Serial.begin(57600);
  delay(1000);

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("   GPS DIAGNOSTIC v2 - AUTO BAUD DETECTION"));
  Serial.println(F("   For BN-880 / Ublox M8N on STM32 BluePill"));
  Serial.println(F("================================================"));
  Serial.println(F("Serial2 pins: PA2(TX) / PA3(RX)"));
  Serial.println();
  Serial.println(F(">>> Type 's' + Enter to SCAN for GPS baud rate"));
  Serial.println(F(">>> Type 'h' + Enter for all commands"));
  Serial.println();
}

void loop() {
  // Background GPS reading if we have a baud rate
  if (current_baud > 0) {
    read_gps_background();
    // LED: solid if >= 4 sats, blink otherwise
    if (number_used_sats >= 4) {
      digitalWrite(STM32_board_LED, LOW);
    } else if (millis() - led_timer >= 500) {
      led_timer = millis();
      led_state = !led_state;
      digitalWrite(STM32_board_LED, led_state ? LOW : HIGH);
    }
  }

  // Check for commands
  if (Serial.available()) {
    char cmd = Serial.read();
    delay(10);
    while (Serial.available()) Serial.read();  // flush

    switch (cmd) {
      case 's': case 'S': cmd_scan(); break;
      case 'r': case 'R': cmd_raw(); break;
      case 'g': case 'G': cmd_parsed(); break;
      case 'd': case 'D': cmd_dump(); break;
      case 'u': case 'U': cmd_ublox_switch(); break;
      case 'h': case 'H': cmd_help(); break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SCAN - Try each baud rate and look for valid NMEA data ($GP or $GN prefix)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_scan() {
  Serial.println();
  Serial.println(F("=== SCANNING ALL BAUD RATES FOR GPS DATA ==="));
  Serial.println(F("Make sure GPS TX -> PA3 (Serial2 RX)"));
  Serial.println();

  current_baud = 0;

  for (uint8_t b = 0; b < num_rates; b++) {
    Serial.print(F("Trying "));
    Serial.print(baud_rates[b]);
    Serial.print(F(" baud ... "));

    Serial2.begin(baud_rates[b]);
    delay(200);

    // Flush any garbage
    while (Serial2.available()) Serial2.read();

    // Listen for 2 seconds for valid NMEA data
    uint32_t timeout = millis() + 2000;
    bool found_dollar = false;
    bool found_nmea = false;
    char check_buf[6];
    uint8_t check_idx = 0;
    uint16_t bytes_received = 0;

    while (millis() < timeout) {
      if (Serial2.available()) {
        char c = Serial2.read();
        bytes_received++;

        if (c == '$') {
          found_dollar = true;
          check_idx = 0;
        } else if (found_dollar && check_idx < 5) {
          check_buf[check_idx] = c;
          check_idx++;
          if (check_idx == 5) {
            check_buf[5] = '\0';
            // Check for common NMEA talker IDs: GP, GN, GL, GA
            if ((check_buf[0] == 'G' && (check_buf[1] == 'P' || check_buf[1] == 'N' || check_buf[1] == 'L' || check_buf[1] == 'A'))) {
              found_nmea = true;
              break;
            }
            found_dollar = false;  // Reset, try next $
          }
        }
      }
    }

    Serial2.end();

    if (found_nmea) {
      Serial.print(F("FOUND! ("));
      Serial.print(bytes_received);
      Serial.println(F(" bytes, valid NMEA)"));
      Serial.println();
      Serial.print(F(">>> GPS detected at "));
      Serial.print(baud_rates[b]);
      Serial.println(F(" baud! <<<"));

      current_baud = baud_rates[b];
      Serial2.begin(current_baud);

      Serial.println();
      Serial.println(F("Now type:"));
      Serial.println(F("  'r' = see raw NMEA sentences"));
      Serial.println(F("  'g' = see parsed data (wait ~10 sec for sats)"));
      Serial.println(F("  'd' = dump GGA with index map"));
      Serial.println(F("============================================"));
      return;
    } else if (bytes_received > 0) {
      Serial.print(F("got "));
      Serial.print(bytes_received);
      Serial.println(F(" bytes but no valid NMEA (wrong baud or garbage)"));
    } else {
      Serial.println(F("no data"));
    }
  }

  Serial.println();
  Serial.println(F("!!! NO GPS FOUND AT ANY BAUD RATE !!!"));
  Serial.println(F("Check wiring:"));
  Serial.println(F("  GPS TX  -> PA3  (STM32 Serial2 RX)"));
  Serial.println(F("  GPS RX  -> PA2  (STM32 Serial2 TX)"));
  Serial.println(F("  GPS VCC -> 3.3V or 5V"));
  Serial.println(F("  GPS GND -> GND"));
  Serial.println(F("============================================"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RAW - Dump raw NMEA for 5 seconds
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_raw() {
  if (current_baud == 0) {
    Serial.println(F("No baud rate set! Type 's' first to scan."));
    return;
  }
  Serial.println();
  Serial.print(F("=== RAW NMEA @ "));
  Serial.print(current_baud);
  Serial.println(F(" baud (5 sec) ==="));

  uint32_t end_time = millis() + 5000;
  while (millis() < end_time) {
    if (Serial2.available()) Serial.write(Serial2.read());
  }

  Serial.println();
  Serial.println(F("=== END RAW ==="));
  new_line_found = 0;
  message_counter = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARSED - Show parsed GPS info
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_parsed() {
  if (current_baud == 0) {
    Serial.println(F("No baud rate set! Type 's' first to scan."));
    return;
  }
  Serial.println();
  Serial.println(F("========== PARSED GPS DATA =========="));
  Serial.print(F("Baud Rate   : ")); Serial.println(current_baud);
  Serial.print(F("Fix Quality : ")); Serial.print(fix_quality);
  if (fix_quality == 0) Serial.println(F(" (No fix)"));
  else if (fix_quality == 1) Serial.println(F(" (GPS fix)"));
  else if (fix_quality == 2) Serial.println(F(" (DGPS fix)"));
  else Serial.println(F(" (Other)"));

  Serial.print(F("Fix Type    : ")); Serial.print(fix_type);
  if (fix_type == 1) Serial.println(F(" (No fix)"));
  else if (fix_type == 2) Serial.println(F(" (2D fix)"));
  else if (fix_type == 3) Serial.println(F(" (3D fix)"));
  else Serial.println(F(" (Unknown)"));

  Serial.print(F("Satellites  : ")); Serial.println(number_used_sats);
  Serial.print(F("Hemisphere  : ")); Serial.print(latitude_north ? "North" : "South");
  Serial.print(F(" / ")); Serial.println(longiude_east ? "East" : "West");
  Serial.print(F("LED would be: ")); Serial.println(number_used_sats >= 4 ? "SOLID (locked)" : "BLINKING (searching)");

  if (has_gga) {
    Serial.println(F("--- Old Code Index Check ---"));
    Serial.print(F("  [44]='"));  Serial.print(last_gga[44]);
    Serial.print(F("' need '1'/'2' -> "));
    Serial.println((last_gga[44] == '1' || last_gga[44] == '2') ? "MATCH" : "MISMATCH!");
    Serial.print(F("  [46]='"));  Serial.print(last_gga[46]); Serial.println(F("' (sat tens)"));
    Serial.print(F("  [47]='"));  Serial.print(last_gga[47]); Serial.println(F("' (sat ones)"));
  } else {
    Serial.println(F("(No GGA received yet - wait a few seconds and retry)"));
  }
  Serial.println(F("====================================="));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DUMP - Show last GGA with character positions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_dump() {
  if (!has_gga) {
    Serial.println(F("No GGA received yet. Wait a few seconds and retry."));
    return;
  }
  Serial.println();
  Serial.println(F("========== GGA INDEX MAP =========="));

  // Find end
  uint8_t end = 80;
  for (uint8_t i = 1; i < 99; i++) {
    if (last_gga[i] == '*' || last_gga[i] == '-') { end = i; break; }
  }

  // Print GGA string
  Serial.print(F("$"));
  for (uint8_t i = 1; i <= end; i++) Serial.print(last_gga[i]);
  Serial.println();

  // Tens ruler
  Serial.print(F("T: "));
  for (uint8_t i = 0; i <= end; i++) {
    if (i % 10 == 0) Serial.print(i / 10);
    else Serial.print(' ');
  }
  Serial.println();

  // Ones ruler
  Serial.print(F("I: "));
  for (uint8_t i = 0; i <= end; i++) Serial.print(i % 10);
  Serial.println();

  // Char line
  Serial.print(F("C: "));
  for (uint8_t i = 0; i <= end; i++) Serial.print(last_gga[i]);
  Serial.println();
  Serial.println();

  // Comma list
  Serial.println(F("Field boundaries (comma positions):"));
  uint8_t cn = 0;
  for (uint8_t i = 0; i <= end; i++) {
    if (last_gga[i] == ',') {
      Serial.print(F("  comma["));  Serial.print(cn);
      Serial.print(F("] @ idx ")); Serial.print(i);
      Serial.print(F(" -> field ")); Serial.println(cn + 1);
      cn++;
    }
  }
  Serial.println(F("==================================="));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UBLOX SWITCH - Attempt ublox protocol baud switch to 57600
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_ublox_switch() {
  Serial.println();
  Serial.println(F("=== ATTEMPTING UBLOX 57600 SWITCH ==="));

  // Start at 9600 (factory default)
  Serial2.begin(9600);
  delay(200);
  Serial.println(F("Sending ublox disable GPGSV..."));
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial2.write(Disable_GPGSV, 11);
  delay(350);

  Serial.println(F("Sending ublox set 5Hz..."));
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial2.write(Set_to_5Hz, 14);
  delay(350);

  Serial.println(F("Sending ublox set 57600 baud..."));
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
  Serial2.write(Set_to_57kbps, 28);
  delay(200);

  Serial.println(F("Switching Serial2 to 57600..."));
  Serial2.begin(57600);
  delay(500);

  // Check for data
  while (Serial2.available()) Serial2.read();  // flush
  uint32_t timeout = millis() + 2000;
  uint16_t count = 0;
  bool found = false;
  while (millis() < timeout) {
    if (Serial2.available()) {
      char c = Serial2.read();
      count++;
      if (c == '$') found = true;
    }
  }

  if (found) {
    Serial.println(F("SUCCESS! GPS responding at 57600 baud."));
    current_baud = 57600;
  } else if (count > 0) {
    Serial.print(F("Got ")); Serial.print(count);
    Serial.println(F(" bytes but no valid NMEA. Switch may have failed."));
    Serial.println(F("Run 's' to scan again."));
  } else {
    Serial.println(F("FAILED - no data at 57600. GPS may not support ublox protocol."));
    Serial.println(F("Run 's' to scan again."));
  }
  Serial.println(F("====================================="));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HELP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_help() {
  Serial.println();
  Serial.println(F("--- GPS DIAGNOSTIC COMMANDS ---"));
  Serial.println(F("  s = Scan baud rates (DO THIS FIRST)"));
  Serial.println(F("  r = Raw NMEA dump (5 sec)"));
  Serial.println(F("  g = Parsed GPS data"));
  Serial.println(F("  d = Dump GGA with index map"));
  Serial.println(F("  u = Try ublox 57600 switch"));
  Serial.println(F("  h = This help"));
  Serial.println(F("-------------------------------"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Background GPS reader (comma-based parsing)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_gps_background() {
  while (Serial2.available() && new_line_found == 0) {
    char c = Serial2.read();
    if (c == '$') {
      for (message_counter = 0; message_counter <= 99; message_counter++)
        incomming_message[message_counter] = '-';
      message_counter = 0;
    }
    else if (message_counter <= 99) message_counter++;
    if (message_counter <= 99) incomming_message[message_counter] = c;
    if (c == '*') new_line_found = 1;
  }

  if (new_line_found == 1) {
    new_line_found = 0;

    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {
      number_used_sats = 0;
      fix_quality = 0;
    }

    if (incomming_message[4] == 'G' && incomming_message[5] == 'A') {
      for (uint8_t i = 0; i < 100; i++) last_gga[i] = incomming_message[i];
      has_gga = 1;

      uint8_t cc = 0;
      uint8_t cp[15];
      for (uint8_t i = 0; i <= 99 && cc < 15; i++) {
        if (incomming_message[i] == ',') { cp[cc] = i; cc++; }
        if (incomming_message[i] == '*') break;
      }

      if (cc >= 8) {
        fix_quality = incomming_message[cp[5] + 1] - '0';
        uint8_t s1 = cp[6] + 1, s2 = cp[7], sl = s2 - s1;
        if (sl == 1) number_used_sats = incomming_message[s1] - '0';
        else if (sl == 2) number_used_sats = (incomming_message[s1] - '0') * 10 + (incomming_message[s1 + 1] - '0');
        else number_used_sats = 0;
        latitude_north = (incomming_message[cp[2] + 1] == 'N') ? 1 : 0;
        longiude_east = (incomming_message[cp[4] + 1] == 'E') ? 1 : 0;
      }
    }

    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')
      fix_type = (int)incomming_message[9] - 48;
  }
}
