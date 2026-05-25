///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the GPS module is setup and read.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gps_setup(void) {

  Serial2.begin(9600);
  delay(250);

  //Disable GPGSV messages by using the ublox protocol.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial2.write(Disable_GPGSV, 11);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the refresh rate to 5Hz by using the ublox protocol.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial2.write(Set_to_5Hz, 14);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the baud rate to 57.6kbps by using the ublox protocol.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
  Serial2.write(Set_to_57kbps, 28);
  delay(200);

  Serial2.begin(57600);
  delay(200);
}

void read_gps(void) {
  while (Serial2.available() && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = Serial2.read();                                                              //Load a new serial byte in the read_serial_byte variable.
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;                                                                                  //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
      //LED toggling removed — PC13 is now driven by gps_led_update() in LED_control.ino (Task 1).
      //Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    //If the line starts with GA we can scan for fix quality and satellite count using comma-delimited parsing.
    //This is robust regardless of coordinate format or field width differences between GPS modules.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A') {
      //Find comma positions in the GGA sentence to locate fields reliably.
      //GGA format: $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,age,ref*cs
      //Fields:       0     1    2   3   4   5   6    7    8    9 10  11  12 13  14
      uint8_t comma_count = 0;
      uint8_t comma_pos[15];                                                                               //Store positions of first 15 commas.
      for (uint8_t i = 0; i <= 99 && comma_count < 15; i++) {
        if (incomming_message[i] == ',') {
          comma_pos[comma_count] = i;
          comma_count++;
        }
        if (incomming_message[i] == '*') break;                                                            //Stop at end-of-sentence marker.
      }

      //We need at least 8 commas to have fix quality (field 6) and satellites (field 7).
      if (comma_count >= 8) {
        //Field 6 = fix quality: starts at comma_pos[5]+1 (after 6th comma, 0-indexed comma 5).
        uint8_t fix_pos = comma_pos[5] + 1;
        char fix_char = incomming_message[fix_pos];

        if (fix_char == '1' || fix_char == '2') {                                                          //Valid GPS or DGPS fix.
          //Parse latitude from field 2 (between comma 1 and comma 2) using original fixed offsets.
          //The original YMFC code assumes ddmm.mmmm for lat and dddmm.mmmm for lon.
          //These offsets are relative to the start of each field.
          lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;
          lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;
          lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;
          lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;
          lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;
          lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;
          lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;
          lat_gps_actual /= (long)6;
          lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;
          lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;
          lat_gps_actual /= 10;

          lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;
          lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;
          lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;
          lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;
          lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;
          lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;
          lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;
          lon_gps_actual /= (long)6;
          lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;
          lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;
          lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;
          lon_gps_actual /= 10;

          if (incomming_message[28] == 'N')latitude_north = 1;
          else latitude_north = 0;

          if (incomming_message[42] == 'E')longiude_east = 1;
          else longiude_east = 0;

          //Field 7 = number of satellites: between comma_pos[6] and comma_pos[7].
          //Parse 1 or 2 digit satellite count from the correct position.
          uint8_t sat_start = comma_pos[6] + 1;
          uint8_t sat_end = comma_pos[7];
          uint8_t sat_len = sat_end - sat_start;
          if (sat_len == 1) {
            number_used_sats = (int)incomming_message[sat_start] - 48;                                     //Single digit satellite count.
          }
          else if (sat_len == 2) {
            number_used_sats = ((int)incomming_message[sat_start] - 48) * 10;                              //Tens digit.
            number_used_sats += (int)incomming_message[sat_start + 1] - 48;                                //Ones digit.
          }
          else {
            number_used_sats = 0;                                                                          //Unexpected format, treat as no sats.
          }

          if (lat_gps_previous == 0 && lon_gps_previous == 0) {
            lat_gps_previous = lat_gps_actual;
            lon_gps_previous = lon_gps_actual;
          }

          lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;
          lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;

          l_lat_gps = lat_gps_previous;
          l_lon_gps = lon_gps_previous;

          lat_gps_previous = lat_gps_actual;
          lon_gps_previous = lon_gps_actual;

          gps_add_counter = 5;
          new_gps_data_counter = 9;
          lat_gps_add = 0;
          lon_gps_add = 0;
          new_gps_data_available = 1;
        }
      }
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;

  }

  //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }

  if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    //LED toggling removed — PC13 is now driven by gps_led_update() in LED_control.ino (Task 1).
    gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.

    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    }

    if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
      //Step 1: Calculate position error — deviation from the locked waypoint in raw GPS integer units.
      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Longitude error: positive = drone is west of waypoint.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Latitude error: positive = drone is north of waypoint.

      //Step 2: Dead-band — ignore errors within ±15 GPS units to prevent jitter fighting the attitude PID.
      //This prevents micro-corrections when the drone is already near the target (Task 2).
      if (abs(gps_lat_error) < 15) gps_lat_error = 0;
      if (abs(gps_lon_error) < 15) gps_lon_error = 0;

      //Step 3: D-term velocity damping via rotating memory (change in error between GPS updates).
      //The difference (error - previous_error) approximates GPS-derived velocity at the 5Hz update rate.
      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the oldest value to make room.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Store the new velocity sample (delta error per GPS tick).
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add to running average for D-term smoothing.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the oldest value to make room.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Store the new velocity sample (delta error per GPS tick).
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add to running average for D-term smoothing.
      gps_rotating_mem_location++;                                                                        //Advance the rotating memory index.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Wrap around at 35 entries.

      //Step 4: Remember current error for next D-term calculation.
      gps_lat_error_previous = gps_lat_error;                                                             //Store lat error for next loop's delta.
      gps_lon_error_previous = gps_lon_error;                                                             //Store lon error for next loop's delta.

      //Step 5: Calculate GPS pitch and roll correction as if the nose is facing north.
      //P-term: proportional to position error — drives drone toward waypoint.
      //D-term: proportional to averaged velocity — damps oscillation around waypoint.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustment because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustment because the quadcopter is flying west of the prime meridian.

      //Step 6: Rotate corrections from north-referenced frame to the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      //Step 7: Clamp outputs to ±150 — reduced from ±300 to limit aggressive correction on 500–700g frame (Task 2).
      if (gps_roll_adjust > 150) gps_roll_adjust = 150;
      if (gps_roll_adjust < -150) gps_roll_adjust = -150;
      if (gps_pitch_adjust > 150) gps_pitch_adjust = 150;
      if (gps_pitch_adjust < -150) gps_pitch_adjust = -150;
    }
  }

  if (gps_watchdog_timer + 1000 < millis()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
    if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
      flight_mode = 2;                                                                                    //Set the flight mode to 2.
      error = 4;                                                                                          //Output an error.
    }
  }

  if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              //If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
    }
  }
}

