/****************************************************************
  Here you have all the parsing stuff for uBlox
 ****************************************************************/
 // Code from Jordi, modified by Jose
 
 //You have to disable all the other string, only leave this ones:
 
 //NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
 //NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
 //NAV-STATUS Receiver Navigation Status, PAGE 67 of datasheet

 // Baud Rate:38400

/* 
 GPSfix Type 
 - 0x00 = no fix
 - 0x01 = dead reckonin
 - 0x02 = 2D-fix
 - 0x03 = 3D-fix
 - 0x04 = GPS + dead re
 - 0x05 = Time only fix
 - 0x06..0xff = reserved*/
 
 //Luckly uBlox has internal EEPROM so all the settings you change will remain forever.  Not like the SIRF modules!
 

/****************************************************************
 * 
 ****************************************************************/
// optimization : This code don´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void decode_gps(void)
{
  static unsigned long GPS_timer=0;
  byte data;
  int numc;
  
  numc = Serial.available();
  if (numc > 0)
    for (int i=0;i<numc;i++)  // Process bytes received
    {
      data = Serial.read();
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //OH first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi=data;
        checksum(UBX_payload_length_hi);
        UBX_step++;
        break;
      case 5:
        UBX_payload_length_lo=data;
        checksum(UBX_payload_length_lo);
        UBX_step++;
        break;
      case 6:         // Payload data read...
	if (UBX_payload_counter < UBX_payload_length_hi)  // We stay in this state until we reach the payload_length
        {
          UBX_buffer[UBX_payload_counter] = data;
          checksum(data);
          UBX_payload_counter++;
          if (UBX_payload_counter==UBX_payload_length_hi)
            UBX_step++;
        }
        break;
      case 7:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
	  	parse_ubx_gps();               // Parse the new GPS packet
        //else
            //Serial.println("Err CHK!!");
        // Variable initialization
        UBX_step=0;
        UBX_payload_counter=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=millis(); //Restarting timer...
        break;
	}
    }    // End for...
}

/****************************************************************
 * 
 ****************************************************************/
void parse_ubx_gps()
{
  int j;
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  if(UBX_class==0x01) 
  {
    switch(UBX_id)//Checking the UBX ID
    {
    case 0x02: //ID NAV-POSLLH 
      j=0;
      iTOW = join_4_bytes(&UBX_buffer[j]);
      j+=4;
      lon = (float)join_4_bytes(&UBX_buffer[j])/10000000.0;
      j+=4;
      lat = (float)join_4_bytes(&UBX_buffer[j])/10000000.0;
      j+=4;
      alt = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
      j+=4;
      alt_MSL = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
      j+=4;
      /*
      hacc = (float)join_4_bytes(&UBX_buffer[j])/(float)1000;
      j+=4;
      vacc = (float)join_4_bytes(&UBX_buffer[j])/(float)1000;
      j+=4;
      */
      data_update_event|=0x01;
      break;
    case 0x03://ID NAV-STATUS 
      if(UBX_buffer[4] >= 0x03)
      {
        gpsFix=0; //valid position
        digitalWrite(6,HIGH);//Turn LED when gps is fixed. 
      }
      else
      {
        gpsFix=1; //invalid position
        digitalWrite(6,LOW);
      }
      break;

    case 0x12:// ID NAV-VELNED 
      j=16;
      speed_3d = (float)join_4_bytes(&UBX_buffer[j])/(float)100; // m/s
      j+=4;
      ground_speed = (float)join_4_bytes(&UBX_buffer[j])/(float)100; // Ground speed 2D
      j+=4;
      ground_course = (float)join_4_bytes(&UBX_buffer[j])/(float)100000; // Heading 2D
      j+=4;
      /*
      sacc = join_4_bytes(&UBX_buffer[j]) // Speed accuracy
      j+=4;
      headacc = join_4_bytes(&UBX_buffer[j]) // Heading accuracy
      j+=4;
      */
      data_update_event|=0x02; //Update the flag to indicate the new data has arrived.
      break; 
      }
    }   
}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
int32_t join_4_bytes(byte Buffer[])
{
  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}
