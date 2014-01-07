// R/C RADIO PPM SIGNAL READ (USING TIMER1 INPUT CAPTURE AND OVERFLOW INTERRUPTS)
// AND SERVO OUTPUT ON OC1A, OC1B pins [Disabled now]
// SERVO OUTPUT FUNCTION USING TIMER2 OVERFLOW INTERRUPT 

// Timer1 Overflow 
// Detects radio signal lost and generate servo outputs (overflow at 22ms (45Hz))
ISR(TIMER1_OVF_vect){

  //TCNT1 = 20000;        // at 16Mhz (0.5us) (65535-20000) = 22 ms (45Hz)
  //OCR1A = 20000 + (Servo1<<1);  // Output for servos...
  //OCR1B = 20000 + (Servo2<<1);
  
  //TCCR1A = 0xF0;        // Set OC1A/OC1B on Compare Match
  //TCCR1C = 0xC0;        // Force Output Compare A/B (Start Servo pulse)
  //TCCR1C = 0x00;        
  //TCCR1A = 0xA0;  	// Clear OC1A/OC1B on Compare Match
  
  TCNT1 = 0;
  Timer1_last_value=0xFFFF;  // Last value before overflow...

  // Radio signal lost...
  radio_status = 0;
}

// Capture RX pulse train using TIMER 1 CAPTURE INTERRUPT
// And also send Servo pulses using OCR1A and OCR1B [disabled now]
// Servo output is synchronized with input pulse train
ISR(TIMER1_CAPT_vect)
{
   if(!bit_is_set(TCCR1B ,ICES1)){	    // falling edge?
	 if(Rx_ch == MAX_CHANNELS) {        // This should be the last pulse...
	     Pulses[Rx_ch++] = ICR1;  
             radio_status = 1;              // Rx channels ready...
	  }
         TCCR1B = 0x42;              // Next time : rising edge
         Timer1_last_value = TCNT1;  // Take last value before reset         
         TCNT1 = 0;                  // Clear counter
         
         // Servo Output on OC1A/OC1B... (syncronised with radio pulse train)
         //TCCR1A = 0xF0;        // Set OC1A/OC1B on Compare Match
         //TCCR1C = 0xC0;        // Force Output Compare A/B (Start Servo pulse)
         //TCCR1C = 0x00;      
         //TCCR1A = 0xA0;  	// Clear OC1A/OC1B on Compare Match         
   }
   else {				  // Rise edge
	  if ((ICR1-ICR1_old) >= SYNC_GAP_LEN){   // SYNC pulse?
		Rx_ch = 1;	          // Channel = 1
                Pulses[0] = ICR1;
	    }
          else {
            if(Rx_ch <= MAX_CHANNELS)
	       Pulses[Rx_ch++] = ICR1;    // Store pulse length
            if(Rx_ch == MAX_CHANNELS)
               TCCR1B = 0x02;             // Next time : falling edge
            }
   }
   ICR1_old = ICR1;
}


// Servo Input PPM Initialization routine
void RxServoInput_ini()
{
  pinMode(icpPin,INPUT);
  Rx_ch = 1;
  //TCCR1A = 0xA0;           // Normal operation mode, PWM Operation disabled, clear OC1A/OC1B on Compare Match
  TCCR1A = 0x00;	     // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM 
  TCCR1B = 0x42;	     // TCNT1 preescaler/8 (16Mhz => 0.5useg, 8 Mhz => 1useg), Rising edge
  TIMSK1 = _BV(ICIE1)|_BV (TOIE1);   // Enable interrupts : Timer1 Capture and Timer1 Overflow
 
  Neutro[1] = 1037;
  Neutro[2] = 1037;
  Neutro[3] = 1037;
  Neutro[4] = 1037;
  Servo1 = 1037;
  Servo2 = 1037;
}

int RxGetChannelPulseWidth( uint8_t channel)
{
  unsigned int result;
  unsigned int result2;
  unsigned int pulso_ant;
  unsigned int pulso_act;
  
  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
  result =  Pulses[channel];
  result2 =  Pulses[channel];
  if (result != result2)
    result =  Pulses[channel];   // if the results are different we make a third reading (this should be fine)
  pulso_act = result;
  
  pulso_ant = Pulses[channel-1];
  result2 =  Pulses[channel-1];
  if (pulso_ant != result2)
     pulso_ant = Pulses[channel-1];   // if the results are different we make a third reading (this should be fine)

    result = (result - pulso_ant)>>1;        // Restamos con el valor del pulso anterior y pasamos a microsegundos (reloj a 0.5us)
  	 
  if ((result > MIN_IN_PULSE_WIDTH)&&(result < MAX_IN_PULSE_WIDTH))  // Out of range?
    return result;
  else
    {
    return Neutro[channel];
    }
}

// SERVO SIGNAL OUTPUT (USING TIMER2 OVERFLOW INTERRUPT AND TIMER1 READINGS)
ISR (TIMER2_OVF_vect)
{ 
  int us;
  int aux;
  
  if (Servo_Channel < num_servos){
    Servo_Timer2_timer1_stop = TCNT1;       // take the timer1 value at this moment
    
    // Now we are going to calculate the time we need to wait until pulse end
    if (Servo_Timer2_timer1_stop>Servo_Timer2_timer1_start)   // Timer1 reset during the pulse?
      Servo_Timer2_pulse_length = Servo_Timer2_timer1_stop-Servo_Timer2_timer1_start;
    else
      Servo_Timer2_pulse_length = ((long)Servo_Timer2_timer1_stop + Timer1_last_value) - (long)Servo_Timer2_timer1_start;
    us = (Servos[Servo_Channel].value) - (Servo_Timer2_pulse_length>>1);
    us -= 2;  // Adjust for the time of this code
    if (us>1)
      {
      us <<= 2; // Translate us to the 4 cyles loop (1/4 us)
      __asm__ __volatile__ (  // 4 cycles loop = 1/4 us  (taken from delayMicroSeconds function)
	"1: sbiw %0,1" "\n\t"            // 2 cycles
	"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
      }
    digitalWrite( Servos[Servo_Channel].pin,LOW);    // pulse this channel low
    Servo_Channel++;                                 // increment to the next channel
    }
  else
    Servo_Channel = 0;                // SYNC pulse end => Start again on first channel
   
  if (Servo_Channel == num_servos){           // This is the SYNC PULSE
    TCCR2B = _BV(CS20)|_BV(CS21)|_BV(CS22);   // set prescaler of 1024 => 64us resolution (overflow = 16384uS) 
    TCNT2 = 0x04;   //64usx4 = 256us
    }
  else{
    TCCR2B = _BV(CS20)|_BV(CS22);                  // Set prescaler of 128  (8uS resolution at 16Mhz) 
    TCNT2 = Servos[Servo_Channel].counter;         // Set the clock counter register for the overflow interrupt
    Servo_Timer2_timer1_start = TCNT1;             // we take the value of Timer1 at the start of the pulse
    digitalWrite(Servos[Servo_Channel].pin,HIGH);  // Pulse start. Let´s go...
    }
}

void Servo_Timer2_ini()
{ 
  // Servos must have correct values at this moment !! Call First Servo_Timer2_set() function...
  // Variable initialization  
  Servo_Channel = 0;
  TIMSK2 = 0;  // Disable interrupts 
  TCCR2A = 0;  // normal counting mode 
  TCCR2B = _BV(CS20)|_BV(CS22);            // Set prescaler of 128  (8uS resolution at 16Mhz)
  TCNT2 = Servos[Servo_Channel].counter;   // Set the clock counter register for the overflow interrupt
  TIFR2 = _BV(TOV2);  // clear pending interrupts; 
  TIMSK2 =  _BV(TOIE2) ; // enable the overflow interrupt	  
}

void Servo_Timer2_set(uint8_t servo_index, int value)
{
  int aux;
    
  if (value > SERVO_MAX_PULSE_WIDTH)
    value = SERVO_MAX_PULSE_WIDTH;
  else if (value < SERVO_MIN_PULSE_WIDTH)
    value = SERVO_MIN_PULSE_WIDTH;
  
  Servos[servo_index].value = value; // Store the desired value on Servo structure
  
  value = value - 20;  // We reserve 20us for compensation...
  
  // Calculate the overflow interrupt counter (8uS step)
  aux = value>>3;  // value/8
  Servos[servo_index].counter = 256 - aux;
}



