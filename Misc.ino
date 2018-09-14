//================================================================
void SetPINS()
{
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
} //SetPINS

//================================================================
void SetTimerInterrupt()
{  
  // Use TIMER 1 for stepper motor X AXIS and Timer 3 for Y AXIS
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = ZERO_SPEED;   // Motor stopped
  TCNT1 = 0;

  // We use TIMER 3 for stepper motor Y AXIS 
  // STEPPER MOTORS INITIALIZATION
  // TIMER3 CTC MODE
  TCCR3B &= ~(1<<WGM13);
  TCCR3B |=  (1<<WGM12);
  TCCR3A &= ~(1<<WGM11); 
  TCCR3A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR3A &= ~(3<<COM1A0); 
  TCCR3A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR3B = (TCCR3B & ~(0x07<<CS10)) | (2<<CS10);

  OCR3A = ZERO_SPEED;   // Motor stopped
  TCNT3 = 0;

  delay(1000);
  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  TIMSK3 |= (1<<OCIE1A);  // Enable Timer1 interrupt
} // SetTimerInterrupt

//================================================================
// Test sequence to check mechanics, motor drivers...
void testMovements()
{
//  if (loop_counter >= 9000) {
//    testmode = false;
//    return;
//  }
//  max_speed = user_max_speed;
//  if (loop_counter > 8000)
//    setPosition_straight(200, 60);
//  else if (loop_counter > 6260)
//    setPosition_straight(100, 200);
//  else if (loop_counter > 6000)
//    setPosition_straight(320, 200);
//  else if (loop_counter > 5000)
//    setPosition_straight(200, 60);
//  else if (loop_counter > 3250)
//    setPosition_straight(300, 280);
//  else if (loop_counter > 3000)
//    setPosition_straight(200, 280);
//  else if (loop_counter > 2500)
//    setPosition_straight(200, 60);
//  else if (loop_counter > 1500)
//    setPosition_straight(200, 300);
//  else
//    setPosition_straight(200, 60);
 //==========================================
  String receivedString;
  
  if ( Serial.available() > 0 )
  {
    receivedString = Serial.readStringUntil('\n');
  }
  
  if ( receivedString.equals("1") )
  {
    hBot.SetPosStraight(ROBOT_MAX_X, ROBOT_MAX_Y); // upper right
  }
  else if ( receivedString.equals("2")  )
  {
    hBot.SetPosStraight(ROBOT_MIN_X, ROBOT_MAX_Y); // upper left
  }
  else if ( receivedString.equals("3") )
  {
    hBot.SetPosStraight(ROBOT_MIN_X, ROBOT_MIN_Y); // lower left
  }
  else if ( receivedString.equals("4") )
  {
    hBot.SetPosStraight(ROBOT_MAX_X, ROBOT_MIN_Y); // lower right
  }
}

//================================================================
void InitParams()
{
}
