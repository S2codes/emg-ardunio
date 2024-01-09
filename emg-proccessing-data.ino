
#include "Servo.h";

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
// emg data pin 
#define EMG_PIN_1 A0
#define EMG_PIN_2 A1
#define EMG_PIN_3 A2
#define EMG_PIN_4 A3
#define EMG_PIN_5 A4

// #define INPUT_PIN_2 A1
#define BUFFER_SIZE 128
// servo 
Servo Index; // pin 3
Servo Middle; // pin 4
Servo Ring; // pin 6
Servo Little; // pin 9
Servo Thumb; // pin  10
Servo Wrist; // pin 11
// servo 
int circular_buffer[BUFFER_SIZE];
int data_index, sum;

void setup() {
	// Serial connection begin
	Serial.begin(BAUD_RATE);
  Index.attach(3);
  Middle.attach(5);
  Ring.attach(6);
  Little.attach(9);
  Thumb.attach(10);
  Wrist.attach(11);

// reset 
  allOpen();
  delay(500);

}





void loop() {
	// Calculate elapsed time
	static unsigned long past = 0;
	unsigned long present = micros();
	unsigned long interval = present - past;
	past = present;

	// Run timer
	static long timer = 0;
	timer -= interval;

	// Sample and get envelop
	if(timer < 0) {
		timer += 1000000 / SAMPLE_RATE;
    // emg sensor value 1 
		int sensor_value_1 = analogRead(EMG_PIN_1);
		int signal_1 = EMGFilter(sensor_value_1);
		int envelop_1 = getEnvelop(abs(signal_1));
    // emg sensor value 2 
		int sensor_value_2 = analogRead(EMG_PIN_2);
		int signal_2 = EMGFilter(sensor_value_2);
		int envelop_2 = getEnvelop(abs(signal_2));
    // emg sensor value 3
		int sensor_value_3 = analogRead(EMG_PIN_3);
		int signal_3 = EMGFilter(sensor_value_3);
		int envelop_3 = getEnvelop(abs(signal_3));
    // emg sensor value 4
		int sensor_value_4 = analogRead(EMG_PIN_4);
		int signal_4 = EMGFilter(sensor_value_4);
		int envelop_4 = getEnvelop(abs(signal_4));
    // emg sensor value 5
		int sensor_value_5 = analogRead(EMG_PIN_5);
		int signal_5 = EMGFilter(sensor_value_5);
		int envelop_5 = getEnvelop(abs(signal_5));



		
    
	}


}


// signal identification 

void signalIdentification(int signal_envelop_1, int signal_envelop_2, int signal_envelop_3, int signal_envelop_4, int signal_envelop_5){
  
  // all close 
  if (signal_envelop_1 >= 90 && signal_envelop_1 < 110 &&
      signal_envelop_2 >= 90 && signal_envelop_2 < 110 &&
      signal_envelop_3 >= 90 && signal_envelop_3 < 110 &&
      signal_envelop_4 >= 90 && signal_envelop_4 < 110 &&
      signal_envelop_5 >= 90 && signal_envelop_5 < 110) {
    
    Serial.println("All close");
    allClosed();
    delay(550);
  }
  // all Open
  if (signal_envelop_1 >= 9 && signal_envelop_1 <= 12 &&
      signal_envelop_2 >= 9 && signal_envelop_2 <= 12 &&
      signal_envelop_3 >= 9 && signal_envelop_3 <= 12 &&
      signal_envelop_4 >= 9 && signal_envelop_4 <= 12 &&
      signal_envelop_5 >= 9 && signal_envelop_5 <= 12) {
    
    Serial.println("All Open");
    allClosed();
    delay(550);
  }

  // index close
  if (signal_envelop_1 >= 18 && signal_envelop_1 <= 24 &&
      signal_envelop_2 >= 18 && signal_envelop_2 <= 24 &&
      signal_envelop_3 >= 18 && signal_envelop_3 <= 24 &&
      signal_envelop_4 >= 18 && signal_envelop_4 <= 24 &&
      signal_envelop_5 >= 18 && signal_envelop_5 <= 24) {
    
    Serial.println("Index Close");
    closedIndex();
    delay(550);
  }
  // Middle close
  if (signal_envelop_1 >= 25 && signal_envelop_1 <= 32 &&
      signal_envelop_2 >= 25 && signal_envelop_2 <= 32 &&
      signal_envelop_3 >= 25 && signal_envelop_3 <= 32 &&
      signal_envelop_4 >= 25 && signal_envelop_4 <= 32 &&
      signal_envelop_5 >= 25 && signal_envelop_5 <= 32) {
    
    Serial.println("Index Close");
    closedMiddle();
    delay(550);
  }

  // Ring close
  if (signal_envelop_1 >= 32 && signal_envelop_1 <= 50 &&
      signal_envelop_2 >= 32 && signal_envelop_2 <= 50 &&
      signal_envelop_3 >= 32 && signal_envelop_3 <= 50 &&
      signal_envelop_4 >= 32 && signal_envelop_4 <= 50 &&
      signal_envelop_5 >= 32 && signal_envelop_5 <= 50) {
    
    Serial.println("Ring Close");
    closedRing();
    delay(550);
  }

  // Little close
  if (signal_envelop_1 >= 16 && signal_envelop_1 <= 20 &&
      signal_envelop_2 >= 16 && signal_envelop_2 <= 20 &&
      signal_envelop_3 >= 16 && signal_envelop_3 <= 20 &&
      signal_envelop_4 >= 16 && signal_envelop_4 <= 20 &&
      signal_envelop_5 >= 16 && signal_envelop_5 <= 20) {
    
    Serial.println("Ring Close");
    closedRing();
    delay(550);
  }

}



// Envelop detection algorithm
int getEnvelop(int abs_emg){
	sum -= circular_buffer[data_index];
	sum += abs_emg;
	circular_buffer[data_index] = abs_emg;
	data_index = (data_index + 1) % BUFFER_SIZE;
	return (sum/BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}



// movement funcations
// index close 
void closedIndex() {
  Index.write(5);
  Middle.write(175);
  Ring.write(175);
  Little.write(175);
  Thumb.write(175);
}
// middle close 
void closedMiddle() {
  Index.write(175);
  Middle.write(5);
  Ring.write(175);
  Little.write(175);
  Thumb.write(175);
}

// ring close 
void closedRing() {
  Index.write(175);
  Middle.write(175);
  Ring.write(5);
  Little.write(175);
  Thumb.write(175);
}
// close little 
void closedLittle() {
  Index.write(175);
  Middle.write(175);
  Ring.write(175);
  Little.write(5);
  Thumb.write(175);
}
// close thumb 
void closedThumb() {
  Index.write(175);
  Middle.write(175);
  Ring.write(175);
  Little.write(175);
  Thumb.write(5);
}
// all open 
void allOpen() {
  Index.write(175);
  Middle.write(175);
  Ring.write(175);
  Little.write(175);
  Thumb.write(175);
}
// all closed 
void allClosed() {
  Index.write(5);
  Middle.write(5);
  Ring.write(5);
  Little.write(5);
  Thumb.write(5);
}

// movement funcations











