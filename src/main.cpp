#include <mbed.h>
#include <chrono>
#include <numeric>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h"

using namespace std::chrono;

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2, PullDown);
InterruptIn userButton(PA_0);
DigitalOut recordLed(PG_13);
DigitalOut unlockLed(PG_14);

#define SEQUENCE_LENGTH 40         // The length of the unlock sequence.
#define RECORDING_INTERVAL_MS 2000 // The duration of the recording period in milliseconds.
#define TOLERANCE 1000             // The tolerance when comparing the recorded and attempted unlock sequences.

#define OUT_X_L 0x28
#define OUT_Y_L 0x2A
#define OUT_Z_L 0x2C
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00010000
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b00001000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define BUTTON_PRESS_FLAG 4

#define NUM_AXES 3 // Number of axes to compare (X, Y, and Z).

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;
int16_t recordedSequence[NUM_AXES][SEQUENCE_LENGTH];
int sequenceIndex = 0;
bool recording = false;
bool hasRecordedSequence = false;

LCD_DISCO_F429ZI lcd; // Initialize the LCD display

void spi_cb(int event)
{
  flags.set(SPI_FLAG);
};

void data_cb()
{
  flags.set(DATA_READY_FLAG);
};

void buttonPressed_cb()
{
  if (!recording)
  {
    recording = true;
    recordLed = recording;
    sequenceIndex = 0; // Reset sequence index before recording
  }
}

void init_spi()
{
  spi.format(8, 3);
  spi.frequency(1'000'000);

  write_buf[0] = CTRL_REG1;
  write_buf[1] = CTRL_REG1_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  write_buf[0] = CTRL_REG4;
  write_buf[1] = CTRL_REG4_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  int2.rise(&data_cb);
  userButton.rise(callback(buttonPressed_cb));

  write_buf[0] = CTRL_REG3;
  write_buf[1] = CTRL_REG3_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))
  {
    flags.set(DATA_READY_FLAG);
  }
}

void read_gyro_data(int16_t *data)
{
  flags.wait_all(DATA_READY_FLAG);
  write_buf[0] = OUT_X_L | 0x80 | 0x40;
  spi.transfer(write_buf, 9, read_buf, 10, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);
  data[0] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
  data[1] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
  data[2] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
}

double mean(int16_t data[], int size)
{
  double sum = std::accumulate(data, data + size, 0.0);
  return sum / size;
}

double stddev(int16_t data[], int size, double mean)
{
  double sum = 0.0;
  for (int i = 0; i < size; ++i)
    sum += (data[i] - mean) * (data[i] - mean);
  return std::sqrt(sum / size);
}

bool sequenceMatch(int16_t seq1[][SEQUENCE_LENGTH], int16_t seq2[][SEQUENCE_LENGTH])
{
  for (int axis = 0; axis < NUM_AXES; axis++)
  {
    double mean1 = mean(seq1[axis], SEQUENCE_LENGTH);
    double mean2 = mean(seq2[axis], SEQUENCE_LENGTH);
    double stddev1 = stddev(seq1[axis], SEQUENCE_LENGTH, mean1);
    double stddev2 = stddev(seq2[axis], SEQUENCE_LENGTH, mean2);

    if (std::abs(mean1 - mean2) > TOLERANCE || std::abs(stddev1 - stddev2) > TOLERANCE)
    {
      return false;
    }
  }
  return true;
}

int main()
{
  init_spi();
  Timer recordingTimer;

  // Clear the LCD display and set the text background color
  lcd.Clear(LCD_COLOR_WHITE);
  lcd.SetBackColor(LCD_COLOR_WHITE);
  lcd.SetTextColor(LCD_COLOR_BLACK);

  // Draw a border around the LCD screen
  lcd.DrawRect(10, 10, lcd.GetXSize() - 20, lcd.GetYSize() - 20);
  while (1)
  {
    if (recording)
    {
      printf("Started recording.\n");
      recordingTimer.start();

      while (recordingTimer.elapsed_time() < milliseconds(RECORDING_INTERVAL_MS) && sequenceIndex < SEQUENCE_LENGTH)
      {
        int16_t data[NUM_AXES];
        read_gyro_data(data);
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
          recordedSequence[axis][sequenceIndex] = data[axis];
        }
        sequenceIndex++;

        ThisThread::sleep_for(50ms); // Sleep for a while to not overwhelm the sensor
      }

      recordingTimer.stop();
      recordingTimer.reset();

      printf("Stopped recording.\n");

      recording = false;
      recordLed = recording;
      hasRecordedSequence = true; // Set the flag to true after recording
    }
    else if (hasRecordedSequence)
    {
      // Matching mode
      int16_t currentSequence[NUM_AXES][SEQUENCE_LENGTH];

      // Store the first set of recorded values
      char buffer_r[NUM_AXES][6];
      for (int axis = 0; axis < NUM_AXES; axis++)
      {
        sprintf(buffer_r[axis], "%d", recordedSequence[axis][0]);
      }

      for (int i = 0; i < SEQUENCE_LENGTH; i++)
      {
        int16_t data[NUM_AXES];
        read_gyro_data(data);
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
          currentSequence[axis][i] = data[axis];
        }

        // Display the current X,Y,Z coordinates on the LCD screen
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"X: ", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Y: ", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Z: ", CENTER_MODE);

        char buffer[6];
        sprintf(buffer, "%d", data[0]);
        lcd.DisplayStringAt(50, LINE(3), (uint8_t *)buffer, CENTER_MODE);

        sprintf(buffer, "%d", data[1]);
        lcd.DisplayStringAt(50, LINE(4), (uint8_t *)buffer, CENTER_MODE);

        sprintf(buffer, "%d", data[2]);
        lcd.DisplayStringAt(50, LINE(5), (uint8_t *)buffer, CENTER_MODE);

        // Display the recorded X,Y,Z coordinates on the LCD screen
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"X_r: ", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Y_r: ", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Z_r: ", CENTER_MODE);

        lcd.DisplayStringAt(60, LINE(9), (uint8_t *)buffer_r[0], CENTER_MODE);
        lcd.DisplayStringAt(60, LINE(10), (uint8_t *)buffer_r[1], CENTER_MODE);
        lcd.DisplayStringAt(60, LINE(11), (uint8_t *)buffer_r[2], CENTER_MODE);

        ThisThread::sleep_for(50ms); // Sleep for a while to not overwhelm the sensor
      }

      if (sequenceMatch(recordedSequence, currentSequence))
      {
        printf("Sequence matched!\n");
        unlockLed = 1;

        // Display the sequence matched message
        lcd.Clear(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Sequence matched!", CENTER_MODE);
        ThisThread::sleep_for(2s);
        lcd.Clear(LCD_COLOR_WHITE); // Clear the message

        unlockLed = 0;
      }
    }
  }
}
