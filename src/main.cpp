#include "mbed.h"
#include "TS_DISCO_F429ZI.h"
#include "LCD_DISCO_F429ZI.h"
#include <iostream>
#include <vector>
#include <cmath>

// Group 35 ECE 6483 Final Project
// Michael Lee, Nashara Marrow

LCD_DISCO_F429ZI lcd;
TS_DISCO_F429ZI ts;

InterruptIn button(BUTTON1);
DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

Timer debounce_timer, record_timer;
vector<array<float, 3>> key;
vector<array<float, 3>> unlocktemp;
// Define control register addresses and their configurations
#define CTRL_REG1 0x20                  // Address of Control Register 1
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1 // Configuration for enabling gyroscope and setting data rate
#define CTRL_REG4 0x23                  // Address of Control Register 4
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 // Configuration for setting full-scale range
#define CTRL_REG3 0x22                  // Address of Control Register 3
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000 // Enable data-ready interrupt

// Define a flag for SPI communication completion
#define SPI_FLAG 1                      // Event flag for SPI transfer completion

// Define the Data Ready Flag
#define DATA_READY_FLAG 2               // Event flag for data-ready interrupt

// Define the address to read the X-axis lower data
#define OUT_X_L 0x28                    // Address of the gyroscope's X-axis lower byte data register

// Declare an EventFlags object for handling asynchronous events
EventFlags flags;
#define RECORD 4
#define UNLOCK 8
#define KEYCLEAR 16

// Window Size for Moving Average Window
#define WINDOW_SIZE 10


// Callback function to be called upon SPI transfer completion
void spi_cb(int event)
{
    // Set the SPI_FLAG to signal the main thread
    flags.set(SPI_FLAG);
}

// Callback function for Data Ready Interrupt
void data_cb() {
    flags.set(DATA_READY_FLAG);
}



// Define a scaling factor for converting raw sensor data to actual angular velocity
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// Variable Definitions for Filters
    uint16_t raw_gx, raw_gy, raw_gz; // raw gyro values
    float gx, gy, gz; // converted gyro values
    // float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f; //lpf filtered values    
    // float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f; //hpf filtered values

// Moving Average Filter Buffer --> only to be used with the Moving Average Filter!
float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
int window_index = 0;

// Checks if the user touched the button
bool button_touch(int x, int y, int button_x, int button_y, int button_width, int button_height)
{
    return (x >= button_x && x <= button_x + button_width && y >= button_y && y <= button_y + button_height);
}

void TouchScreen();
void gyroscope();

void unlockedscreen();
void lockedscreen();

float calculate_mean(const vector<float> &vector);
float correlation(const vector<float> &a, const vector<float> &b);
array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2);

// Function to calculate the mean of a vector
float calculate_mean(const vector<float> &vector) {
    float sum = 0.0;
    for (const auto &value : vector) {
        sum += value;
    }
    return sum / vector.size();
}

// Function to calculate the correlation coefficient
float correlation(const vector<float> &a, const vector<float> &b) {

    float mean_a = calculate_mean(a);
    float mean_b = calculate_mean(b);

    float numerator = 0.0;
    float denominator_a = 0.0;
    float denominator_b = 0.0;

    for (size_t i = 0; i < a.size(); ++i) {
        float diff_a = a[i] - mean_a;
        float diff_b = b[i] - mean_b;

        numerator += diff_a * diff_b;
        denominator_a += diff_a * diff_a;
        denominator_b += diff_b * diff_b;
    }

    float denominator = sqrt(denominator_a * denominator_b);

    if (denominator == 0.0f) {
        return 0.0f; // To handle division by zero
    }

    return numerator / denominator;
}

array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2) {
    array<float, 3> result;
    for (int i = 0; i < 3; i++) {
        vector<float> a;
        vector<float> b;
        for (const auto& arr : vec1) {
            a.push_back(arr[i]);
        }
        for (const auto& arr : vec2) {
            b.push_back(arr[i]);
        }
        if (a.size() > b.size()) {
            a.resize(b.size(), 0);
        } else if (b.size() > a.size()) {
            b.resize(a.size(), 0);
        }

        result[i] = correlation(a, b);
    }
    return result;
}

int main()
{
    // Touchscreen initialization
    uint8_t status = ts.Init(lcd.GetXSize(), lcd.GetYSize());
    if (status != TS_OK)
    {
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"FAILED", CENTER_MODE);
    }
    else
    {
      lcd.Clear(LCD_COLOR_GREEN);
      lcd.SetBackColor(LCD_COLOR_GREEN);
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"SUCCESS", CENTER_MODE);
    }
    thread_sleep_for(1000);
    
    unlockedscreen();

    Thread gyro;
    gyro.start(callback(gyroscope));

    Thread touch;
    touch.start(callback(TouchScreen));

    while (1)
    {
        thread_sleep_for(100);
    }
}

void unlockedscreen()
{
// When no key is recorded yet or has been deleted (unlocked mode)
    red_led = 0;
    green_led = 1;
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTMAGENTA);
    lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Embedded Sentry", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"UNLOCKED", CENTER_MODE);
    lcd.DrawRect(45, 125, 140, 50);
    lcd.FillRect(45, 125, 140, 50);
    lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RECORD", CENTER_MODE);
}

void lockedscreen()
{
    // When a recorded key exists (locked mode)
    red_led = 1;
    green_led = 0;
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTMAGENTA);
    lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Embedded Sentry", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"LOCKED", CENTER_MODE);
    lcd.DrawRect(45, 125, 140, 50);
    lcd.FillRect(45, 125, 140, 50);
    lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"UNLOCK", CENTER_MODE);
}


void TouchScreen()
{
    uint16_t x,y;

    while (1){

        TS_StateTypeDef TS_State;
        ts.GetState(&TS_State);      
        if (TS_State.TouchDetected)
        {
            x = TS_State.X;
            y = TS_State.Y;

            // User wants to record
            if(key.empty() && button_touch(x, y, 45, 125, 140, 50)) 
            {
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.SetBackColor(LCD_COLOR_BLACK);
                lcd.SetTextColor(LCD_COLOR_WHITE);
                lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"Record Starting...", CENTER_MODE);
                thread_sleep_for(1000);
                flags.set(RECORD);
            }
            // User wants to unlock
            else if(button_touch(x, y, 45, 125, 140, 50))
            {   
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.SetTextColor(LCD_COLOR_WHITE);
                lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"Unlocking...", CENTER_MODE);
                thread_sleep_for(1000);
                flags.set(UNLOCK);
            }
        } 
    }
}



void gyroscope()
{
    //Gyroscope Initialization
    // Initialize the SPI object with specific pins
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // SPI pins: MOSI, MISO, SCK, and Slave Select

    // Buffers for sending and receiving data over SPI
    uint8_t write_buf[32], read_buf[32];

    // Configure the interrupt pin for the data-ready signal
    InterruptIn int2(PA_2, PullDown);   // Initialize INT2 pin with pull-down resistor
    int2.rise(&data_cb);                // Attach the data-ready callback to the rising edge of INT2

    // Configure SPI format and frequency
    spi.format(8, 3);                   // 8-bit data size, SPI mode 3
    spi.frequency(1'000'000);           // SPI clock frequency set to 1 MHz

    // Configure CTRL_REG1 to enable gyroscope and set data rate
    write_buf[0] = CTRL_REG1;           // Register address
    write_buf[1] = CTRL_REG1_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion

    // Configure CTRL_REG4 to set full-scale range
    write_buf[0] = CTRL_REG4;           // Register address
    write_buf[1] = CTRL_REG4_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion

    // Configure CTRL_REG3 to enable data-ready interrupt
    write_buf[0] = CTRL_REG3;           // Register address
    write_buf[1] = CTRL_REG3_CONFIG;    // Configuration value

    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Perform SPI transfer
    flags.wait_all(SPI_FLAG);           // Wait for SPI transfer completion

    // Dummy value to reset the write buffer
    write_buf[1] = 0xFF;


    while(1){
        vector<array<float, 3>> tempkey;
        int flag = flags.wait_any(RECORD | UNLOCK);
        // if(flag & KEYCLEAR) 
        // {
        //     lcd.Clear(LCD_COLOR_BLACK);
        //     lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RESETTING KEY", CENTER_MODE);
        //     key.clear();
        //     unlocktemp.clear();
        //     lcd.Clear(LCD_COLOR_BLACK);
        //     lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"KEY RESET.", CENTER_MODE);
        //     thread_sleep_for(1500);
        //     unlockedscreen();
        // }
        
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"CALIBRATING", CENTER_MODE);

        lcd.Clear(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RECORDING IN 3", CENTER_MODE);
        thread_sleep_for(1000);
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RECORDING IN 2", CENTER_MODE);
        thread_sleep_for(1000);
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RECORDING IN 1", CENTER_MODE);
        thread_sleep_for(1000);
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"RECORDING...", CENTER_MODE);

        record_timer.start();
        while (record_timer.elapsed_time() < 5s)
        {
                
            //Wait for the data-ready interrupt flag to be set
            flags.wait_all(DATA_READY_FLAG);

            // Read GYRO Data using SPI transfer --> 6 bytes!
            write_buf[0] = OUT_X_L | 0x80 | 0x40;
            spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
            flags.wait_all(SPI_FLAG);

            // Extract raw 16-bit gyroscope data for X, Y, Z
            raw_gx = (read_buf[2] << 8) | read_buf[1];
            raw_gy = (read_buf[4] << 8) | read_buf[3];
            raw_gz = (read_buf[6] << 8) | read_buf[5];

            // Convert raw data to radians per second!
            gx = raw_gx * SCALING_FACTOR;
            gy = raw_gy * SCALING_FACTOR;
            gz = raw_gz * SCALING_FACTOR;

            window_gx[window_index] = gx;
            window_gy[window_index] = gy;
            window_gz[window_index] = gz;

            float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;

            for (int i = 0; i < WINDOW_SIZE; i++) {
                avg_gx += window_gx[i];
                avg_gy += window_gy[i];
                avg_gz += window_gz[i];
            }
            avg_gx /= WINDOW_SIZE;
            avg_gy /= WINDOW_SIZE;
            avg_gz /= WINDOW_SIZE;
            window_index = (window_index + 1) % WINDOW_SIZE;

            tempkey.push_back({avg_gx, avg_gy, avg_gz}); // Record key

            printf("Moving Average -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", avg_gx, avg_gy, avg_gz);
            printf(">Moving Average X axis-> gx: %4.5f|g\n", avg_gx);
            printf(">Moving Average Y axis-> gy: %4.5f|g\n", avg_gy);
            printf(">Moving Average Z axis-> gz: %4.5f|g\n", avg_gz);
        }
        record_timer.stop();  
        record_timer.reset(); 

        if(flag & RECORD) // Recording mode
        {
            flags.clear(RECORD);
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"SAVING KEY...", CENTER_MODE);
            key = tempkey; // Save key
            tempkey.clear();

            lcd.Clear(LCD_COLOR_BLACK);
            lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"KEY SAVED", CENTER_MODE);
            thread_sleep_for(1500);
            lockedscreen();
        }

        else if (flag & UNLOCK) // Unlocking mode
        {
            flags.clear(UNLOCK);
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"UNLOCKING...", CENTER_MODE);

            int count = 0; 
            array<float, 3> correlationResult = calculateCorrelationVectors(key, tempkey);

            printf("Correlation values: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
            for (size_t i = 0; i < correlationResult.size(); i++)
            {
                if (correlationResult[i] > 0.1f)
                {
                    count++;
                }
            }
            tempkey.clear();
            if(count == 3)
            {
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"UNLOCKED", CENTER_MODE);
                thread_sleep_for(1500);
                unlockedscreen();
                key.clear();
            }
            else
            {
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"UNLOCK FAILED", CENTER_MODE);
                thread_sleep_for(1500);
                lockedscreen();
            }
        }
    }
}




























