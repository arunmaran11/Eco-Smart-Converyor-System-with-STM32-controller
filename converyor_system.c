#include "stm32f4xx.h"

// Define GPIO pins for LEDs, relay, buzzer, sensor, and button on Port A
#define LED_GREEN_PIN      (1 << 1)  // PA1 (Green LED)
#define LED_RED_PIN        (1 << 2)  // PA2 (Red LED)
#define RELAY_PIN          (1 << 4)  // PA4 (Relay control)
#define BUZZER_PIN         (1 << 5)  // PA5 (Buzzer control)
#define SENSOR_PIN         (1 << 0)  // PA0 (Sensor input)
#define BUTTON_PIN         (1 << 6)  // PA6 (Button for reset)

#define GPIO_PORT          GPIOA  // All pins on Port A

// Global variables
volatile uint32_t system_time = 0;  // Millisecond counter
uint8_t sensor_count = 0;           // Counter for sensor detections
uint8_t sensor_limit = 10;          // The detection limit (10 times)
uint32_t sensor_debounce_timer = 0; // Timer for holding sensor detection (2 seconds)
uint32_t relay_hold_timer = 0;      // Timer to hold the relay on for 5 seconds
uint32_t buzzer_timer = 0;          // Timer to hold the buzzer on for 5 seconds
uint8_t buzzer_active = 0;          // Flag to track buzzer state
uint32_t button_debounce_timer = 0; // Timer for button debounce (200ms)
uint8_t button_pressed = 0;         // Flag to track button press status

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init(void);
void Delay(uint32_t delay);
void SysTick_Handler(void);  // SysTick interrupt handler
void Control_Output(uint8_t sensor_state);
void Reset_Sensor_Count(void);  // Function to reset sensor count

// Main function
int main(void)
{
    // Initialize the system (use default system clock configuration)
    SystemClock_Config();
    GPIO_Init();

    // Enable SysTick interrupt every 1 millisecond
    SysTick_Config(SystemCoreClock / 1000);  // Set SysTick to interrupt every 1ms

    // Main loop
    while (1)
    {
        // Read the sensor state (high or low)
        uint8_t sensor_state = (GPIOA->IDR & SENSOR_PIN) != 0; // Read PA0

        // Read the button state (active low, normally open button)
        uint8_t button_state = (GPIOA->IDR & BUTTON_PIN) == 0; // Button pressed (low state)

        // If the button is pressed and debounced, reset the sensor count
        if (button_state && !button_pressed && (system_time - button_debounce_timer >= 200))  // Active low, debounce 200ms
        {
            Reset_Sensor_Count();
            button_debounce_timer = system_time;  // Reset the debounce timer
            button_pressed = 1;  // Set button pressed flag
        }
        else if (!button_state)  // If button is released, allow it to be pressed again
        {
            button_pressed = 0;
        }

        // Control the output based on the sensor state (high or low)
        Control_Output(sensor_state);

        // Delay for stabilization (100ms)
        Delay(100);
    }
}

void SystemClock_Config(void)
{
    // No PLL configuration, use the default system clock (HSI)
    RCC->CFGR |= RCC_CFGR_SW_HSI;  // Select HSI as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait until HSI is used
}

void GPIO_Init(void)
{
    // Enable clock for Port A (GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure pins as output (PA1, PA2, PA4, PA5) and input (PA0, PA6)
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 |
                     GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6);  // Clear the bits

    GPIOA->MODER |= (1 << 2) | (1 << 4) | (1 << 8) | (1 << 10);  // Set PA1 (Green LED), PA2 (Red LED), PA4 (Relay), PA5 (Buzzer) as output

    // Configure PA0 (sensor) and PA6 (button) as input
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE6);  
}

void SysTick_Handler(void)
{
    // Increment system time every 1 millisecond
    system_time++;
}

void Delay(uint32_t delay)
{
    uint32_t start_time = system_time;
    while ((system_time - start_time) < delay)
    {
        __NOP();  // No operation, just wait
    }
}

void Control_Output(uint8_t sensor_state)
{
    // If the sensor is detecting an object (active high) and no debounce period is active
    if (sensor_state && (system_time - sensor_debounce_timer >= 1000))  // 1000ms = 1 second debounce
    {
        // If the count is less than the limit, increase the count
        if (sensor_count < sensor_limit)
        {
            sensor_count++;  // Increment sensor count
            sensor_debounce_timer = system_time;  // Set debounce timer to prevent further detection for 1 second
        }

        // If the sensor count is less than 10, turn on the green LED, turn off the red LED, and activate relay
        if (sensor_count < sensor_limit)
        {
            GPIOA->ODR |= LED_GREEN_PIN;  // Turn on Green LED (PA1)
            GPIOA->ODR &= ~LED_RED_PIN;   // Turn off Red LED (PA2)

            // Start the 5-second loop (Relay is turned on during this 5-second period)
            GPIOA->ODR |= RELAY_PIN;  // Turn on Relay (PA4)

            // Wait for 5 seconds
            uint32_t start_time = system_time;
            while (system_time - start_time < 5000)  // 5000ms = 5 seconds
            {
                __NOP();  // No operation, just wait for 5 seconds
            }

            // After 5 seconds, turn off the relay
            GPIOA->ODR &= ~RELAY_PIN;  // Turn off Relay (PA4)
        }
    }
    else
    {
        // If the sensor is not detecting (inactive) or count >= 10, handle accordingly
        if (sensor_count >= sensor_limit)
        {
            // When the sensor count reaches or exceeds 10:
            // Turn off Green LED, turn on Red LED, and deactivate relay
            GPIOA->ODR &= ~LED_GREEN_PIN;  // Turn off Green LED (PA1)
            GPIOA->ODR |= LED_RED_PIN;     // Turn on Red LED (PA2)

            // Ensure relay is off when sensor count reaches 10 or more
            GPIOA->ODR &= ~RELAY_PIN;  // Turn off Relay (PA4)

            // Start buzzer when sensor count >= 10, and hold it on for 5 seconds
            if (!buzzer_active)
            {
                buzzer_timer = system_time;  // Set buzzer on-time
                GPIOA->ODR |= BUZZER_PIN;    // Turn on Buzzer (PA5)
                buzzer_active = 1;           // Set buzzer as active
            }
        }
        else
        {
            // If sensor count is less than 10, make sure green LED is on and red LED is off
            GPIOA->ODR |= LED_GREEN_PIN;  // Turn on Green LED (PA1)
            GPIOA->ODR &= ~LED_RED_PIN;   // Turn off Red LED (PA2)

            // Ensure relay is off when sensor is not active
            GPIOA->ODR &= ~RELAY_PIN;  // Turn off Relay (PA4)

            // Ensure buzzer is off when sensor is not detecting and count < 10
            GPIOA->ODR &= ~BUZZER_PIN; // Turn off Buzzer (PA5)
            buzzer_active = 0;         // Reset buzzer active state
        }
    }

    // Check if 5 seconds have passed since buzzer was triggered, and turn off the buzzer
    if (buzzer_active && system_time - buzzer_timer >= 500)  // 5000ms = 5 seconds
    {
        GPIOA->ODR &= ~BUZZER_PIN; // Turn off Buzzer (PA5)
        buzzer_active = 0;         // Reset buzzer active state
    }
}

// Reset the sensor count to 0
void Reset_Sensor_Count(void)
{
    sensor_count = 0;  // Reset sensor count to 0
    GPIOA->ODR &= ~LED_GREEN_PIN;  // Ensure Green LED is off
    GPIOA->ODR &= ~LED_RED_PIN;    // Ensure Red LED is off
    GPIOA->ODR &= ~RELAY_PIN;      // Ensure Relay is off
    GPIOA->ODR &= ~BUZZER_PIN;     // Ensure Buzzer is off
}
