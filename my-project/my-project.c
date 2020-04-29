#include "api.h"
#include "api-asm.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "queue.h"


#define COMMAND_BUFFER_LEN 100
#define COMMAND_QUEUE_LEN 30

char command_buffer[COMMAND_BUFFER_LEN];
xQueueHandle command_queue;

void testTask(void);

void _putchar(char character)
{
	usart_send_blocking(USART3, character);
}

static void clock_setup(void)
{
	// Setup system clock
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	// Setup USART peripheral clocks
	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_GPIOD);

	// Setup LED GPIO clock	
	rcc_periph_clock_enable(RCC_GPIOB);
}

static void usart_setup(void)
{
	// Setup interrupts
	nvic_enable_irq(NVIC_USART3_IRQ);

	// Setup USART
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_enable_rx_interrupt(USART3);
	usart_enable(USART3);
	printf("\r\n");
}

static void gpio_setup(void)
{
	// Setup LED GPIO
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);

	// Setup USART pin modes
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO9);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO9);
}

void testTask(void)
{
	printf("Starting testTask");
	const TickType_t delay = pdMS_TO_TICKS(25);
	while(1)
	{
			//gpio_toggle(GPIOB, GPIO0);
			vTaskDelay(delay);
	}
}

void heap_monitor_task()
{
    printf("Started heap monitor task");
    uint32_t current_heap_size;
    const TickType_t delay = pdMS_TO_TICKS(100);

    while(1)
    {
        current_heap_size = xPortGetFreeHeapSize();
        if(current_heap_size > configMAX_HEAP_SIZE)
        {
            printf("Current heap size: %d near max: %d", current_heap_size, configMAX_HEAP_SIZE)
        }

        vTaskDelay(delay);
    {
}

int main(void) 
{
	uint32_t rc = 0;
	clock_setup();
	gpio_setup();
	usart_setup();
	printf("Hardware setup complete\r\n");

	rc = xTaskCreate(heap_monitor_task, "heap_monitor_task", 100, NULL, 1, NULL);
	printf("Task creation code: %d\r\n", rc);

	rc = xTaskCreate(testTask, "testTask", 100, NULL, 1, NULL);
	printf("Task creation code: %d\r\n", rc);

	// Create command queue
	command_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(char));	
	printf("Command queue created with handle: %d\r\n");

	vTaskStartScheduler();
	printf("vTaskStartScheduler returned");
	return 0;
}

void usart3_isr(void)
{
	static int index = 0;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOB, GPIO0);

		/* Retrieve the data from the peripheral. */
		char character = usart_recv(USART3);
		command_buffer[index++] = character;
		if(character == '\n')
		{
			index = 0;
			xQueueSendToBackFromISR(command_queue, &command_buffer, pdFALSE);
		}

	}
}
