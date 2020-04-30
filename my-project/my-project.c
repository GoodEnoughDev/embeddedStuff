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
#include "semphr.h"

#define COMMAND_QUEUE_LEN 20
#define COMMAND_BUFFER_LEN 50

xQueueHandle command_queue;
xSemaphoreHandle print_semaphore;

void testTask(void);
void heap_monitor_task(void);

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
	xSemaphoreTake(print_semaphore, portMAX_DELAY);
	printf("Starting testTask\r\n");
	xSemaphoreGive(print_semaphore);
	const TickType_t delay = pdMS_TO_TICKS(25);
	while(1)
	{
			vTaskDelay(delay);
	}
}

void heap_monitor_task(void)
{
	xSemaphoreTake(print_semaphore, portMAX_DELAY);
    printf("Started heap monitor task\r\n");
	xSemaphoreGive(print_semaphore);
    uint32_t current_heap_size;
    const TickType_t delay = pdMS_TO_TICKS(500);

    while(1)
    {
        current_heap_size = xPortGetFreeHeapSize();
        if(current_heap_size > (configTOTAL_HEAP_SIZE * 0.9))
        {
            printf("Current heap size: %d near max: %d", current_heap_size, configTOTAL_HEAP_SIZE);
        }

        vTaskDelay(delay);
	}
}

void command_parser_task(void)
{
	xSemaphoreTake(print_semaphore, portMAX_DELAY);
    printf("Started command parser task\r\n");
	xSemaphoreGive(print_semaphore);
	char character;
	char command_buffer[COMMAND_BUFFER_LEN];
	int index = 0;
	const TickType_t delay = pdMS_TO_TICKS(25);

	while(1)
	{
		xQueueReceive(command_queue, &character, portMAX_DELAY);
		if(character != NULL)
		{
			if(character == '\n' || character == '\r')
			{
				for(int i = 0; i <= index; i++)
				{
					usart_send_blocking(USART3, command_buffer[i]);
					command_buffer[i] = 0;
					//vTaskDelay(delay);
				}
				index = 0;
				printf("\r\n");
			}
			command_buffer[index] = character;
			gpio_toggle(GPIOB, GPIO0);
			index++;
		}
	}
}

int main(void) 
{
	uint32_t rc = 0;
	print_semaphore = xSemaphoreCreateMutex();
	clock_setup();
	gpio_setup();
	usart_setup();
	printf("Hardware setup complete\r\n");

	rc = xTaskCreate(heap_monitor_task, "heap_monitor_task", 100, NULL,configMAX_PRIORITIES-1, NULL);
	printf("Task creation code: %d\r\n", rc);

	rc = xTaskCreate(testTask, "testTask", 100, NULL, configMAX_PRIORITIES-1, NULL);
	printf("Task creation code: %d\r\n", rc);

	// Create command queue
	command_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(char));	
	printf("Command queue created with handle: %d\r\n");

	// Create command parser task
	xTaskCreate(command_parser_task, "command_parser_task", 200, NULL, configMAX_PRIORITIES-1, NULL);

	vTaskStartScheduler();
	printf("vTaskStartScheduler returned");
	return 0;
}

void usart3_isr(void)
{
	BaseType_t xHighPriorityTaskWoken;
	xHighPriorityTaskWoken = pdFALSE;
	static char character;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) 
	{
		/* Indicate that we got data. */
		//gpio_toggle(GPIOB, GPIO0);

		/* Retrieve the data from the peripheral. */
		character = usart_recv(USART3);
		
		xQueueSendToBack(command_queue, &character, xHighPriorityTaskWoken);
	}
}
