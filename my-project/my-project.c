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
#define COMMAND_BUILDER_QUEUE_LEN 20
#define CHARACTER_QUEUE_LEN 50

// Queues
xQueueHandle command_builder_queue;
xQueueHandle character_queue;
xQueueHandle command_queue;

// Semaphores
xSemaphoreHandle USART_semaphore;

void testTask(void);
void heap_monitor_task(void);
void command_parser_task(void);


void _putchar(char character)
{
	xSemaphoreTake(USART_semaphore, portMAX_DELAY);
	usart_send_blocking(USART3, character);
	xSemaphoreGive(USART_semaphore);
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
	usart_enable(USART3);
	usart_enable_rx_interrupt(USART3);
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

void heap_monitor_task(void)
{
    printf("Started heap monitor task\r\n");
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

void command_builder_task(void)
{
    printf("Started command parser task\r\n");
	char character;
	char command_buffer[COMMAND_BUILDER_QUEUE_LEN];
	int index = 0;
	const TickType_t delay = pdMS_TO_TICKS(25);

	while(1)
	{
		xQueueReceive(character_queue, &character, portMAX_DELAY);
		if(character != NULL)
		{
			if(character == '\n' || character == '\r')
			{
				for(int i = 0; i <= index; i++)
				{
					_putchar(command_buffer[i]);
					command_buffer[i] = NULL;
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
	USART_semaphore = xSemaphoreCreateMutex();

	clock_setup();
	gpio_setup();
	usart_setup();
	printf("Hardware initialized\r\n");

	character_queue = xQueueCreate(CHARACTER_QUEUE_LEN, sizeof(char));	
	command_builder_queue = xQueueCreate(COMMAND_BUILDER_QUEUE_LEN, sizeof(char));	

	// Create tasks
	xTaskCreate(heap_monitor_task, "heap_monitor_task", 100, NULL,configMAX_PRIORITIES-1, NULL);
	xTaskCreate(command_builder_task, "command_builder_task", 200, NULL, configMAX_PRIORITIES-1, NULL);
	gpio_toggle(GPIOB, GPIO0);

	vTaskStartScheduler();
	printf("vTaskStartScheduler returned");
	
	
	return 0;
}

void usart3_isr(void)
{
	static char character;
	gpio_toggle(GPIOB, GPIO0);

	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) 
	{
		character = usart_recv(USART3);
		xQueueSendToBackFromISR(character_queue, &character, pdFALSE);
	}
}
