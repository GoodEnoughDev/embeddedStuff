#include "api.h"
#include "api-asm.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"

void _putchar(char character)
{
	usart_send_blocking(USART3, character);
}

static void clock_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART3);
}

static void usart_setup(void)
{
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	usart_enable(USART3);
}

static void gpio_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8);
}

static void testTask(void *args)
{
	const TickType_t delay = pdMS_TO_TICKS(250);
	while(1)
	{
			gpio_toggle(GPIOB, GPIO0);
			vTaskDelay(delay);
	}
}

int main(void) 
{
	uint32_t rc = 0;
	clock_setup();
	gpio_setup();
	usart_setup();
	printf("Hardware setup complete\r\n");
	rc = xTaskCreate(testTask, "testTask", 100, NULL, configMAX_PRIORITIES-1, NULL);
	printf("Task creation code: %d\r\n", rc);
	gpio_toggle(GPIOB, GPIO0);
	vTaskStartScheduler();
	gpio_toggle(GPIOB, GPIO0);
	

	/* add your own code */
	uint32_t rev = 0xaabbccdd;
	rev = rev_bytes(rev);
	return my_func(rev);
}
