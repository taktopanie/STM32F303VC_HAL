/*
 * HD44780.c
 *
 *  Created on: Oct 25, 2023
 *      Author: maciej
 */


#include "HD44780.h"

void Set_RS(void)
{
	LCD_RS_PORT->ODR |= (LCD_RS_PIN);
}

void Set_RW (void)
{
	LCD_RW_PORT->ODR |= (LCD_RW_PIN);
}

void Set_E (void)
{
	LCD_E_PORT->ODR |= (LCD_E_PIN);
}

void Reset_RS (void)
{
	LCD_RS_PORT->ODR &= ~(LCD_RS_PIN);
}
void Reset_RW (void)
{
	LCD_RW_PORT->ODR &= ~(LCD_RW_PIN);
}
void Reset_E (void)
{
	LCD_E_PORT->ODR &= ~(LCD_E_PIN);
}

//special charaters
uint8_t shapes [2][8] = {
		{0b00000100, 0b00010101, 0b00001110, 0b00000100, 0b00001110, 0b00010101, 0b00000100, 0b00000000},
		{0b00000100, 0b00000100, 0b0001110, 0b00001110, 0b00001110, 0b00001110, 0b00011111, 0b00011111}
};



/*
 * function lcd_init
 */

void lcd_init(void){

	Set_RS();
	Set_RW();
	Set_E();
	// wyzerowanie pinow RS,RW,E
	Reset_RS();
	Reset_RW();
	Reset_E();
	HAL_Delay(50);

	lcd_sendHalf(0b0011);
	HAL_Delay(5);
	lcd_sendHalf(0b0011);
	HAL_Delay(100);
	lcd_sendHalf(0b0011);
	HAL_Delay(100);
	lcd_sendHalf(0b0010);


	// ustawienie 4bitow 2 wierszy i rozdzielczosci znakow
	lcd_send_command(FUNCTION_SET|DATA_LENGHT_4BIT|NUMBER_OF_LINES_2|CHARACTER_FONT_5x8);
	HAL_Delay(100);
	// wylaczenie wyswietlacza
	lcd_send_command(LCD_ONOFF|LCD_DISPLAY_ON|LCD_BLINKINGCOURSOR_ON);
	HAL_Delay(100);
	//wyczysczenie wyswietlacza
	lcd_send_command(LCD_CLEAR);
	HAL_Delay(100);
	// entry mode
	lcd_send_command(ENTRY_MODE|INCREMENT);
	HAL_Delay(100);
}

void _save_state(uint8_t * saved_state){
	// SAVE PREVIOUS STATE //
	if(HAL_GPIO_ReadPin(LCD_RS_PORT, LCD_RS_PIN)){
		*saved_state |= (1 << 0);
	}else{
		*saved_state &= ~(1 << 0);
	}

	if(HAL_GPIO_ReadPin(LCD_RW_PORT, LCD_RW_PIN)){
		*saved_state |= (1 << 1);
	}else{
		*saved_state &= ~(1 << 1);
	}
}

void _return_state(uint8_t * saved_state){
	//return to previous state
	if(*saved_state & (1 << 0)){
		HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, ENABLE);
	}else{
		HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, DISABLE);
	}

	if(*saved_state & (1 << 1)){
		HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, ENABLE);
	}else{
		HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, DISABLE);
	}
}

void lcd_clear(void){
	lcd_send_command(LCD_CLEAR);
}

/*
 * function lcd_send_half
 */

void lcd_sendHalf(uint8_t data){
	if(data&(1<<0)){
		HAL_GPIO_WritePin(LCD_DATA_4_PORT, LCD_DATA_4_PIN, ENABLE);
		}else{HAL_GPIO_WritePin(LCD_DATA_4_PORT, LCD_DATA_4_PIN, DISABLE);}

	if(data&(1<<1)){
		HAL_GPIO_WritePin(LCD_DATA_5_PORT, LCD_DATA_5_PIN, ENABLE);
		}else{HAL_GPIO_WritePin(LCD_DATA_5_PORT, LCD_DATA_5_PIN, DISABLE);}

	if(data&(1<<2)){
		HAL_GPIO_WritePin(LCD_DATA_6_PORT, LCD_DATA_6_PIN, ENABLE);
		}else{HAL_GPIO_WritePin(LCD_DATA_6_PORT, LCD_DATA_6_PIN, DISABLE);}

	if(data&(1<<3)){
		HAL_GPIO_WritePin(LCD_DATA_7_PORT, LCD_DATA_7_PIN, ENABLE);
		}else{HAL_GPIO_WritePin(LCD_DATA_7_PORT, LCD_DATA_7_PIN, DISABLE);}
}

/*
 * function lcd_send_byte
 */

void lcd_send_byte(uint8_t byte)
{
	Set_E();
	lcd_sendHalf(byte>>4);
	HAL_Delay(2);
	Reset_E();

	Set_E();
	lcd_sendHalf(byte);
	HAL_Delay(2);
	Reset_E();

}

/*
 * function lcd_send_command
 */

void lcd_send_command(unsigned char command){
	uint8_t saved_state = 0;
	_save_state(&saved_state);

	Reset_RS();
	lcd_send_byte(command);

	_return_state(&saved_state);
}

/*
 * function lcd_send_data
 */

void lcd_send_data(unsigned char data)
{
	uint8_t saved_state = 0;
	_save_state(&saved_state);

	Set_RS();
	lcd_send_byte(data);

	_return_state(&saved_state);
}

/*
 * function lcd_send_text
 */

void lcd_send_text(char *text){
	while(*text){
		lcd_send_data(*text);
		text++;
	}
}

uint8_t lcd_readHalf(void){
	uint8_t data = 0;
	if(HAL_GPIO_ReadPin(LCD_DATA_4_PORT, LCD_DATA_4_PIN)){
		data |= (1<<0);
		}else{data &= ~(1<<0);}

	if(HAL_GPIO_ReadPin(LCD_DATA_5_PORT, LCD_DATA_5_PIN)){
		data |= (1<<1);
		}else{data &= ~(1<<1);}

	if(HAL_GPIO_ReadPin(LCD_DATA_6_PORT, LCD_DATA_6_PIN)){
		data |= (1<<2);
		}else{data &= ~(1<<2);}

	if(HAL_GPIO_ReadPin(LCD_DATA_7_PORT, LCD_DATA_7_PIN)){
		data |= (1<<3);
		}else{data &= ~(1<<3);}

	return data;
}

uint8_t lcd_receive_byte(void)
{
	uint8_t byte = 0;

	//switch PINOUT to input
	LCD_DATA_4_PORT->MODER &= ~(3<<LCD_DATA_4_PIN*2);
	LCD_DATA_4_PORT->PUPDR |=  (2<<LCD_DATA_4_PIN*2);
	LCD_DATA_5_PORT->MODER &= ~(3<<LCD_DATA_5_PIN*2);
	LCD_DATA_5_PORT->PUPDR |=  (2<<LCD_DATA_5_PIN*2);
	LCD_DATA_6_PORT->MODER &= ~(3<<LCD_DATA_6_PIN*2);
	LCD_DATA_6_PORT->PUPDR |=  (2<<LCD_DATA_6_PIN*2);
	LCD_DATA_7_PORT->MODER &= ~(3<<LCD_DATA_7_PIN*2);
	LCD_DATA_7_PORT->PUPDR |=  (2<<LCD_DATA_7_PIN*2);
	Set_E();
	HAL_Delay(2);
	byte = (lcd_readHalf()<<4) & 0xF0;
	Reset_E();

	Set_E();
	HAL_Delay(2);
	byte |= (lcd_readHalf()) & 0x0F;
	Reset_E();

	//switch PINOUT to output again
	LCD_DATA_4_PORT->MODER |= (1<<LCD_DATA_4_PIN*2);
	LCD_DATA_4_PORT->PUPDR &= ~(3<<LCD_DATA_4_PIN*2);
	LCD_DATA_5_PORT->MODER |= (1<<LCD_DATA_5_PIN*2);
	LCD_DATA_5_PORT->PUPDR &= ~(3<<LCD_DATA_5_PIN*2);
	LCD_DATA_6_PORT->MODER |= (1<<LCD_DATA_6_PIN*2);
	LCD_DATA_6_PORT->PUPDR &= ~(3<<LCD_DATA_6_PIN*2);
	LCD_DATA_7_PORT->MODER |= (1<<LCD_DATA_7_PIN*2);
	LCD_DATA_7_PORT->PUPDR &= ~(3<<LCD_DATA_7_PIN*2);

	return byte;

}

void lcd_add_specials(void){

	uint8_t saved_state = 0;
	uint32_t comm = 0;

	_save_state(&saved_state);

	Reset_RS();
	Reset_RW();
	lcd_send_byte(0b10000000);		//SET DDRAM ADDRESS - first column - see documentation

	for(int j = 0; j < 2; j++){
		Set_RS();
		lcd_send_byte(0b00000000);		//SEND DDRAM DATA
		Reset_RS();

		for(int i = 0; i <= 7; i++){
			comm = (0b01000000)+i; // CGRAM ROW ADDRESS
			lcd_send_byte(comm);		//SET CGRAM ADDRESS
			HAL_Delay(1);
			Set_RS();
			lcd_send_byte(shapes[j][i]);//SEND CGRAM DATA
			Reset_RS();

		}

	_return_state(&saved_state);


	}
}
