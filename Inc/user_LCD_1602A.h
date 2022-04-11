/*
 * user_LCD_1602A.h
 *
 *  Created on: 2022. ápr. 11.
 *
 */

#ifndef INC_USER_LCD_1602A_H_
#define INC_USER_LCD_1602A_H_

typedef enum
{
	LCD_No_Error = 				0x00,
	LCD_I2C_Error = 			0x32

} LCD_Error_StatusTypeDef;


/*---------------Register Address---------------------*/
// Device
#define LCD_DEV_I2C_ADDR 				0x4E




/*---------------------CMD--------------------*/

void lcd_send_cmd (char cmd);

void lcd_send_data (char data);

void lcd_init (void);

void lcd_send_string (char *str);





#endif /* INC_USER_LCD_1602A_H_ */
