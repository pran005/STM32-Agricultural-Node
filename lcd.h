#ifndef lcd_h
#define lcd_h


#define clr_lcd() 		cmd(0x01) 
#define lcd_newline() cmd(0xC0)

void cmd (char command); 
void lcd_data(char data) ; 
void lcd (char *string) ;
void lcd(int m) ;
void lcd(double d) ; 
void init_lcd() ; 

#endif 
