

#ifndef __KERNEL__
    #define __KERNEL__
#endif

#ifndef MODULE
    #define MODULE
#endif

#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-sunxi.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <mach/sunxi-chip.h>
#include <linux/clk/sunxi.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/module.h>    // included for all kernel modules
#include <linux/kernel.h>    // included for KERN_INFO
#include <linux/init.h>      // included for __init and __exit macros
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <asm/ioctl.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>
#include "gpio_lib.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lakshman");
MODULE_DESCRIPTION("keypad module");

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
// --------------- Kernel timer -------------------
//static struct timer_list rtc_timer ;
struct timer_list rtc_timer ;
static void rtc_timedout (unsigned long arg) ;
int kernel_rtc_flag= 0 ;

struct lcd_display1 {
        unsigned char row ;
        unsigned char col ;
        unsigned char font ;
        unsigned char size ;
        unsigned char *buffer ;
}__attribute__ ((packed)) ;

#define CLRSCREEN		0x11
#define CLRLINE			0x12
#define LCDTEXT			0x13
#define LCDBMP			0x14
#define LCDBOX			0x15
#define ALPHABETSENTRY	0x16
#define NUMERICENTRY	0x17
#define LISTUPDOWN      0x18
#define MENUOPTION      0x19
#define IDENTRY         0x01
#define NAMEENTRY       0x22
#define KEYREAD         0x23
/*
#define  R1  SUNXI_GPA(06)
#define  R2  SUNXI_GPA(00)  //(07)
#define  R3  SUNXI_GPA(01)  //(07) //notworking  15 pin is used as D7 in lcd
#define  R4  SUNXI_GPA(10)  // (19)

#define  C1  SUNXI_GPA(03)  //(18)
#define  C2  SUNXI_GPA(07)  //(02)
#define  C3  SUNXI_GPG(07)  //(13)
#define  C4  SUNXI_GPG(06)  //(10)
#define  C5  SUNXI_GPA(01)  //this pin is used both by lcd and keypad
#define  C6  SUNXI_GPA(14)  //this pin is used both by lcd and keypad
*/
// columns are in out put mode only columns are multiplexed

#define  C1  SUNXI_GPA(06) //ok
#define  C2  SUNXI_GPA(03)  //used for lcd as well
#define  C3  SUNXI_GPA(01)  //used in lcd
#define  C4  SUNXI_GPA(10)  // (19)

// rows are used in input mode
#define  R1  SUNXI_GPA(00)  //   as out put is not working properly rows are confgrd as inputs
#define  R2  SUNXI_GPA(07)  //
#define  R3  SUNXI_GPG(07)  //   input pins are separate and not muxed with lcd lines because output to input transformation
#define  R4  SUNXI_GPG(06)  //  difficuilty and not working properly may need more delay to transform after cal

#define LCDKPD_DEVICE_NAME "keypad_cdrv"
#define SUNXI_PULL_NONE (0)
#define SUNXI_PULL_UP (1)
#define SUNXI_PULL_DOWN (2)
#define ROWS 4
#define COLS 4
#define LOW 0
#define HIGH 1
#define digitalwrite(pin,val)  sunxi_gpio_output(pin,val)
#define digitalread(pin)  sunxi_gpio_input(pin)
#define pinmode(pin,mode)    sunxi_gpio_set_cfgpin(pin,mode)
#define INPUT SUNXI_GPIO_INPUT
#define OUTPUT SUNXI_GPIO_OUTPUT

#define CHR_BOUND  3+'0'
#define BACKSPACE  8
#define CLEARSCREEN  12
#define CARRIAGE_RETURN  13
#define CAPSLOCK_ON  17
#define CAPSLOCK_OFF  18
#define NUMLOCK_ON  19
#define NUMLOCK_OFF  20

#define MAX_LENGTH 50
#define uint8_t u8
#define uint32_t u32
#define false 0
#define true 1
/*************lcd**********/
#define _RS  SUNXI_GPA(12)
#define _RW  SUNXI_GPA(16)
#define _E SUNXI_GPA(11)
#define _D4 SUNXI_GPA(01)
#define _D5 SUNXI_GPA(14)
#define _D6 SUNXI_GPA(03)
#define _D7 SUNXI_GPA(15)   //this pin may be avoided to be used as joit mux pin for both lcd keypad because its read/write
//#define _LED SUNXI_GPA(06)
signed char alphanum_idscan_samekey(unsigned char *ptr,unsigned char line_no,unsigned char count,unsigned char max_cnt,unsigned char start_pos);
void cursor_display(unsigned char line_no,unsigned char position,unsigned char action);
void blink_cursor(void);
void Disp_buffer(unsigned char *str,unsigned char page_no,unsigned char position,unsigned char font,unsigned char size);
void lcdpinsconf(void);
void lcdgotoxy(char x, char y);
void lcdgotoaddr(char addr);
void lcdinit();
void lcdclear();
void lcdwritetext(char* text);
void lcdbusy();
void sendcommand(char opCode);
void sendcommand4bit(char opCode);
/***********lcd*************/
/******lcdap******/
//int displaymenu(void);
int buttons=2;
/******lcdap********/
const uint16_t MULTITAP_PERIODS = 5000;
const uint16_t LONGTAP_PERIODS = 5000;
static uint8_t engflag;

struct lcd_display1 display1 ;

enum KEYCODE
{
    NOKEY = 0xFF,
    KEY1 = 0xEE, KEY2 = 0xDE, KEY3 = 0xBE, CLEAR = 0x7E,
    KEY4 = 0xED, KEY5 = 0xDD, KEY6 = 0xBD, SHIFT = 0x7D,
    KEY7 = 0xEB, KEY8 = 0xDB, KEY9 = 0xBB, CANCEL = 0x7B,
    STAR = 0xE7, KEY0 = 0xD7, ESC = 0xB7, ENTER = 0x77
};

enum KEY_CODE
{
    NO_KEY = 0xFF,
    KEY_1 = 0xEE, KEY_2 = 0xDE, KEY_3 = 0xBE, KEY_A = 0x7E,
    KEY_4 = 0xED, KEY_5 = 0xDD, KEY_6 = 0xBD, KEY_B = 0x7D,
    KEY_7 = 0xEB, KEY_8 = 0xDB, KEY_9 = 0xBB, KEY_C = 0x7B,
    KEY_ASTERISK = 0xE7, KEY_0 = 0xD7, KEY_NUMBER_SIGN = 0xB7, KEY_D = 0x77
};

enum KEY_STATE
{
    KEY_DOWN, MULTI_TAP, LONG_TAP, MULTI_KEY_DOWN, KEY_UP, CANCELED
};

const char KEY_CHARACTER[] =
{
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

typedef struct keys
{
    uint8_t code;
    uint8_t lastCode;
    uint8_t beforeLastCode;
    uint8_t character;
    unsigned int tapCounter;
    enum KEY_STATE state;
}Key;

uint8_t    keyPins[8];
uint8_t    beforeLastKeyCode = NO_KEY;
uint8_t    lastKeyCode = NO_KEY;
uint8_t    currentKeyCode = NO_KEY;
int   tapCounter = 0;
static uint32_t   multitapTimeout;
uint32_t   longtapTimeout=30;

unsigned char  MAXCHARS = 20 ;
unsigned char edit_flag_samekey = 0 ;
unsigned char edit_flag = 0 ;
unsigned char same_key_flag = 1 ;
unsigned char next_key_flag = 0 ;
unsigned char key_pressed_flag = 0 ;
unsigned char key_timer_flag = 0 ;
int key_count = 0 ;
static void  (*user_function) (void);
int getKey(void);
inline static void resetTapCounter(void) { tapCounter = 0; }
inline static void attachFunction(void (*funct) (void)) { user_function = funct; }
static uint8_t  scanKeyCode(void);
static int  scanRows(void);
static uint8_t  getChr(uint8_t);
static uint8_t  getIndex(uint8_t);
static bool     isMultiKeyDown(uint8_t code);
bool alphaMode = false;
bool upperCaseMode = true;
bool autoOffBacklight = false;
bool isEndOfDisplay = false;
uint8_t startCursorPos;
uint8_t endCursorPos;
uint8_t cursorPos;
uint8_t chrCtr;
unsigned long backlightTimeout;
//char strBuffer[20];
int longtapState = true;
bool  isCanceled = false;
static int lcdkpd_device_id;
struct task_struct *task;
int data;
int ret;
unsigned int SUNXI_PIO_BASE =0;

void *pc;
int keypad_init();
int findlowRow();
Key retkey,key;
static char lcdkpd_device_buf[MAX_LENGTH];
struct cdev *keypad_cdev;
dev_t keypaddev,dev;
static struct file_operations lcdkpd_device_file_ops;

int sunxi_gpio_init(void) {
    unsigned int addr_start, addr_offset;
    unsigned int PageSize, PageMask;
    PageSize = PAGE_SIZE;
    PageMask = (~(PageSize - 1));
    addr_start = SW_PORTC_IO_BASE & PageMask;
    addr_offset = SW_PORTC_IO_BASE & ~PageMask;
    pc = ioremap(addr_start, PageSize*2);
    if (pc == NULL) {
        return (-1);
    }

    SUNXI_PIO_BASE = (unsigned int) pc;
    SUNXI_PIO_BASE += addr_offset;
    return 0;
}

int sunxi_gpio_set_cfgpin(unsigned int pin, unsigned int val) {

    unsigned int cfg;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int index = GPIO_CFG_INDEX(pin);
    unsigned int offset = GPIO_CFG_OFFSET(pin);

    if (SUNXI_PIO_BASE == 0) {
        return -1;
    }

    struct sunxi_gpio *pio =NULL;
           pio = &((struct sunxi_gpio_reg*) SUNXI_PIO_BASE)->gpio_bank[bank];

    cfg = *(&pio->cfg[0] + index);
    cfg &= ~(0xf << offset);
    cfg |= val << offset;
    *(&pio->cfg[0] + index) = cfg;

    return 0;
}

int sunxi_gpio_get_cfgpin(unsigned int pin) {
    unsigned int cfg;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int index = GPIO_CFG_INDEX(pin);
    unsigned int offset = GPIO_CFG_OFFSET(pin);
    if (SUNXI_PIO_BASE == 0) {
        return -1;
    }
    struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *) SUNXI_PIO_BASE)->gpio_bank[bank];
    cfg = *(&pio->cfg[0] + index);
    cfg >>= offset;
    return (cfg & 0xf);
}

int sunxi_gpio_output(unsigned int pin, unsigned int val) {
    unsigned int bank = GPIO_BANK(pin);
    unsigned int num = GPIO_NUM(pin);
    if (SUNXI_PIO_BASE == 0) {
        return -1;
    }
    struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *) SUNXI_PIO_BASE)->gpio_bank[bank];
    if (val)
        *(&pio->dat) |= 1 << num;
    else
        *(&pio->dat) &= ~(1 << num);
    return 0;
}

int sunxi_gpio_pullup(unsigned int pin, unsigned int pull) {
    unsigned int cfg;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int index = GPIO_PUL_INDEX(pin);
    unsigned int offset = GPIO_PUL_OFFSET(pin);
    if (SUNXI_PIO_BASE == 0) {
        return -1;
    }
    struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *) SUNXI_PIO_BASE)->gpio_bank[bank];
    cfg = *(&pio->pull[0] + index);
    cfg &= ~(0x3 << offset);
    cfg |= pull << offset;

    *(&pio->pull[0] + index) = cfg;

    return 0;
}

int sunxi_gpio_input(unsigned int pin) {

    unsigned int dat;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int num = GPIO_NUM(pin);

    if (SUNXI_PIO_BASE == 0) {
        return -1;
    }

    struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *) SUNXI_PIO_BASE)->gpio_bank[bank];

    dat = *(&pio->dat);
    dat >>= num;

    return (dat & 0x1);
}



/*********************************************/


signed char alphanum_idscan_samekey(unsigned char *ptr,unsigned char line_no,unsigned char count,unsigned char max_cnt,unsigned char start_pos)
{
        unsigned char const alptbl[][10]={
                {2,'0'},
                {2,'1'},
                {5,'A','B','C','2'},
                {6,'D','E','F','3'},
                {5,'G','H','I','4'},
                {5,'J','K','L','5'},
                {5,'M','N','O','6'},
                {6,'P','Q','R','S','7'},
                {5,'T','U','V','8'},
                {6,'W','X','Y','Z','9'},

        };
        unsigned char cursor_pos=0;
        unsigned char active_cursor=0;
        unsigned char atemp_buff[65];
        unsigned char col_var=0;
        static unsigned char *temp_ptr=NULL;
        static unsigned char *disp_ptr=NULL;
        unsigned char dup_row_no;
        unsigned char clr_flag=0;
        unsigned char col,row=0;
        unsigned char shift_column;
        unsigned char same_key = 0xff ,keyp=0xff;
        int i = 0 ,count_flag = 0 ;
        int key_len  ;
        int alptblln;

        //display();

        MAXCHARS = 21;
        if(count<=(MAXCHARS-start_pos)){    printk(KERN_ERR,"count is lesss mxcar -str\n");
                Disp_buffer(ptr,line_no,start_pos,0,count);
        }
        else{
                printk(KERN_ERR,"count is lesss mxcar -str else\n");
                Disp_buffer(ptr,line_no,start_pos,0,MAXCHARS-start_pos);
        }
        dup_row_no=line_no;

        if(count<=(MAXCHARS-start_pos-0x02)){
            printk(KERN_ERR,"count is lesss mxcar -str -0x02\n");
                cursor_pos=count;
                active_cursor=count;
        }
        else{    printk(KERN_ERR,"count is -str -0x02  else\n");
                cursor_pos=MAXCHARS-start_pos;
                active_cursor=MAXCHARS-start_pos;
        }
        col=0;
        if(count!=0)
                col=1;
        disp_ptr=ptr;
        edit_flag_samekey = 1;
        do{
                //longtapTimeout = millis() + LONGTAP_PERIODS;

                //key = 0xff;
                 //cursor_display (dup_row_no, (active_cursor + start_pos-1), 0x00) ;
                 //blink_cursor() ;
                 //getKey();
                 keyp=scanKeyCode();//key.code;
                if (keyp != 0xFF){
                    //printk(KERN_ERR "in if key not eaqual\n");
                    switch (keyp) {
                    case KEY0:
                        row=0;  //row 0 is the first row of alptbl array rows, a row contains a set of chars.
                        break;  //row is considered differently at some parts of code but here it represents row numer of total rows in alptbl.
                    case KEY1:
                        row=1;
                        break;
                    case KEY2:
                        row=2;
                        break;
                    case KEY3:
                        row=3;
                        break;
                    case KEY4:
                        row=4;
                        break;
                    case KEY5:
                        row=5;
                        break;
                    case KEY6:
                        row=6;
                        break;
                    case KEY7:
                        row=7;
                        break;
                    case KEY8:
                        row=8;
                        break;
                    case KEY9:
                        row=9;
                        break;
                    default:
                        break;
                    }

        //                row = key ;
        //	printk ("KEY = %d\n",key) ;
                        if(same_key == keyp){
                //printk ("SAME KEY \n") ;
                if(count == max_cnt){
                    count_flag = 1 ;
                }

                                if(next_key_flag){
                                        same_key_flag = 1 ;
                                        next_key_flag = 0 ;
                                        key_count = 0 ;
                                        key_timer_flag = 0 ;
                                        key_pressed_flag = 1 ;
                                }
                                else{
                                        same_key_flag = 0 ;
                                        key_count = 0 ;
                                }

                        }
                        else {

                                same_key_flag = 1 ;
                                key_pressed_flag = 1 ;
                                next_key_flag = 0 ;
                                key_count = 0 ;
                                multitapTimeout=0;
                        }
                }
                if (count < 2 ){
                        temp_ptr = ptr ;
                        disp_ptr = ptr ;
                }
                if(keyp != 0xFF){ //printk(KERN_ERR "in if key not eaqual second\n");
                switch (keyp){//printk(KERN_ERR "second case\n");
                        //case KEY0:
                          //      break ;
                        //case PAPERFEED:
                          //      break ;

                        //case HASH: break;
                        case ESC :
                                same_key_flag = 1 ;
                                next_key_flag = 0 ;
                                key_pressed_flag = 0 ;
                                key_timer_flag = 0 ;
                                key_count = 0 ;
                                edit_flag_samekey = 0 ;
                              /*  if (org_cur1 == 0x80)
                                        org_cur1 = 0x40 ;
                                if (org_cur2 == 0x00)
                                        org_cur2 = 0xff ; */
                              //  blink_cursor() ;
                                return -1 ;break;
/*                      case HASH :
                                if (cursor_pos == 0)
                                        break ;
                                row = 0 ;
                                for (line_no = 0; line_no < 15; line_no ++){
                                        for (col_var = 1; col_var < alptbl[row][0]; col_var ++){
                                                if (disp_ptr[active_cursor - 1] == alptbl[row][col_var]){
                                                        col = col_var ;
                                                        col_var = alptbl[row][0] ;
                                                        line_no = 15 ;
                                                        break ;
                                                }
                                        }
                                        if (line_no < 15)
                                                row ++ ;
                                }
                                col = col + 1 ;
                                if (col >= col_var)
                                        col = 1 ;
                                disp_ptr[active_cursor - 1] = alptbl[row][col] ;
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
*/
                        case ENTER :
                                same_key_flag = 1 ;
                                next_key_flag = 0 ;
                                key_pressed_flag = 0 ;
                                key_timer_flag = 0 ;
                                key_count = 0 ;
                                break ;
                        case CLEAR :
                                if (cursor_pos == 0)
                                        break ;
                                if ((cursor_pos >= 1) && (cursor_pos <= (MAXCHARS - start_pos))){
                                        if (count == cursor_pos)
                                                ptr[cursor_pos - 1] = 0x20 ;
                                        else{
                                                memcpy(atemp_buff, ptr,count) ;
                                                memset(&ptr[cursor_pos], 0x20, max_cnt) ;
                                                memcpy(&ptr[cursor_pos - 1],&atemp_buff[cursor_pos], (count - cursor_pos)) ;
                                        }
                                        cursor_pos -- ;
                                        active_cursor -- ;
                                        count -- ;
                                }
                                else if (cursor_pos > (MAXCHARS - start_pos)){
                                        if (count == cursor_pos)
                                                ptr[cursor_pos - 1] = 0x20 ;
                                        else{
                                                memcpy(atemp_buff, ptr,count) ;
                                                memset(&ptr[cursor_pos], 0x20, max_cnt) ;
                                                memcpy(&ptr[cursor_pos - 1],&atemp_buff[cursor_pos], (count - cursor_pos)) ;
                                        }
                                        cursor_pos -- ;
                                        count -- ;
                                        disp_ptr -- ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count+1);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                key_pressed_flag = 0 ;
                                //same_key_flag = 1 ;
                                same_key = 0xff ; //mdelay(100);
                                break ;
                        /*
                        case F1 :
                                if (cursor_pos == 0)
                                        break ;
                                if((cursor_pos >= 1) && (cursor_pos <= (MAXCHARS - start_pos))){
                                        cursor_pos -- ;
                                        active_cursor -- ;
                                }
                                else if (cursor_pos > (MAXCHARS - start_pos)){
                                        disp_ptr -- ;
                                        cursor_pos -- ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
                        case F4 :
                                if (cursor_pos == count)
                                        break ;
                                if (cursor_pos < (MAXCHARS - start_pos)){
                                        cursor_pos ++ ;
                                        active_cursor ++ ;
                                }
                                else if (cursor_pos >= (MAXCHARS - start_pos)){
                                        cursor_pos ++ ;
                                        disp_ptr ++ ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
                          */
                        default :
                               // printk (KERN_ERR "In Default\n");
                               // printk (KERN_ERR "COUNT = %d\n",count) ;
                               // printk (KERN_ERR "max_cnt = %d\n",max_cnt) ;

                                /*if(count>=19){
                                    keyp = 0xff ;
                                    edit_flag_samekey = 0 ;
                                    return 20;
                                }*/
                                if(count_flag == 1){
                                    count_flag = 0 ;
                                }
                                else{
                                    //printk(KERN_ERR "in else continue\n");
                                if ((count >= max_cnt)) //|| keyp > 0x0b)
                                        continue ;
                                }
                                //printk (KERN_ERR "keyp = %x\n",keyp) ;
                                same_key = keyp ;
                                if(same_key_flag){
                                    //printk(KERN_ERR "same key flag if\n");
                                col_var = alptbl[row][0] ;
                                //printk (KERN_ERR "col_var = %d\n",col_var) ;
                                col = 1 ;
                                if (cursor_pos != count){ //printk(KERN_ERR "curnt cur not equal count\n");
                                        memcpy (atemp_buff,ptr,count) ;
                                        ptr[cursor_pos] = alptbl[row][col] ;
                                        memcpy (&ptr[cursor_pos + 1], &atemp_buff[cursor_pos],count) ;
                                        //printk(KERN_ERR "disp_ptr=%s\n ptr=%s\n",atemp_buff,ptr );
                                }
                                else{
                                        //printk(KERN_ERR "cunrt cur eqlse count\n");
                                        ptr[count] = alptbl[row][col] ;
                                }
                                //printk(KERN_ERR "count=%d \t cursor_pos=%d\n",count,cursor_pos);
                                count ++ ;
                                cursor_pos ++ ;


                                if((cursor_pos<=(MAXCHARS-start_pos))&&(active_cursor<=(MAXCHARS-start_pos))){
                                                active_cursor++;   // printk(KERN_ERR,"active_cursor%d\n",active_cursor);
                                }
                                if((cursor_pos>(MAXCHARS-start_pos)) && (active_cursor==(MAXCHARS-start_pos))){
                                                disp_ptr+=1;
                                               // printk(KERN_ERR,"disp_ptr = %s\n",disp_ptr);
                                }
                                if(count<(MAXCHARS-start_pos)){
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count);printk(KERN_ERR,"disp_ptr = %s\n",disp_ptr);
                                }
                                else{
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                        //printk(KERN_ERR,"disp_ptr = %s\n",disp_ptr);
                                }
                                }
                                else{
                                  //  printk(KERN_ERR "same key flag else\n");
                                row = 0 ;//row 0 is the first element of alptbl array rows which indicates the total no of chars in that row.
                                //alptblln represents how many total rows in alptbl array incrementing it is moving through each row one after other.
                                //col_var is the actual charecter element index in array
                                for (alptblln = 0; alptblln < 10; alptblln ++){
                                        for (col_var = 1; col_var < alptbl[row][0]; col_var ++){
                                                if (disp_ptr[active_cursor - 1] == alptbl[row][col_var]){
                                                        col = col_var ;
                                                        col_var = alptbl[row][0] ;
                                                        alptblln = 10 ;
                                                        break ;
                                                }
                                        }
                                        if (alptblln < 9)
                                                row ++ ;
                                }
                                col = col + 1 ;
                                if (col >= col_var)
                                        col = 1 ;
                                disp_ptr[active_cursor - 1] = alptbl[row][col] ;//printk(KERN_ERR "disp_ptr**=%s\n ",disp_ptr);
                                if (count < (MAXCHARS - start_pos)){
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count);
                                //printk(KERN_ERR "disp_ptr##=%s\n ",disp_ptr);
                                }
                                else    {
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                //printk(KERN_ERR "disp_ptr@@=%s\n ",disp_ptr );
                                }
                        }
                                break; //default case
                }
                }


        }while(keyp!= ENTER || count<20) ;//keep reading keys entered until enter is pressed or max or 20 charecters read.
        //printk(KERN_ERR "disp_ptrEE=%s\n ptrEE=%s\n",disp_ptr,ptr );

        keyp = 0xff ;
        edit_flag_samekey = 0 ;

        //blink_cursor();
    if (count>0)
        return count;
        else return 0;
}
/*********************************************/


signed char edit_alphabets_scan_samekey(unsigned char *ptr,unsigned char line_no,unsigned char count,unsigned char max_cnt,unsigned char start_pos)
{
        unsigned char const alptbl[][10]={
                {8,'0','#','/',' ','"','$','<','>'},
                {5,'1','-','+','='},
                {8,'2','A','a','B','b','C','c'},
                {8,'3','D','d','E','e','F','f'},
                {8,'4','G','g','H','h','I','i'},
                {8,'5','J','j','K','k','L','l'},
                {8,'6','M','m','N','n','O','o'},
                {10,'7','P','p','Q','q','R','r','S','s'},
                {8,'8','T','t','U','u','V','v'},
                {10,'9','W','w','X','x','Y','y','Z','z'},

        };
        unsigned char cursor_pos=0x00;
        unsigned char active_cursor=0x00;
        unsigned char atemp_buff[255];
        unsigned char col_var=0;
        static unsigned char *temp_ptr;
        static unsigned char *disp_ptr;
        unsigned char dup_row_no;
        unsigned char clr_flag=0;
        unsigned char col,row=0;
        unsigned char shift_column;
        unsigned char same_key = 0xff ,keyp;
        int i = 0 ,count_flag = 0 ;
        int key_len  ;
        unsigned char buff[2];
        display();

        MAXCHARS = 20 ;
        if(count<=(MAXCHARS-start_pos)){
                Disp_buffer(ptr,line_no,start_pos,0,count);
        }
        else{
                Disp_buffer(ptr,line_no,start_pos,0,MAXCHARS-start_pos);
        }
        dup_row_no=line_no;

        if(count<=(MAXCHARS-start_pos-0x02)){
                cursor_pos=count;
                active_cursor=count;
        }
        else{
                cursor_pos=MAXCHARS-start_pos;
                active_cursor=MAXCHARS-start_pos;
        }
        col=0;
        if(count!=0)
                col=1;
        disp_ptr=ptr;
        edit_flag_samekey  = 1 ;
        do{
                //longtapTimeout = millis() + LONGTAP_PERIODS;

                //key = 0xff ;
                 cursor_display (dup_row_no, (active_cursor + start_pos-1), 0x00) ;
                 blink_cursor() ;
                 //getKey();
                 keyp=scanKeyCode();//key.code;
                if (keyp != 0xFF){
                    printk(KERN_ERR "in if key not eaqual\n");
                    switch (keyp) {
                    case KEY0:
                        row=0;
                        break;
                    case KEY1:
                        row=1;
                        break;
                    case KEY2:
                        row=2;
                        break;
                    case KEY3:
                        row=3;
                        break;
                    case KEY4:
                        row=4;
                        break;
                    case KEY5:
                        row=5;
                        break;
                    case KEY6:
                        row=6;
                        break;
                    case KEY7:
                        row=7;
                        break;
                    case KEY8:
                        row=8;
                        break;
                    case KEY9:
                        row=9;
                        break;
                    default:
                        break;
                    }

        //                row = key ;
        //	printk ("KEY = %d\n",key) ;
                        if(same_key == keyp){
                //printk ("SAME KEY \n") ;
                if(count == max_cnt){
                    count_flag = 1 ;
                }

                                if(next_key_flag){
                                        same_key_flag = 1 ;
                                        next_key_flag = 0 ;
                                        key_count = 0 ;
                                        key_timer_flag = 0 ;
                                        key_pressed_flag = 1 ;
                                }
                                else{
                                        same_key_flag = 0 ;
                                        key_count = 0 ;
                                }

                        }
                        else {

                                same_key_flag = 1 ;
                                key_pressed_flag = 1 ;
                                next_key_flag = 0 ;
                                key_count = 0 ;
                                multitapTimeout=0;
                        }
                }
                if (count < 2 ){
                        temp_ptr = ptr ;
                        disp_ptr = ptr ;
                }
                if(keyp != 0xFF){ //printk(KERN_ERR "in if key not eaqual second\n");
                switch (keyp){//printk(KERN_ERR "second case\n");
                        //case KEY0:
                          //      break ;
                        //case PAPERFEED:
                          //      break ;

                        //case HASH: break;
                        case ESC :
                                same_key_flag = 1 ;
                                next_key_flag = 0 ;
                                key_pressed_flag = 0 ;
                                key_timer_flag = 0 ;
                                key_count = 0 ;
                                edit_flag_samekey = 0 ;
                              /*  if (org_cur1 == 0x80)
                                        org_cur1 = 0x40 ;
                                if (org_cur2 == 0x00)
                                        org_cur2 = 0xff ; */
                                blink_cursor() ;
                                return -1 ;
                                //break;
/*                      case HASH :
                                if (cursor_pos == 0)
                                        break ;
                                row = 0 ;
                                for (line_no = 0; line_no < 15; line_no ++){
                                        for (col_var = 1; col_var < alptbl[row][0]; col_var ++){
                                                if (disp_ptr[active_cursor - 1] == alptbl[row][col_var]){
                                                        col = col_var ;
                                                        col_var = alptbl[row][0] ;
                                                        line_no = 15 ;
                                                        break ;
                                                }
                                        }
                                        if (line_no < 15)
                                                row ++ ;
                                }
                                col = col + 1 ;
                                if (col >= col_var)
                                        col = 1 ;
                                disp_ptr[active_cursor - 1] = alptbl[row][col] ;
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
*/
                        case ENTER :
                                same_key_flag = 1 ;
                                next_key_flag = 0 ;
                                key_pressed_flag = 0 ;
                                key_timer_flag = 0 ;
                                key_count = 0 ;
                                break ;
                        case CLEAR :
                                if (cursor_pos == 0)
                                        break ;
                                if ((cursor_pos >= 1) && (cursor_pos <= (MAXCHARS - start_pos))){
                                        if (count == cursor_pos)
                                                ptr[cursor_pos - 1] = 0x20 ;
                                        else{
                                                memcpy(atemp_buff, ptr,count) ;
                                                memset(&ptr[cursor_pos], 0x20, max_cnt) ;
                                                memcpy(&ptr[cursor_pos - 1],&atemp_buff[cursor_pos], (count - cursor_pos)) ;
                                        }
                                        cursor_pos -- ;
                                        active_cursor -- ;
                                        count -- ;
                                }
                                else if (cursor_pos > (MAXCHARS - start_pos)){
                                        if (count == cursor_pos)
                                                ptr[cursor_pos - 1] = 0x20 ;
                                        else{
                                                memcpy(atemp_buff, ptr,count) ;
                                                memset(&ptr[cursor_pos], 0x20, max_cnt) ;
                                                memcpy(&ptr[cursor_pos - 1],&atemp_buff[cursor_pos], (count - cursor_pos)) ;
                                        }
                                        cursor_pos -- ;
                                        count -- ;
                                        disp_ptr -- ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count+1);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                key_pressed_flag = 0 ;
                                //same_key_flag = 1 ;
                                same_key = 0xff ; mdelay(100);
                                break ;
                        /*
                        case F1 :
                                if (cursor_pos == 0)
                                        break ;
                                if((cursor_pos >= 1) && (cursor_pos <= (MAXCHARS - start_pos))){
                                        cursor_pos -- ;
                                        active_cursor -- ;
                                }
                                else if (cursor_pos > (MAXCHARS - start_pos)){
                                        disp_ptr -- ;
                                        cursor_pos -- ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
                        case F4 :
                                if (cursor_pos == count)
                                        break ;
                                if (cursor_pos < (MAXCHARS - start_pos)){
                                        cursor_pos ++ ;
                                        active_cursor ++ ;
                                }
                                else if (cursor_pos >= (MAXCHARS - start_pos)){
                                        cursor_pos ++ ;
                                        disp_ptr ++ ;
                                }
                                if (count < (MAXCHARS - start_pos))
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,count);
                                else
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,FONT0,MAXCHARS-start_pos);
                                break ;
                          */
                        default :
                                printk (KERN_ERR "In Default\n");
                                printk (KERN_ERR "COUNT = %d\n",count) ;
                                printk (KERN_ERR "max_cnt = %d\n",max_cnt) ;
                                if(count_flag == 1){
                                    count_flag = 0 ;
                                }
                                else{printk(KERN_ERR "in else continue\n");
                                if ((count >= max_cnt)) //|| keyp > 0x0b)
                                        continue ;
                                }
                                printk (KERN_ERR "keyp = %x\n",keyp) ;
                                same_key = keyp ;
                                if(same_key_flag){ printk(KERN_ERR "same key flag if\n");
                                col_var = alptbl[row][0] ;
                                printk (KERN_ERR "col_var = %d\n",col_var) ;
                                col = 1 ;
                                if (cursor_pos != count){ printk(KERN_ERR "curnt cur not equal count\n");
                                        memcpy (atemp_buff,ptr,count) ;
                                        ptr[cursor_pos] = alptbl[row][col] ;
                                        memcpy (&ptr[cursor_pos + 1], &atemp_buff[cursor_pos],count) ;
                                        printk(KERN_ERR "disp_ptr=%s\n ptr=%s\n",atemp_buff,ptr );
                                }
                                else{   printk(KERN_ERR "cunrt cur eqlse count\n");
                                        ptr[count] = alptbl[row][col] ;
                                }
                                count ++ ;
                                cursor_pos ++ ;


                                if((cursor_pos<=(MAXCHARS-start_pos))&&(active_cursor<=(MAXCHARS-start_pos))){
                                                active_cursor++;
                                }
                                if((cursor_pos>(MAXCHARS-start_pos)) && (active_cursor==(MAXCHARS-start_pos))){
                                                disp_ptr+=1;
                                }
                                if(count<(MAXCHARS-start_pos)){
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count);
                                }
                                else{
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                }
                                }
                                else{
                                    printk(KERN_ERR "same key flag else\n");
                                row = 0 ;
                                for (line_no = 0; line_no < 10; line_no ++){
                                        for (col_var = 1; col_var < alptbl[row][0]; col_var ++){
                                                if (disp_ptr[active_cursor - 1] == alptbl[row][col_var]){
                                                        col = col_var ;
                                                        col_var = alptbl[row][0] ;
                                                        line_no = 10 ;
                                                        break ;
                                                }
                                        }
                                        if (line_no < 9)
                                                row ++ ;
                                }
                                col = col + 1 ;
                                if (col >= col_var)
                                        col = 1 ;
                                disp_ptr[active_cursor - 1] = alptbl[row][col] ;printk(KERN_ERR "disp_ptr**=%s\n ",disp_ptr);
                                if (count < (MAXCHARS - start_pos)){
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,count);
                                printk(KERN_ERR "disp_ptr##=%s\n ",disp_ptr);
                                }
                                else    {
                                        Disp_buffer(disp_ptr,dup_row_no,start_pos,0,MAXCHARS-start_pos);
                                printk(KERN_ERR "disp_ptr@@=%s\n ",disp_ptr );
                                }
                        }
                                break; //default case
                }
                }
        //      printk(KERN_ERR "disp_ptr=%s\n ptr=%s\n",disp_ptr,ptr );

        }while(keyp!= ENTER || count <20) ; printk(KERN_ERR "disp_ptrEE=%s\n ptrEE=%s\n",disp_ptr,ptr );

        keyp = 0xff ;
        edit_flag_samekey = 0 ;
 /*       if (org_cur1 == 0x80)
                org_cur1 = 0x40 ;
        if (org_cur2 == 0x00)
                org_cur2 = 0xff ; */
        blink_cursor();
    if (count>0)
        return count;
        else return 0;
}

void Disp_buffer(unsigned char *str,unsigned char page_no,unsigned char position,unsigned char font,unsigned char size){
//char dspbuf[20];

//strncpy(dspbuf,str,size);
   // printk(KERN_ERR "pageno = %d\t posi =%d\n " ,page_no,position);
    str[20]='\0';
lcdgotoxy(page_no,0);
lcdwritetext(str);

}
void blink_cursor(void){

//    display();
    cursor();
    //mdelay(5);
    //noCursor();
    /*mdelay(5);
    cursor();
    mdelay(5);
    noCursor();*/
}


int getId(char* buff){
    int i,ret;
    char strBuffer[20];
    for(i=0;i<=3;i++){
        memset(strBuffer,0,20);
        if(i<3){
        /*unsigned char *ptr,unsigned char line_no,unsigned char count,unsigned char max_cnt,unsigned char start_pos*/

           ret=alphanum_idscan_samekey(strBuffer,i,0,20,0);
           if(ret<0){
        memcpy(buff+(20*i),strBuffer,20);
           }
           else
               return -1;//its for if user wanted to cancel entered data or exit from this
        }
        if(i==3)
        {

            ret=alphanum_idscan_samekey(strBuffer,i,0,4,0);
            if(ret<0){
            memcpy(buff+(20*i),strBuffer,4);
            printk(KERN_ERR "srtbuff= %s\n",strBuffer);
            }
        }
        else
            return -1;
    }
buff[64]='\0';
printk(KERN_ERR "buff= %s\n",buff);
return 0;
}

int getName(char * name){

edit_alphabets_scan_samekey(name,0,0,20,0);
return 0;
}

void cursor_display(unsigned char line_no,unsigned char position,unsigned char action) {
  // display();
   setCursor(position,line_no);//mdelay(1000);

}

uint32_t millis(){
unsigned long seconds_milli=jiffies/1000ul;
return (uint32_t)seconds_milli;
}

int keypad_init(){
 //      if(!engflag){
        printk(KERN_ERR "keypad initialized\n");

        pinmode(C1, OUTPUT);
        if(!engflag){
        pinmode(C2, OUTPUT);
        pinmode(C3, OUTPUT);
        }
        pinmode(C4, OUTPUT);

        sunxi_gpio_pullup(R1,SUNXI_PULL_DOWN);mdelay(2);
        sunxi_gpio_pullup(R2,SUNXI_PULL_DOWN);mdelay(2);
        sunxi_gpio_pullup(R3,SUNXI_PULL_DOWN);mdelay(2);
        sunxi_gpio_pullup(R4,SUNXI_PULL_DOWN);mdelay(3);

        digitalwrite(C1,LOW);
        if(!engflag){
        digitalwrite(C2,LOW);
        digitalwrite(C3,LOW);
        }
        digitalwrite(C4,LOW);

        pinmode(R1, INPUT);mdelay(2);
        pinmode(R2, INPUT);mdelay(2);
        pinmode(R3, INPUT);mdelay(2);
        pinmode(R4, INPUT);mdelay(2);
  // }
       return 0;
}


// reading key pressed with debounce handled
uint8_t scanKeyCode(void)
{
uint8_t rows = 0, cols = 0;
  int rowIndex;
        int col=0;
 //       if(!engflag){
        for (col=0;col<=3;col++){
              cols=0 ;rows=0;
              rowIndex=-1;
               digitalwrite(C1,LOW);
               if(!engflag){
               digitalwrite(C2,LOW);
               digitalwrite(C3,LOW);
               }
               digitalwrite(C4,LOW);

          switch(col){
                 case 0:  if(!engflag){
                          digitalwrite(C1, HIGH);//pressedkey='\0';
                          digitalwrite(C2,LOW);
                          digitalwrite(C3,LOW);
                          digitalwrite(C4,LOW);
                          }
                          if (rowIndex ==-1)
                          {
                            do{
                                  rowIndex = scanRows();
                                  if(rowIndex >= 0)
                                  mdelay(50);//debounce delay
                              }while(rowIndex!=scanRows()); //to avoid debounce key
                            if ( rowIndex> -1){
                                 //printk(KERN_ERR "row is %d\n",rowIndex);
                                 //printk(KERN_ERR "col is C1\n");
                                cols=1<<col;rows=1<<rowIndex;
                                return ~(cols | rows<<4);
                            }
                           }
                             break;

                 case 1:  if(!engflag){
                          digitalwrite(C2, HIGH);//pressedkey='\0';
                          digitalwrite(C1,LOW);
                          digitalwrite(C3,LOW);
                          digitalwrite(C4,LOW);
                          }
                          if (rowIndex == -1)
                          {
                           do{
                                  rowIndex = scanRows();
                                  if(rowIndex >= 0)
                                  mdelay(50);
                              }while(rowIndex!=scanRows());
                            if  (rowIndex> -1){
                                 //printk(KERN_ERR "row is %d\n",rowIndex);
                                 //printk(KERN_ERR "col is C2\n");
                                cols=1<<col;rows=1<<rowIndex;
                                return ~(cols | rows<<4);
                                }
                           }

                           break;

                  case 2: if(!engflag){
                          digitalwrite(C3, HIGH);//pressedkey='\0';
                          digitalwrite(C1,LOW);
                          digitalwrite(C2,LOW);
                          digitalwrite(C4,LOW);
                          }
                          if (rowIndex ==-1)
                          {
                            do{
                                  rowIndex = scanRows();
                                  if(rowIndex >= 0)
                                  mdelay(50);
                              }while(rowIndex!=scanRows());
                            if(rowIndex> -1){
                                //printk(KERN_ERR "row is %d\n",rowIndex);
                               //printk(KERN_ERR "col is C3\n");
                                cols=1<<col;rows=1<<rowIndex;
                              return ~(cols | rows<<4);
                                }
                           }

                          break;

                    case 3:if(!engflag){
                          digitalwrite(C4, HIGH);//pressedkey='\0';
                          digitalwrite(C1,LOW);
                          digitalwrite(C2,LOW);
                          digitalwrite(C3,LOW);
                          }
                          if (rowIndex== -1)
                          {
                            do{
                                  rowIndex = scanRows();
                                  if(rowIndex>=0)
                                  mdelay(50);// debouce delay
                              }while(rowIndex!=scanRows()); //if not same key is pressed for a while debounce elimination
                             if (rowIndex> -1){
                                // printk(KERN_ERR "row is %d\n",rowIndex);
                                //printk(KERN_ERR "col is C4\n");
                                cols=0x01<<col;rows=0x01<<rowIndex;
                                return ~(cols  | rows<<4);
                                }
                           }

                         break;

            default: break;
                      }
                 }
 //} //engflag
    // The 8-bits reading code consist off:
    // Bit-7 | Bit-6 | Bit-5 | Bit-4 | Bit-3 | Bit-2 | Bit-1 | Bit-0
    // Col-3 | Col-2 | Col-1 | Col-0 | Row-3 | Row-2 | Row-1 | Row-0

    return 0xFF;
}


int scanRows(void)
{
    int rows = -1;
  // if(!engflag){
            if (digitalread(R1) == HIGH)
            {
                //printk(KERN_ERR "row is R1 in scanrow\n");
                rows=0;
            }
            else if (digitalread(R2) == HIGH)
            {//printk(KERN_ERR "row is R2 in scan row\n");
                rows=1;
            }
            else if(digitalread(R3) == HIGH)
            {//printk(KERN_ERR "row is R3 in scan row\n");
                     rows=2;
            }
            else if (digitalread(R4) == HIGH)
            {//printk(KERN_ERR "row is R4 in scan row\n");
            rows=3;sunxi_gpio_pullup(R4,SUNXI_PULL_DOWN);
            }
 //}
    return rows;
}


static int lcdkpd_device_open(struct inode *inode,struct file  *file)
{



    return 0;
}

/*This function is called when the user program uses close() function
 * The function decrements the number of processes currently
 * using this device. This should be done because if there are no
 * users of a driver for a long time, the kernel will unload
 * the driver from the memory.
 * Return value
 * 	Always returns SUCCESS
 * */
static int lcdkpd_device_release(struct inode *inode,struct file *file)
{

    return 0;
}


static int lcdkpd_device_read(struct file *file,char *buf,size_t lbuf,loff_t *ppos)
{
 return 0;
}


static long lcdkpd_device_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
    char keycode=0xFF;
    int select,nopts,i;//loop;
    static bool prevpage; //
    int ret=0;
    char buf[65];
    /*char *buf;
           buf =(char*)kmalloc(sizeof(char)*65,GFP_KERNEL);*/
    //memset(buf,0,65);
    printk(KERN_ERR "ioctl\n");
    switch (cmd) {

            case CLRSCREEN:
                lcdclear();
                break ;
         /*   case CLRLINE:
                        if(copy_from_user(&clrline1, (struct lcd_clrline1*)arg, sizeof(struct lcd_clrline1)))
                            return -EFAULT;

                break ;
                */
            case LCDTEXT:
                        if(copy_from_user(&display1, (struct lcd_display1*)arg, sizeof(struct lcd_display1))){
                        printk ("Copy from user error\n") ;
                            return -EFAULT;
                }

                        //copy_from_user(display1.buffer,(struct lcd_display1*)arg.buffer,(struct lcd_display1*)arg.size);
                        lcdgotoxy((display1.row-1),2);// row-1 ,as 1st row starts at 0 1 is subtracted
                        lcdwritetext(display1.buffer);
                        printk(KERN_ERR "from ioctl buffer= %s\n",display1.buffer);
                        break ;                  
            /*
            case ALPHABETSENTRY:
                if (copy_from_user (&alpha1, (struct lcd_alpha1*)arg, sizeof(struct lcd_alpha1)))
                    return -EFAULT ;

                            break;

            case NUMERICENTRY:
                if (copy_from_user (&numeric1, (struct lcd_numeric1*)arg, sizeof(struct lcd_numeric1)))
                    return -EFAULT ;
                no_keys = edit_numeric_entry(numeric1.data,numeric1.line_no,numeric1.count, numeric1.max_cnt, numeric1.start_pos) ;
                if (no_keys < 0)
                                    return -1;
                            else
                                    if (no_keys == 0)
                                    return 0;
                            else
                                    return no_keys ;
                            break;*/

              case LISTUPDOWN:
                     select=0;//loop=0;
                     nopts=*((int*)arg);
                     //printk(KERN_ERR "nopts = %d \n",nopts);
                     //lcdclear();
                     if(!prevpage){
                     lcdgotoxy(select,0);
                     lcdwritetext(">>");
                     }
                     do{//printk(KERN_ERR "in do while\n");
                      //   select=0;
                      //   lcdgotoxy(select,0);
                      //   lcdwritetext(">>");
                        keycode=scanKeyCode();
                        if(keycode == KEY_B || keycode == KEY_C){
                         for(i=0;i<=3;i++){
                         lcdgotoxy(i,0);
                         lcdwritetext("  ");  // to clear previous seclect arrow to change it to new location
                         }
                         if(keycode == KEY_C && select<3)
                         {
                             printk(KERN_ERR "in do while keyB  sle= %d\n",select);
                             select = select+1;
                             //*((int*)arg)=select; //we are doing this after breaking do while
                             lcdgotoxy(select,0);
                             lcdwritetext(">>");//we may need to wait here to verify
                             if(select==3)
                                 keycode=0xFF;
                             //mdelay(100);//loop=1;

                         }
                         else if(keycode == KEY_C && select==3){
                             *((int*)arg)=select+2;
                             keycode=0xFF;
                             return 5; //none is selected from current opts continue for next four opts
                         }
                         else if(keycode == KEY_B && select>=0)
                         {
                             if(prevpage){ //coming one page back so pointer need to be at bottom row, that is 4th, each page is of 4 row
                                 select=4;
                                 prevpage=false;
                             }

                             printk(KERN_ERR "in do while keyC sle= %d\n",select);
                             select=select-1;
                             if(select>=0 ){
                             lcdgotoxy(select,0);
                             lcdwritetext(">>"); //mdelay(100);
                             }
                         }
                         else if(keycode == KEY_B && select==-1){
                                 *((int*)arg)=select;//keycode=0xFF;
                                 prevpage=true;
                                 return -1;
                         }

                       keycode=0xFF;
                     }
                        mdelay(10);//its because scankey is too fast and pointer ">>"running down very fast among rows
                     } while(keycode != KEY_D || keycode != KEY_A || keycode !=KEY_ASTERISK);
                     
                     if(keycode== KEY_A)  //this is for exit the contact list display
                     {
                        //*((int*)arg)= -1;//storing negative value in arg may not work as intended
                        return -2;
                    }
                     *((int*)arg)= select+1;  //arg stores contact is selected
                     printk(KERN_ERR "select in list updown = %d\n",select);
                     if(keycode== KEY_D)
                         return 1;

                     break;

                     //list add del call these functions we may keep in menu options
             case MENUOPTION:
                    // lcdclear();
                    // lcdgotoxy(1,0);
                    // lcdwritetext("Please Select Menu keys");
                     do{
                     keycode = 0xFF;
                     keycode = scanKeyCode();
                     if(keycode==KEY_A||keycode==KEY_B||keycode==KEY_C||keycode==KEY_D||keycode==KEY_ASTERISK||keycode==KEY_NUMBER_SIGN)
                     {
                        *((char*)arg) = keycode;
                        return 0;
                     }
                     /*else if (keycode!=0xFF){
                        lcdclear();
                        lcdgotoxy(1,0);
                        lcdwritetext("Please Select Menu keys");
                        //scrollDisplayLeft();
                     }
                     mdelay(20);*/ //scrollDisplayLeft();
                     }while(keycode!=KEY_A||keycode!=KEY_B||keycode!=KEY_C||keycode!=KEY_D||keycode!=KEY_ASTERISK||keycode!=KEY_NUMBER_SIGN);
                     break;
                     //copy_to_user((unsigned char*)buffer,inbuffer,result);
                     // read keypad keys up and down and confirm for enter

                    /*we are reading caps alphabet and numeric in IDENTRY from keypad*/
             case IDENTRY:
                   // printk(KERN_ERR "in IDENTRY case\n");
                    lcdclear();
                    lcdgotoxy(1,0);
                    lcdwritetext("Please Enter toxID");
                    //mdelay(30);// delay is only to keep the above text to be displayed on LCD for a while without clearing
                    memset(buf,0,65);
                    ret=getId(buf);
                    if(ret>0){
                    buf[64]='\0';
                    printk(KERN_ERR "bufioctl =%s\n",buf);
                    ret=copy_to_user((unsigned char*)arg,buf,65);
                    if(ret){
                        //printk(KERN_ERR "addre of arg %p\n",arg);
                        printk(KERN_ERR "cannot copy bytes ret= %d\n ",ret);
                    }
                    }
                    else return ret;
                    break;

               case NAMEENTRY:
                    printk(KERN_ERR "name entry \n");
                    //mdelay(200);
                    lcdclear();
                    lcdgotoxy(1,0);
                    lcdwritetext("Enter Name");
                    //mdelay(300);// delay is only to keep the above text to be displayed on LCD for a while without clearing
                    memset(buf,0,65);
                    getName(buf);
                    buf[64]='\0';
                    printk(KERN_ERR "bufioctl =%s\n",buf);
                    ret=copy_to_user((unsigned char*)arg,buf,20);
                    if(ret){
                    //printk(KERN_ERR "addre of arg %p\n",arg);
                    printk(KERN_ERR "cannot copy bytes ret= %d\n ",ret);
                    }
                    break;

               case KEYREAD:
                    do{
                       keycode = 0xFF;
                       keycode = scanKeyCode();
                       *((char*)arg) = keycode;
                       }while(keycode!=0xFF);
                       break;


              /* case default:
                    break; */


    }
    //kfree(buf);
    return 0;
}

/******************lcd ************/
/*updt*/
// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the system doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).



void setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */

void begin(uint8_t cols, uint8_t lines){

    _numlines = lines;

    setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

}
void clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  udelay(2000);  // this command takes a long time!
}

void home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  udelay(2000);  // this command takes a long time!
}

void setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
void noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
  mdelay(100);
}

// Turn on and off the blinking cursor
void noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */
/*low for cmd and high for data ,i think by:blp*/
void command(uint8_t value) {
  send(value, LOW);
}

size_t write(uint8_t value) {
  send(value, HIGH);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void send(uint8_t value, uint8_t mode) {
  digitalwrite(_RS, mode);

  //  RW pin  set it low to Write

    digitalwrite(_RW, LOW);

    write4bits(value>>4);
    write4bits(value);

}

void pulseEnable(void) {
  digitalwrite(_E, LOW);
  udelay(1);
  digitalwrite(_E, HIGH);
  udelay(1);    // enable pulse must be >450ns
  digitalwrite(_E, LOW);
  udelay(100);   // commands need > 37us to settle
}

void write4bits(uint8_t value) {
engflag=1;
    digitalwrite(_D4, (value >> 0) & 0x01);
    digitalwrite(_D5, (value >> 1) & 0x01);
    digitalwrite(_D6, (value >> 2) & 0x01);
    digitalwrite(_D7, (value >> 3) & 0x01);
engflag=0;
  pulseEnable();
}

/*updt*/
void lcdpinsconf(void)
{
   engflag=1;
 //   sunxi_gpio_init();    //commented because we are doing it already in init module
        pinmode(_RS, OUTPUT);
        pinmode(_RW, OUTPUT);
        pinmode(_E, OUTPUT);
        pinmode(_D4, OUTPUT);
        pinmode(_D5, OUTPUT);
        pinmode(_D6, OUTPUT);
        pinmode(_D7, OUTPUT);
   engflag=0;
}

void lcdgotoxy(char x, char y)
{
  char addr;
  switch(x)
  {
     case 0: addr = 0x80; break; //Starting address of 1st line
     case 1: addr = 0xC0; break; //Starting address of 2nd line
     case 2: addr = 0x94; break; //Starting address of 3rd line
     case 3: addr = 0xD4; break; //Starting address of 4th line
     default: ;
  }

  addr +=y;

  lcdgotoaddr(addr);
}

void lcdgotoaddr(char addr)
{
    char cmd = 0x80 | addr;
    engflag=1;
    digitalwrite(_RS, LOW);
    digitalwrite(_RW, LOW);
    engflag=0;
    sendcommand4bit(cmd);
}


void lcdinit()
{
  lcdpinsconf();
  begin(20,4); //by this we are initialising lib variable to 20 columns and 4 row lcd
  udelay(500);
  //Do the wake up call
  sendcommand(0x30);
  udelay(500);
  sendcommand(0x30);
  udelay(500);
  sendcommand(0x30);
  udelay(500);
  sendcommand(0x20);  //Let's make it 4 bit mode
  udelay(500);
  //That's it LCD is initialized in 4 bit mode.

  sendcommand4bit(0x28); //N = 1 (2 line display) F = 0 (5x8 characters)
 //4Bit
  udelay(100);
  sendcommand4bit(0x0C); //Display on/off control D=0,C=0, B=0
  udelay(100);
  sendcommand4bit(0x01); //Clear Display
  udelay(100);

  sendcommand4bit(0x06); //Entry mode set - I/D = 1 (increment cursor) & S = 0 (no shift)
  sendcommand4bit(0x0E);
}

void lcdclear()
{

    engflag=1;
   digitalwrite(_RS, LOW);
   digitalwrite(_RW, LOW);
   engflag=0;
   sendcommand4bit(0x01);
}

 void lcdwritetext(char* text)
 {

    while( *text)
    {
        engflag=1;
                digitalwrite(_RS,HIGH);
        engflag=0;
        sendcommand4bit(*text);text++;
    }
 }

void lcdbusy()
{
  engflag=1;
  digitalwrite(_RS, LOW);
  pinmode(_D7, INPUT);
  digitalwrite(_RW, HIGH);
  int busyflag = 1;
  while(busyflag == 1)
  {
    //The data should be read while Enable pin is HIGH
    digitalwrite(_E, HIGH);
    busyflag = digitalread(_D7);
    udelay(100);
    digitalwrite(_E, LOW);

    //Clock out the lower part of data, since we are interested in only the
    //upper part. more precissaley D7 pin.
    digitalwrite(_E, HIGH);
    digitalwrite(_E, LOW);
  }
  pinmode(_D7, OUTPUT);
  digitalwrite(_RW, LOW);
  engflag=0;
}

void sendcommand(char opcode)
{
    engflag=1;
    digitalwrite(_RW, LOW);
    digitalwrite(_D4, opcode & 0x10);
    digitalwrite(_D5, opcode & 0x20);
    digitalwrite(_D6, opcode & 0x40);
    digitalwrite(_D7, opcode & 0x80);

    digitalwrite(_E, LOW);
    digitalwrite(_E, HIGH);
    digitalwrite(_E, LOW);
    engflag=0;
}

void sendcommand4bit(char opcode)
{
  char c = opcode;
  engflag=1;
  digitalwrite(_D4,0);
  digitalwrite(_D5,0);
  digitalwrite(_D6,0);
  digitalwrite(_D7,0);

  digitalwrite(_D4, c & 0x10);
  digitalwrite(_D5, c & 0x20);
  digitalwrite(_D6, c & 0x40);
  digitalwrite(_D7, c & 0x80);
  digitalwrite(_E,HIGH);
  digitalwrite(_E,LOW);
  digitalwrite(_D4,0);
  digitalwrite(_D5,0);
  digitalwrite(_D6,0);
  digitalwrite(_D7,0);
  digitalwrite(_D4, c & 0x01);
  digitalwrite(_D5, c & 0x02);
  digitalwrite(_D6, c & 0x04);
  digitalwrite(_D7, c & 0x08);
  digitalwrite(_E,HIGH);
  digitalwrite(_E,LOW);
  engflag=0;
  lcdbusy();
}
/******************************/


/********lcdapp*********/


static void rtc_timedout (unsigned long arg)
{

    //static uint32_t   multitapTimeout;

    kernel_rtc_flag = 1 ;

        if (edit_flag_samekey == 1){
                if(same_key_flag == 1)
                       //blink_cursor() ;
                if(key_pressed_flag){
                        key_count ++ ;
                        key_timer_flag = 0 ;
                }
                if(key_count == 4){
                        next_key_flag = 1 ;
                      // blink_cursor() ;
                        key_count = 0 ;
                        key_pressed_flag = 0 ;
                        key_timer_flag = 1 ;
                }
                //if(key_timer_flag)
                  //      blink_cursor() ;
                multitapTimeout++;
                if (multitapTimeout>=5)
                {
                    multitapTimeout=0;
                    next_key_flag=1;
                }
        }
    //simcom = 1 ;
        if (edit_flag == 1){
                blink_cursor() ;
        }

//	rtc_timer.expires = jiffies + 50 ;
    rtc_timer.expires = jiffies + 60 ;
    add_timer(&rtc_timer) ;
    kernel_rtc_flag = 0 ;
    /* ******** */
}

/*
static int lcdkpd_device_id;
struct task_struct *task;
int data;
int ret;


static char lcdkpd_device_buf[MAX_LENGTH];
struct cdev *keypad_cdev;
dev_t keypaddev,dev;
static struct file_operations lcdkpd_device_file_ops;

*/

static int __init keymod_init(void)
{
    int count=0;int i=0;
    printk(KERN_INFO "Keypad module\n");
    sunxi_gpio_init();
    lcdinit();
    lcdclear();
    keypad_init();
    data = 0;
    printk(KERN_INFO"--------------------------------------------");

    lcdkpd_device_file_ops.owner = THIS_MODULE;
    lcdkpd_device_file_ops.read = lcdkpd_device_read;
    //lcdkpd_device_file_ops.ioctl = lcdkpd_device_ioctl;
    lcdkpd_device_file_ops.open = lcdkpd_device_open;
    lcdkpd_device_file_ops.release = lcdkpd_device_release;
    lcdkpd_device_file_ops.unlocked_ioctl = lcdkpd_device_ioctl;


    ret=alloc_chrdev_region(&keypaddev,0,1,"keypad_cdrv");
    if(ret < 0)
    {
    printk(KERN_ERR "sorry no major number left");
    return ret;
    }
    lcdkpd_device_id= MAJOR(keypaddev);//extract major no
    dev=MKDEV(lcdkpd_device_id,0);
    keypad_cdev= cdev_alloc();
    /*Register our character Device*/
    keypad_cdev->owner=THIS_MODULE;
    keypad_cdev->ops= &lcdkpd_device_file_ops;
    ret=cdev_add(keypad_cdev,dev,1);
    if( ret < 0 ) {
        printk(KERN_ERR "Error registering device driver\n");
        return ret;
    }
    printk(KERN_ERR "Device Registered with MAJOR NO[%d]\n",lcdkpd_device_id);
    //for(i=0; i<MAX_LENGTH; i++) lcdkpd_device_buf[i] = 0;
    //lcdkpd_device_buf[MAX_LENGTH] = '\0';

    //mdelay(500);

    init_timer (&rtc_timer) ;
            rtc_timer.function = rtc_timedout ;
        rtc_timer.expires = jiffies + 60 ;
        rtc_timer.data=0;
            add_timer (&rtc_timer) ;

    return 0;    // Non-zero return means that the module couldn't be loaded.
}

static void __exit keymod_cleanup(void)
{
    printk(KERN_INFO "Cleaning up module.\n");

    while(kernel_rtc_flag == 1); // wait until the flag becomes 0 because rtc is active ,so delete timer will cause trouble
        /* *********** */
    del_timer (&rtc_timer) ;
    unregister_chrdev_region(keypaddev,1);
    cdev_del(keypad_cdev);
    //kthread_stop(task);
    iounmap(pc);
}

module_init(keymod_init);
module_exit(keymod_cleanup);
