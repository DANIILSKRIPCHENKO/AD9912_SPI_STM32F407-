
// registers addresses
#define AD9912_SERIAL_CONFIG_ADDR 0x0000
#define AD9912_PART_ID_1_ADDR  0x0002
#define AD9912_PART_ID_2_ADDR  0x0003
#define AD9912_SERIAL_OPTIONS_1_ADDR  0x0004
#define AD9912_SERIAL_OPTIONS_2_ADDR  0x0005

#define AD9912_FTW0_0_ADDR  0x01A6
#define AD9912_FTW0_1_ADDR  0x01A7
#define AD9912_FTW0_2_ADDR  0x01A8
#define AD9912_FTW0_3_ADDR  0x01A9
#define AD9912_FTW0_4_ADDR  0x01AA
#define AD9912_FTW0_5_ADDR  0x01AB

#define AD9912_DAC_current_1_ADDR  0x040B
#define AD9912_DAC_current_2_ADDR  0x040C



//values
uint8_t AD9912_IO_UPDATE_VALUE = 0b00000001;


//structures

typedef struct {
	uint8_t Byte1;
	uint8_t Byte2;
} AD9912_ID;
