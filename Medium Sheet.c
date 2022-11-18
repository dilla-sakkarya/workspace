

// THIS SNIPPED IS THE BOOTLOADER PROGRAM.

#include "main.h"                  

#define BOOT_FLAG_ADDRESS           0x08004000U
#define APP_ADDRESS                 0x08008000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U
                                                  
static UART_HandleTypeDef huart;
static uint8_t RX_Buffer[32];
typedef enum                                         
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;

static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
    Clk_Update();
    Boot_Init();


    Transmit_ACK(&huart);                                                           // Transmitting acknowledge which may mean "I am Ready"
    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)        // Trying to receive 2 byte before timeout
    {
        Transmit_NACK(&huart);												     	// uart timeout occurs, transmitting notacknowledge
        Jump2App();                                                                 // jumping to main program address.
    }
    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)                    //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                                     // uart timeout occurs, transmitting notacknowledge
        Jump2App();                                                                // jumping to main program address.
    }
    
	for(;;)                                                                         //Selection of action to be taken based on the information received 
	{
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 2 byte before timeout    
        
        if(Check_Checksum(RX_Buffer, 2) != 1)                                       //checksum control of received bytes to ensure received data is valid
        {
            Transmit_NACK(&huart);                                                  // checksum error occurs, transmitting notacknowledge
        }
        else 
        {
            switch(RX_Buffer[0])                                                    //Deciding which of the functions to delete, write, check and jump according to the incoming buffer value.
            {
                case ERASE:
                    Transmit_ACK(&huart);                                          //transmitting acknowledge
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);                                          //transmitting acknowledge
                    Write();
                    break;
                case CHECK:
                    Transmit_ACK(&huart);                                          //transmitting acknowledge
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);                                          //transmitting acknowledge
                    Jump2App();                                                    // jumping to main program address.
                    break;
                default: 
                    Transmit_NACK(&huart);                                         //transmitting notacknowledge                                 
                    break;
            }
        }
	}
    
    for(;;);
	return 0;                                                                      //will execute and process successfully
}

static void Jump2App(void)                                                        // main program address.
{
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        __disable_irq();
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4);
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);
        void (*pmain_app)(void) = (void (*)(void))(jump_address);
        pmain_app();
    }
    
}


static void Boot_Init(void)                                               //initialization settings
{
    GPIO_InitTypeDef gpio_uart;
    
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_uart.Mode = GPIO_MODE_AF_PP;
    gpio_uart.Pull = GPIO_PULL_NONE;
    gpio_uart.Speed = GPIO_SPEED_LOW;
    gpio_uart.Alternate = GPIO_AF7_USART2;
    
    HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &gpio_uart);
    
    huart.Init.BaudRate = 115200;
    huart.Init.Mode = HAL_UART_MODE_TX_RX;
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16;
    huart.Init.Parity = HAL_UART_PARITY_NONE;
    huart.Init.StopBits = HAL_UART_STOP_1;
    huart.Init.WordLength = HAL_UART_WORD8;
    huart.Instance = USART2;
    
    HAL_RCC_USART2_CLK_ENABLE();
    HAL_UART_Init(&huart);
}


static void Transmit_ACK(UART_HandleTypeDef *handle)                                //Transmit acknowledge function
{
    uint8_t msg[2] = {ACK, ACK};
    
    HAL_UART_Tx(handle, msg, 2);
}


static void Transmit_NACK(UART_HandleTypeDef *handle)                              //Transmit notacknowledge function
{
    uint8_t msg[2] = {NACK, NACK};
    
    HAL_UART_Tx(handle, msg, 2);
}


static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)                      //Checksum function
{
    uint8_t initial = 0xFF;
    uint8_t result = 0x7F; 
    
    result = initial ^ *pBuffer++;
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }
    
    result ^= 0xFF;
    
    if(result == 0x00)                                                           //null character
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void Erase(void)                                                    //Erase part
{
    Flash_EraseInitTypeDef flashEraseConfig;
    uint32_t sectorError;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 3 byte before timeout

    if(Check_Checksum(RX_Buffer, 3) != 1)                                  //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                             //transmitting notacknowledge
        return;
    }
    
    if(RX_Buffer[0] == 0xFF)                                               //Is the first value wrong?
    {
        Transmit_NACK(&huart);                                             //transmitting notacknowledge
    }
    else
    {

        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;          //The sector to be deleted is determined
        flashEraseConfig.NbSectors = RX_Buffer[0];                        //The location of the sector to be deleted is determined
        flashEraseConfig.Sector = RX_Buffer[1];                           //sector to be deleted

        HAL_Flash_Unlock();                                               //The data to be changed is unlocked so that the deletion can be performed.
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);                 //the specified area is deleted
        HAL_Flash_Lock();                                                 //is locked again
        
        Transmit_ACK(&huart);                                             //Transaction terminated correctly 
    }
}

static void Write(void)                                                      //Write Data
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 5 byte before timeout

    if(Check_Checksum(RX_Buffer, 5) != 1)                                   //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                              //transmitting notacknowledge
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }

    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);          //There is a starting address to add to add by shifting the buffers of the incoming data so that they are not written on top of each other.
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 2 byte before timeout
    numBytes = RX_Buffer[0];
    
    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    if(Check_Checksum(RX_Buffer, 5) != 1)                                   //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                              //transmitting notacknowledge
        return;
    }

    i = 0;
    HAL_Flash_Unlock();                                                     //location to write is unlocked
    while(numBytes--)                                                       
    {
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]); //The write operation is performed by starting from the start address and increasing one by one.
        startingAddress++;
        i++; 
    }
    HAL_Flash_Lock();                                                             //The part whose writing process ends is locked.
    Transmit_ACK(&huart);                                                         //Transaction terminated correctly 
}

static void Check(void)                                                     //control process
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 5 byte before timeout
    
    if(Check_Checksum(RX_Buffer, 5) != 1)                                   //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                              //transmitting notacknowledge
        return;
    }
    else
    {
        Transmit_ACK(&huart);                                              //transmitting notacknowledge
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);       //There is a starting address to add to add by shifting the buffers of the incoming data so that they are not written on top of each other.
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Trying to receive 5 byte before timeout

    if(Check_Checksum(RX_Buffer, 5) != 1)                                  //checksum control of received bytes to ensure received data is valid
    {
        Transmit_NACK(&huart);                                            //transmitting notacknowledge
        return;
    }
    else
    {
        Transmit_ACK(&huart);                                             //transmitting acknowledge
    }
    
    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);         //By shifting the buffers of the incoming data in such a way that it will not be written on top of each other, the end address is found. 
    
    HAL_RCC_CRC_CLK_ENABLE();
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    for(address = startingAddress; address < endingAddress; address += 4) //It is checked by increasing the address bytes by 4 from the start address to the ending address.
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1);
    }
    
    HAL_RCC_CRC_CLK_DISABLE();
    if(crcResult == 0x00)                                                  //If it returns null after the check is finished, the operation was successful.
    {
        Transmit_ACK(&huart);                                              //transmitting acknowledge
    } 
    else
    {
        Transmit_NACK(&huart);                                             //transmitting notacknowledge
    }
    
    Jump2App();                                                            // jumping to main program address.
}