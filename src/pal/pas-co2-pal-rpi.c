#include "pas-co2-serial-rpi.h"
#include "xensiv_pasco2.h"

#define XENSIV_PASCO2_UART_TIMEOUT_MS           (500U)

int32_t xensiv_pasco2_plat_i2c_transfer(void * ctx, uint16_t dev_addr, const uint8_t * tx_buffer, size_t tx_len, uint8_t * rx_buffer, size_t rx_len)
{
    (void)(dev_addr);
    int *i2c = (int*)(ctx);  //change made, assigning the i2c handle as pointer, typecasting from void * to int *
    size_t ret = 0;
    ret = write(*i2c,tx_buffer,tx_len);
    if( ret != tx_len)
    {
        return XENSIV_PASCO2_ERR_COMM;
    }

    if(NULL != rx_buffer)
    {
        ret = read(*i2c, rx_buffer, rx_len);
        if( ret != rx_len)
        {
            return XENSIV_PASCO2_ERR_COMM;
        }
    }

   return XENSIV_PASCO2_OK;
}
int32_t xensiv_pasco2_plat_uart_read(void * ctx, uint8_t * data, size_t len)
{
    RPI_ASSERT(ctx != NULL);
    RPI_ASSERT(data != NULL);

    int *uart = (int*)ctx;  //change made, assigning the uart handle as pointer, typecasting from void * to int *
    uint32_t timeout = XENSIV_PASCO2_UART_TIMEOUT_MS;
    size_t xfer_len = 0;

    xensiv_pasco2_plat_delay(timeout);

    xfer_len = read(*uart, data, len);

    return (len == xfer_len) ? 
            XENSIV_PASCO2_OK : 
            XENSIV_PASCO2_ERR_COMM;
}

int32_t xensiv_pasco2_plat_uart_write(void * ctx, uint8_t * data, size_t len)
{
    RPI_ASSERT(ctx != NULL);
    RPI_ASSERT(data != NULL);
        
    int *uart = (int*) ctx; //change made, assigning the uart handle as pointer, typecasting from void * to int *
    tcflush(*uart, TCIFLUSH);

    size_t xfer_len = write(*uart, data, len);
    
    return (len == xfer_len) ? 
            XENSIV_PASCO2_OK : 
            XENSIV_PASCO2_ERR_COMM;
}

void xensiv_pasco2_plat_delay(uint32_t ms)
{
    usleep(ms*1000);
}

uint16_t xensiv_pasco2_plat_htons(uint16_t x)
{
    uint16_t rev_x = ((x & 0xFF) << 8) | ((x & 0xFF00) >> 8);

    return rev_x;
}

void xensiv_pasco2_plat_assert(int expr)
{
    RPI_ASSERT(expr);
}
