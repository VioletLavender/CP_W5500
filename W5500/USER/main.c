#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "wizchip_conf.h"
#include "string.h"
#include "loopback.h"


#define W5500_SCS         GPIO_Pin_4
#define W5500_SCS_PORT    GPIOA
#define W5500_RST         GPIO_Pin_3
#define W5500_RST_PORT    GPIOA

//配置网络参数
void W5500_Network_Init(void)
{
    uint8_t chipid[6];
    wiz_NetInfo gWIZNETINFO;
    wiz_NetTimeout w_NetTimeout;


    uint8_t mac[6] = {0x00, 0x08, 0xdc, 0x11, 0x11, 0x11}; ///< Source Mac Address
    uint8_t ip[4] = {192, 168, 50, 88}; ///< Source IP Address
    uint8_t sn[4] = {255, 255, 255, 0}; ///< Subnet Mask
    uint8_t gw[4] = {192, 168, 50, 1}; ///< Gateway IP Address
    uint8_t dns[4] = {8, 8, 8, 8}; ///< DNS server IP Address

    memcpy(gWIZNETINFO.ip, ip, 4);
    memcpy(gWIZNETINFO.sn, sn, 4);
    memcpy(gWIZNETINFO.gw, gw, 4);
    memcpy(gWIZNETINFO.mac, mac, 6);
    memcpy(gWIZNETINFO.dns, dns, 4);
    gWIZNETINFO.dhcp = NETINFO_STATIC; //< 1 - Static, 2 - DHCP
    ctlnetwork(CN_SET_NETINFO, (void *) &gWIZNETINFO);

    ctlnetwork(CN_GET_NETINFO, (void *) &gWIZNETINFO);
// Display Network Information
    ctlwizchip(CW_GET_ID, (void *) chipid);
    printf("\r\n=== %s NET CONF ===\r\n", (char *) chipid);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2],
           gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
    printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
    printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
    printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
    printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
    printf("======================\r\n");

    wizchip_init(NULL, NULL);


    w_NetTimeout.retry_cnt = 50;
    w_NetTimeout.time_100us = 1000;
    wizchip_settimeout(&w_NetTimeout);
}

//SPI写入函数
void SPI1WriteByte(uint8_t TxData)
{
    while ((SPI1->SR & SPI_I2S_FLAG_TXE) == 0);     //等待发送区空
    SPI1->DR = TxData;                                       //发送一个byte
    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);     //等待接收完一个byte
    SPI1->DR;
}
//SPI读取函数
uint8_t SPI1ReadByte(void)
{
    while ((SPI1->SR & SPI_I2S_FLAG_TXE) == 0);     //等待发送区空
    SPI1->DR = 0xFF;                                             //发送一个空数据产生输入数据的时钟
    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);     //等待接收完一个byte
    return SPI1->DR;
}
//片选函数
void SPI1_CS_Select(void)
{
    GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);
}
//片选取消函数
void SPI1_CS_Deselect(void)
{
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);
}
//进入临界区函数
void SPI_CrisEnter(void)
{
    __set_PRIMASK(1);
}
//退出临界区函数
void SPI_CrisExit(void)
{
    __set_PRIMASK(0);
}
/*
* WIZ_SPI_Init
* SPI接口初始化
* 硬件连接: 
* PA3 -> W5500_RST
* PA4 -> W5500_SCS
* PA5 -> W5500_SCK
* PA6 -> W5500_MISO
* PA7 -> W5500_MOSI
* PA8 -> W5500_INT
*/
static void W5500_SPI_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef       SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);



    /* 初始化CS引脚 */
    GPIO_InitStructure.GPIO_Pin = W5500_SCS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

    /* W5500_RST引脚初始化配置 */
    GPIO_InitStructure.GPIO_Pin  = W5500_RST;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
    GPIO_SetBits(W5500_RST_PORT, W5500_RST);

    /* 初始化SCK、MISO、MOSI引脚 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    /* 初始化配置STM32 SPI1 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                         //设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                     //SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                            //时钟悬空低
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                          //数据捕获于第1个时钟沿
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                             //NSS由外部管脚管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;    //波特率预分频值为2
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                    //数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;                              //CRC多项式为7
    SPI_Init(SPI1, &SPI_InitStructure);                                    //根据SPI_InitStruct中指定的参数初始化外设SPI1寄存器

    SPI_Cmd(SPI1, ENABLE);     //STM32使能SPI1
}

void W5500Register(void)
{
    // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP
    /* Critical section callback */
    reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);    //注册临界区函数
    /* Chip selection call back */
    reg_wizchip_cs_cbfunc(SPI1_CS_Select, SPI1_CS_Deselect);    //注册SPI片选信号函数
    reg_wizchip_spi_cbfunc(SPI1ReadByte, SPI1WriteByte);    //注册读写函数

}

void W5500_Init(void)
{
    uint16_t i;
    W5500_SPI_Init();

    for (i = 0x5FFF; i > 0; i--) {}                       // 短暂延时,不然导致初始化失败

    W5500Register();

    W5500_Network_Init();
}


int main(void)
{
    uint8_t dest_ip[4] = {192, 168, 50, 100};
    uint16_t dest_port = 8000;

    uint8_t dataBuffer[DATA_BUF_SIZE] = {0x0};

    delay_init();                                       //延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     // 设置中断优先级分组2
    uart_init(115200);                                  //串口初始化为115200
    printf("usart inited ......");
    LED_Init();                                         //初始化与LED连接的硬件接口
    printf("LED inited ......");
    W5500_Init();
    printf("W5500 inited ......");


    /* Infinite loop */
    while (1)
    {
        loopback_tcpc(0x0, dataBuffer, dest_ip, dest_port);
        LED0 = 0;
        delay_ms(50);
        LED0 = 1;
        delay_ms(50);
    }
}

