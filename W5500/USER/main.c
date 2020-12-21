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

//�����������
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

//SPIд�뺯��
void SPI1WriteByte(uint8_t TxData)
{
    while ((SPI1->SR & SPI_I2S_FLAG_TXE) == 0);     //�ȴ���������
    SPI1->DR = TxData;                                       //����һ��byte
    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);     //�ȴ�������һ��byte
    SPI1->DR;
}
//SPI��ȡ����
uint8_t SPI1ReadByte(void)
{
    while ((SPI1->SR & SPI_I2S_FLAG_TXE) == 0);     //�ȴ���������
    SPI1->DR = 0xFF;                                             //����һ�������ݲ����������ݵ�ʱ��
    while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == 0);     //�ȴ�������һ��byte
    return SPI1->DR;
}
//Ƭѡ����
void SPI1_CS_Select(void)
{
    GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);
}
//Ƭѡȡ������
void SPI1_CS_Deselect(void)
{
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);
}
//�����ٽ�������
void SPI_CrisEnter(void)
{
    __set_PRIMASK(1);
}
//�˳��ٽ�������
void SPI_CrisExit(void)
{
    __set_PRIMASK(0);
}
/*
* WIZ_SPI_Init
* SPI�ӿڳ�ʼ��
* Ӳ������: 
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



    /* ��ʼ��CS���� */
    GPIO_InitStructure.GPIO_Pin = W5500_SCS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

    /* W5500_RST���ų�ʼ������ */
    GPIO_InitStructure.GPIO_Pin  = W5500_RST;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
    GPIO_SetBits(W5500_RST_PORT, W5500_RST);

    /* ��ʼ��SCK��MISO��MOSI���� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    /* ��ʼ������STM32 SPI1 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                         //����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                     //SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                            //ʱ�����յ�
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                          //���ݲ����ڵ�1��ʱ����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                             //NSS���ⲿ�ܽŹ���
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;    //������Ԥ��ƵֵΪ2
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                    //���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;                              //CRC����ʽΪ7
    SPI_Init(SPI1, &SPI_InitStructure);                                    //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPI1�Ĵ���

    SPI_Cmd(SPI1, ENABLE);     //STM32ʹ��SPI1
}

void W5500Register(void)
{
    // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP
    /* Critical section callback */
    reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);    //ע���ٽ�������
    /* Chip selection call back */
    reg_wizchip_cs_cbfunc(SPI1_CS_Select, SPI1_CS_Deselect);    //ע��SPIƬѡ�źź���
    reg_wizchip_spi_cbfunc(SPI1ReadByte, SPI1WriteByte);    //ע���д����

}

void W5500_Init(void)
{
    uint16_t i;
    W5500_SPI_Init();

    for (i = 0x5FFF; i > 0; i--) {}                       // ������ʱ,��Ȼ���³�ʼ��ʧ��

    W5500Register();

    W5500_Network_Init();
}


int main(void)
{
    uint8_t dest_ip[4] = {192, 168, 50, 100};
    uint16_t dest_port = 8000;

    uint8_t dataBuffer[DATA_BUF_SIZE] = {0x0};

    delay_init();                                       //��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     // �����ж����ȼ�����2
    uart_init(115200);                                  //���ڳ�ʼ��Ϊ115200
    printf("usart inited ......");
    LED_Init();                                         //��ʼ����LED���ӵ�Ӳ���ӿ�
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

