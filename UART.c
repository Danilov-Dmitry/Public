/*==========================================================================================================================================
 *                                                            File Header
 *==========================================================================================================================================
 * Name File    : UART.c
 * Project Name : RTOS
 * Autor        : Danilov Dmitry
 * E-mail       : DanilovDmitry72@ya.ru
 * Company      : 
 * Start design : 5 ������ 2018 �. 14:59:17
 * Last change  :
 *
 *******************************************************************************************************************************************
 *
 * File description:
 *
 *========================================================================================================================================*/

/*==========================================================================================================================================
 *                                                            Section of including files
 *========================================================================================================================================*/

#include <UART.h>

/*==========================================================================================================================================
 *                                                            Section of defining variables
 *========================================================================================================================================*/

static struct
{
    uint8_t Time;
    TPTR Task;
} UartTimer [2];                                              // ������ ���������� �������� UART

BUFF_CREATE (UART_PROG_TX, RxPackLen - 1);                    // �������� ���������� ������� ��� ������ ������ ����� UART
BUFF_CREATE (UART_PROG_RX, RxPackLen - 1);                    // �������� ���������� ������� ��� �������� ������ ����� UART

T_UART_MESS UartMess;                                         // ��������� ��������� UART

/*==========================================================================================================================================
 *                                                            Section of functions prototypes
 *========================================================================================================================================*/

void uart_init      (void);
void uart_tx        (void);
void uart_rx        (void);
void uart_analisis  (void);
/*==========================================================================================================================================
 *                                                            Section of function description
 *========================================================================================================================================*/

/*******************************************************************************************************************************************
 *
 *Function:   uart_init
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description:  Initialization of UART
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/

void uart_init (void)
{
// ������������� ������ UART
    UART_DDR |= (1<<UART_TX);
    UART_DDR &= ~(1<<UART_RX);
    UART_PORT |= (1<<UART_TX);

// ������������� ������� UART
    TCCR2 = 0<<WGM21|0<<WGM20|0<<COM21|0<<COM20|0<<CS22|0<<CS21|0<<CS20;// ������������ 8, ������ ���� ����������
    TCNT2 = (uint8_t) Timer2_TCNT2;
    TIMSK |= 1<<TOIE2;
    //TIMSK &= ~(1<<TOIE2);

    UartTimer[0].Time = 0;
    UartTimer[1].Time = 0;
    UartTimer[0].Task = uart_tx;
    UartTimer[1].Task = uart_rx;

// ������������� �������� ���������� INT0 �� �����
    MCUCR = (1<<ISC01)|(0<<ISC00);
    GICR = (1<<INT0);

// ������������� ��������� UART
    UartMess.DataValid      = ON;
    UartMess.RxComplete     = OFF;
    UartMess.TxComplete     = ON;
    UartMess.ParityError    = OFF;
}

/*******************************************************************************************************************************************
 *
 *Function: uart_tx
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description: Function of transmitting data on UART from cycle buffer with calculation CRC8_SAE_J1850
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/


void uart_tx (void)
{
    static T_UART_FSM_STATE UartStateTx = START_BIT;          // ��������� ��������� ��������
    static uint8_t          buffer;                           // ������������� ������ � �������, ����������� ��������
    static uint8_t          CounterTxBit;                     // ������� ������������ �����
    static uint8_t          CRC = 0xFF;                       // CRC

    static struct                                             // ��������� ������
    {
        uint8_t CRC     :1;// ���� �������� CRC
        uint8_t PARITY  :1;// ��� ��������
    } Flags = {0};

    switch (UartStateTx)                                      // ���������� ��������� ��������
    {
        case (START_BIT):                                     // �������� �����-����
        {
            UartTimer[0].Time = UART_T;                       // ������������� ����������� ������� �� �������� UART

            UART_PORT &= ~(1<<UART_TX);                       // ���������� ����� TX

            CounterTxBit = 0;                                 // �������� ������� ���������� �����
            buffer = buff_read_byte(&UART_PROG_TX);              // �������� ������������ ���� �� �������

            CRC ^= buffer;                                    // ��������� ����� �������� ��� ���������� CRC

            UartStateTx = DATA;                               // ����������� �������� ������� �� �������� ������

        }; break;

        case (DATA):                                          // �������� ����� ������
        {
           UartTimer[0].Time = UART_T;

           CRC = CRC & 0x80 ? (CRC << 1)^0x1D : (CRC<<1);     // ��������� CRC  ������������ � ��������� �������� ����� ������

           UART_PORT &= ~(1<<UART_TX);
           UART_PORT |= ((buffer & 0x01)<<UART_TX);

           Flags.PARITY ^= (buffer & 0x01);                   // ��������� ��� �������� (� ������ �������� ���� �������� �� ��������)

           buffer = buffer>>1;

            if ((++CounterTxBit) == 8)                        // ���� ��� ���� ������ ��������, �� ��������� �� �������� ���� ��������
            {
                UartStateTx=PARITY;

                if ((UART_PROG_TX.status.empty == ON) & (Flags.CRC == OFF))// ���� ������ ����, �� �������� ���� CRC
                {
                    buff_wr_byte (&UART_PROG_TX, CRC ^ 0xFF);
                    Flags.CRC = ON;
                }
            }
        }; break;

        case (PARITY):                                        // �������� ���� ��������
        {
            UartTimer[0].Time = UART_T;

            UART_PORT &= ~(1<<UART_TX);
            UART_PORT |= ((Flags.PARITY)<<UART_TX);

            UartStateTx=STOP_BIT;

        }; break;

        case (STOP_BIT):                                      // �������� ����-����
        {
            UART_PORT |= (1<<UART_TX);
             Flags.PARITY = 0;

            if (UART_PROG_TX.status.empty == ON)              // ���� ������ ����, ��
            {
                UartTimer[0].Time = 0;                        // ��������� ����������� ������ ��������� ��������
                CRC = 0xFF;                                   // ���������� ��������� �������� CRC
                Flags.CRC = OFF;                              // ���������� ���� �������� CRC
                                             // ���������� ��������� �������� ���� ��������
            }
            else                                              // ����� ����������������� ����������� ������
            {
                UartTimer[0].Time = UART_T;
            }

            UartStateTx=START_BIT;

        }; break;

        default: break;
    }
}

/*******************************************************************************************************************************************
 *
 *Function: uart_rx
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description: Function of receiving data with calculation CRC8_SAE_J1850
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/

// void uart_rx (void)
// {
//     static T_UART_FSM_STATE UartStateRx = START_BIT;          // ��������� ��������� ��������
//     static uint8_t buffer;                                    // ������������� ������
//     static uint8_t PackLen =0; //RxPackLen;                       // ����� ����������� �������
//     static uint8_t CounterRxBit;                              // ������� �������� �����
//     static uint8_t CRC = 0xFF;                                // �RC
//
//     static struct                                             //  ��������� ������
//     {
//         uint8_t CRC     :1;//  ����� CRC
//         uint8_t PARITY  :1;// ��� ��������
//     } Flags = {0};
//
//     uint8_t InData;                                           // ����������� ���
//     InData = ((UART_PIN & (1<<UART_RX))>>UART_RX);            // ���������� ������������ ���� � ����� UART
//
//     switch (UartStateRx)                                      // ���������� ��������� ��������
//     {
//         case (START_BIT):                                     // ����� �����-����
//         {
//             if (InData == 1)                                  // ���� ����� RX � ��������� "1", �� ������ ������, � �� ������
//             {
//                 UartTimer[1].Time = 0;                        // ��������� ���������� ������
//                 GICR |= (1<<INT0);                            // �������� ������� ����������
//                 break;
//             }
//
//             UartTimer[1].Time = UART_T;                       // ����������������� ���������� ������ ��������� ������
//
//             UartStateRx = DATA;                               // ����������� �������� ������� �� ����� ������
//
//             UartMess.RxComplete =   OFF;                      // ����� �� ��������!
//
//             CounterRxBit    = 0;                              // �������� ������� �������� ����� ������
//             buffer          = 0;                              // �������� ������
//             UartMess.ParityError = OFF;
//         }; break;
//
//         case (DATA):                                          // ����� ������
//         {
//             UartTimer[1].Time = UART_T;
//
//             Flags.PARITY ^= InData;                           //  ������������ ��� ��������
//
//             if (PackLen != 0)                                 // ���� �� ������ ���� ���������, ��
//             {
//                 CRC = (CRC & 0x80) ? (CRC << 1)^0x1D : (CRC<<1);    // ��������� CRC
//             }
//
//             buffer |= InData<<CounterRxBit;                   // ���������� �������� ��� � ������������� ������
//
//             if ((++CounterRxBit) == 8)                        // ���� ������� ��� ����, ��
//             {
//                 UartStateRx = PARITY;                         // ��������� �� �������� ���� ��������
//                 UartTimer[1].Time = UART_T;                   // ����������������� ���������� ������
//                 break;
//             }
//         }; break;
//
//         case (PARITY):
//         {
//             UartTimer[1].Time = UART_T;
//
//             if (InData != Flags.PARITY)                       // ���� ��� �������� �� ���������, �� ������ ��������
//             {
//                 UartMess.ParityError = ON;
//             }
//
//             UartStateRx=STOP_BIT;
//         }; break;
//
//         case (STOP_BIT):
//         {
//             GICR |= (1<<INT0);
//
//             UartStateRx = START_BIT;
//
//             UartTimer[1].Time = 0;
//
//             Flags.PARITY = 0;
//
//             if ((++PackLen) == RxPackLen)                               // ���� ������� ��� �������, ��
//             {
//                 UartMess.DataValid  =  ((buffer == (CRC^0xFF)) & (UartMess.ParityError == OFF)) ? ON : OFF;// ���������� CRC
//                 UartMess.RxComplete =   ON;                   // ���������� ���� ���������� ������
//                 PackLen = 0; //RxPackLen;                          // ������ ����� ����������� �������
//                 CRC = 0xFF;                                   // ���������� ��������� �������� CRC
//
//                 SetTask (uart_analisis, low);                 // ������ � ������� ������ ������� �������� �������
//
//                 return ;
//             }
//
//             CRC ^= buffer;                                    // ��������� ����� �������� ��� ���������� CRC
//
//             buff_wr_byte (&UART_PROG_RX,buffer);              // ��������� �������� ���� � �������
//
//         }; break;
//
//         default: break;
//     }
// }



//-------------------------------------------------------------------
void uart_rx (void)
{
    static T_UART_FSM_STATE UartStateRx = START_BIT;          // ��������� ��������� ��������
    static uint8_t buffer;                                    // ������������� ������
    static uint8_t CounterRxBit;                              // ������� �������� �����
    static uint8_t CRC = 0xFF;                                // �RC

    static struct                                             //  ��������� ������
    {
        uint8_t CRC     :1;//  ����� CRC
        uint8_t PARITY  :1;// ��� ��������
    } Flags = {0};

    uint8_t InData;                                           // ����������� ���
    InData = ((UART_PIN & (1<<UART_RX))>>UART_RX);            // ���������� ������������ ���� � ����� UART

    switch (UartStateRx)                                      // ���������� ��������� ��������
    {
        case (START_BIT):                                     // ����� �����-����
        {
            if (InData == 1)                                  // ���� ����� RX � ��������� "1", �� ������ ������, � �� ������
            {
                UartTimer[1].Time = 0;                        // ��������� ���������� ������
                GICR |= (1<<INT0);                            // �������� ������� ����������
                break;
            }

            UartTimer[1].Time = UART_T;                       // ����������������� ���������� ������ ��������� ������

            UartStateRx = DATA;                               // ����������� �������� ������� �� ����� ������

            UartMess.RxComplete =   OFF;                      // ����� �� ��������!

            CounterRxBit    = 0;                              // �������� ������� �������� ����� ������
            buffer          = 0;                              // �������� ������
            UartMess.ParityError = OFF;
        }; break;

        case (DATA):                                          // ����� ������
        {
            UartTimer[1].Time = UART_T;

            Flags.PARITY ^= InData;                           //  ������������ ��� ��������
            CRC = (CRC & 0x80) ? (CRC << 1)^0x1D : (CRC<<1);   // ��������� CRC

            if (UART_PROG_RX.status.empty == ON)
            {
                CRC = 0xFF;
            }

            buffer |= InData<<CounterRxBit;                   // ���������� �������� ��� � ������������� ������

            if ((++CounterRxBit) == 8)                        // ���� ������� ��� ����, ��
            {
                UartStateRx = PARITY;                         // ��������� �� �������� ���� ��������
                UartTimer[1].Time = UART_T;                   // ����������������� ���������� ������
                break;
            }
        }; break;

        case (PARITY):
        {
            UartTimer[1].Time = UART_T;

            if (InData != Flags.PARITY)                       // ���� ��� �������� �� ���������, �� ������ ��������
            {
                UartMess.ParityError = ON;
            }

            UartStateRx=STOP_BIT;
        }; break;

        case (STOP_BIT):
        {
            GICR |= (1<<INT0);

            UartStateRx = START_BIT;

            UartTimer[1].Time = 0;

            Flags.PARITY = 0;

            if (UART_PROG_RX.status.full == ON)               // ���� ������� ��� �������, ��
            {
                UartMess.DataValid  =  ((buffer == (CRC^0xFF)) & (UartMess.ParityError == OFF)) ? ON : OFF;// ���������� CRC
                UartMess.RxComplete =   ON;                   // ���������� ���� ���������� ������
                CRC = 0xFF;                                   // ���������� ��������� �������� CRC

                SetTask (uart_analisis, low);                 // ������ � ������� ������ ������� �������� �������

                return ;
            }

            CRC ^= buffer;                                    // ��������� ����� �������� ��� ���������� CRC

            buff_wr_byte (&UART_PROG_RX,buffer);              // ��������� �������� ���� � �������

        }; break;

        default: break;
    }
}
//------------------------------------------------------------------


/*******************************************************************************************************************************************
 *
 *Function: uart_analisis
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description: Analisis of received data on UART
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/

void uart_analisis (void)
{
    uint16_t    temp16;
    uint8_t     temp8;


    if (UartMess.DataValid == ON)                             // ���� ������ �������, �� �������� �� ������, ����� ����������� ������ �����
    {
        switch (buff_read_byte(&UART_PROG_RX))
        {
            case (UART_DATA):                                 // ������ ������, ���������� �� � EEPROM � RAM
            {
                ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
                {
                temp8  = buff_read_byte(&UART_PROG_RX);
                temp16 = ((uint16_t) (buff_read_byte(&UART_PROG_RX)<<8)| temp8);

                eeprom_write_word (&EepromNum[0], temp16);

                temp8  =  buff_read_byte(&UART_PROG_RX);
                temp16 = ((uint16_t) (buff_read_byte(&UART_PROG_RX)<<8)| temp8);

                eeprom_write_word (&EepromNum[1], temp16);

                temp8  = (uint8_t) buff_read_byte(&UART_PROG_RX);

                eeprom_write_byte (&EepromOper, temp8);

                eeprom_to_ram ();

                }

            }; break;

            case (UART_RES):                                  //  ������ ��������� ����������
            {
            ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
                {
                uint8_t i;

                uint32_t  temp32 = 0;

                for (i = 0; i < 4; i++)
                {
                    temp32 |= (((uint32_t) buff_read_byte(&UART_PROG_RX))<<(8*i));
                }
                Result = (int32_t) temp32;
                ComState = Disp;
                SetTask (com_task, low);                      // ������� ��������� ����������
                }

            }; break;

            case (UART_SERVICE):                              // ������ ��������� ���������
            {
                if (buff_read_byte(&UART_PROG_RX) == 0x01)    // ���������� ������, ���������� ��������� ��������
                {
                    ComState = Rx;
                    SetTask (com_task, low);
                }
            }; break;

            default:
            {
                ComState = RepData;
                SetTask (com_task, low);

            };break;

        }
    }
    else
    {
        ComState = RepData;
        SetTask (com_task, low);
    }

    BUFF_CLR(UART_PROG_RX);                                   // ������� ������� ��� ������
}

/*==========================================================================================================================================
 *                                                            Section of interrupt handlers
 *========================================================================================================================================*/

/*******************************************************************************************************************************************
 *
 *Function: INT0_vect
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description: Handler of  external  interrupt  INT0
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/

ISR (INT0_vect)
{
    GICR &= ~(1<<INT0);                                       // ��������� ������ ����������
    UartTimer[1].Time = UART_T_2;
}

/*******************************************************************************************************************************************
 *
 *Function: TIMER2_OVF_vect
 *
 *------------------------------------------------------------------------------------------------------------------------------------------
 *
 *description: Interrupt handler of  timer2 ovrflow
 *
 *parameters: void
 *
 *on return:  void
 *
 ******************************************************************************************************************************************/

ISR (TIMER2_OVF_vect)
{
    TCNT2 = (uint8_t) Timer2_TCNT2;                           // ������������� ������2

    if ( UartTimer[0].Time != 0)                              // ���� ������ �� ��������, �� �������������� ���, ��� �� ��������� ������
    {
         if (UartTimer[0].Time == 1)
         {
              (*UartTimer[0].Task) ();
         }
         else
         {
             UartTimer[0].Time --;
         }
    }

    if ( UartTimer[1].Time != 0)                              // ���� ������ �� ��������, �� �������������� ���, ��� �� ��������� ������
    {
         if (UartTimer[1].Time == 1)
         {
              (*UartTimer[1].Task) ();
         }
         else
         {
             UartTimer[1].Time --;
         }
    }
}


//
// uint8_t CRC8 (uint8_t *array)
// {
//     uint8_t Crc = 0xFF;
//     uint8_t Bit;
//     uint8_t Byte = 0;
//
//     while (Byte++ != 3)
//     {
//         Crc ^= *(array++);
//         for (Bit = 0; Bit < 8; Bit++)
//         {
//             Crc = Crc & 0x80 ? (Crc << 1)^0x1D : (Crc<<1);
//         }
//     }
//     Crc ^= 0xFF;
//     return Crc;
// }