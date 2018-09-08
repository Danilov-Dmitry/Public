/*==========================================================================================================================================
 *                                                            File Header
 *==========================================================================================================================================
 * Name File    : UART.c
 * Project Name : RTOS
 * Autor        : Danilov Dmitry
 * E-mail       : DanilovDmitry72@ya.ru
 * Company      : 
 * Start design : 5 Август 2018 г. 14:59:17
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
} UartTimer [2];                                              // Массив програмных таймеров UART

BUFF_CREATE (UART_PROG_TX, RxPackLen - 1);                    // Создание кольцевого буффера для приема данных через UART
BUFF_CREATE (UART_PROG_RX, RxPackLen - 1);                    // Создание кольцевого буффера для передачи данных через UART

T_UART_MESS UartMess;                                         // Структура состояния UART

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
// Инициализация портов UART
    UART_DDR |= (1<<UART_TX);
    UART_DDR &= ~(1<<UART_RX);
    UART_PORT |= (1<<UART_TX);

// Инициализация таймера UART
    TCCR2 = 0<<WGM21|0<<WGM20|0<<COM21|0<<COM20|0<<CS22|0<<CS21|0<<CS20;// предделитель 8, таймер пока остановлен
    TCNT2 = (uint8_t) Timer2_TCNT2;
    TIMSK |= 1<<TOIE2;
    //TIMSK &= ~(1<<TOIE2);

    UartTimer[0].Time = 0;
    UartTimer[1].Time = 0;
    UartTimer[0].Task = uart_tx;
    UartTimer[1].Task = uart_rx;

// Инициализация внешнего прерывания INT0 по спаду
    MCUCR = (1<<ISC01)|(0<<ISC00);
    GICR = (1<<INT0);

// Инициализация состояния UART
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
    static T_UART_FSM_STATE UartStateTx = START_BIT;          // Состояние конечного автомата
    static uint8_t          buffer;                           // Промежуточный буффер в данными, подлежащими отправке
    static uint8_t          CounterTxBit;                     // Счетчик отправленных битов
    static uint8_t          CRC = 0xFF;                       // CRC

    static struct                                             // Структура флагов
    {
        uint8_t CRC     :1;// Флаг отправки CRC
        uint8_t PARITY  :1;// Бит четности
    } Flags = {0};

    switch (UartStateTx)                                      // Реализация конечного автомата
    {
        case (START_BIT):                                     // Передача старт-бита
        {
            UartTimer[0].Time = UART_T;                       // Переустановка програмного таймера на передачу UART

            UART_PORT &= ~(1<<UART_TX);                       // Сбрасываем линию TX

            CounterTxBit = 0;                                 // Обнуляем счетчик переданных битов
            buffer = buff_read_byte(&UART_PROG_TX);              // Получаем передаваемый байт из буффера

            CRC ^= buffer;                                    // Загружаем новое значение для вычисления CRC

            UartStateTx = DATA;                               // Переключаем конечный автомат на передачу данных

        }; break;

        case (DATA):                                          // Передача битов данных
        {
           UartTimer[0].Time = UART_T;

           CRC = CRC & 0x80 ? (CRC << 1)^0x1D : (CRC<<1);     // Вычисляем CRC  одновременно с процессом передачи битов данных

           UART_PORT &= ~(1<<UART_TX);
           UART_PORT |= ((buffer & 0x01)<<UART_TX);

           Flags.PARITY ^= (buffer & 0x01);                   // Вычисляем бит четности (в данным варианте идет проверка на ЧЕТНОСТЬ)

           buffer = buffer>>1;

            if ((++CounterTxBit) == 8)                        // Если все биты данных переданы, то переходим на передачу бита четности
            {
                UartStateTx=PARITY;

                if ((UART_PROG_TX.status.empty == ON) & (Flags.CRC == OFF))// Если буффер пуст, то передаем байт CRC
                {
                    buff_wr_byte (&UART_PROG_TX, CRC ^ 0xFF);
                    Flags.CRC = ON;
                }
            }
        }; break;

        case (PARITY):                                        // Передача бита четности
        {
            UartTimer[0].Time = UART_T;

            UART_PORT &= ~(1<<UART_TX);
            UART_PORT |= ((Flags.PARITY)<<UART_TX);

            UartStateTx=STOP_BIT;

        }; break;

        case (STOP_BIT):                                      // Передача стоп-бита
        {
            UART_PORT |= (1<<UART_TX);
             Flags.PARITY = 0;

            if (UART_PROG_TX.status.empty == ON)              // Если буффер пуст, то
            {
                UartTimer[0].Time = 0;                        // Отключаем программный таймер процедуры передачи
                CRC = 0xFF;                                   // Записываем начальное значение CRC
                Flags.CRC = OFF;                              // Сбрасываем флаг отправки CRC
                                             // Записываем начальное значение бита четности
            }
            else                                              // иначе переустанавливаем программный таймер
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
//     static T_UART_FSM_STATE UartStateRx = START_BIT;          // Состояние конечного автомата
//     static uint8_t buffer;                                    // Промежуточный буффер
//     static uint8_t PackLen =0; //RxPackLen;                       // Длина принимаемой посылки
//     static uint8_t CounterRxBit;                              // Счетчик принятых битов
//     static uint8_t CRC = 0xFF;                                // СRC
//
//     static struct                                             //  Структура флагов
//     {
//         uint8_t CRC     :1;//  Прием CRC
//         uint8_t PARITY  :1;// Бит четности
//     } Flags = {0};
//
//     uint8_t InData;                                           // Принимаемый бит
//     InData = ((UART_PIN & (1<<UART_RX))>>UART_RX);            // Считывание принимаемого бита с порта UART
//
//     switch (UartStateRx)                                      // Реализация конечного автомата
//     {
//         case (START_BIT):                                     // Прием старт-бита
//         {
//             if (InData == 1)                                  // Если линия RX в состоянии "1", то пришла помеха, а не данные
//             {
//                 UartTimer[1].Time = 0;                        // Выключаем програмный таймер
//                 GICR |= (1<<INT0);                            // Включаем внешнее прерывание
//                 break;
//             }
//
//             UartTimer[1].Time = UART_T;                       // Переустанавливаем програмный таймер процедуры приема
//
//             UartStateRx = DATA;                               // Переключаем конечный автомат на прием данных
//
//             UartMess.RxComplete =   OFF;                      // Прием не завершен!
//
//             CounterRxBit    = 0;                              // Обнуляем счетчик принятых битов данных
//             buffer          = 0;                              // Обнуляем буффер
//             UartMess.ParityError = OFF;
//         }; break;
//
//         case (DATA):                                          // Прием данных
//         {
//             UartTimer[1].Time = UART_T;
//
//             Flags.PARITY ^= InData;                           //  Подсчитываем бит четности
//
//             if (PackLen != 0)                                 // Если не первый байт принимаем, то
//             {
//                 CRC = (CRC & 0x80) ? (CRC << 1)^0x1D : (CRC<<1);    // вычисляем CRC
//             }
//
//             buffer |= InData<<CounterRxBit;                   // Записываем принятый бит в промежуточный буффер
//
//             if ((++CounterRxBit) == 8)                        // Если приняли все биты, то
//             {
//                 UartStateRx = PARITY;                         // Переходим на принятие бита четности
//                 UartTimer[1].Time = UART_T;                   // Переустанавливаем програмный таймер
//                 break;
//             }
//         }; break;
//
//         case (PARITY):
//         {
//             UartTimer[1].Time = UART_T;
//
//             if (InData != Flags.PARITY)                       // Если бит четности не совпадает, то ошибка четности
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
//             if ((++PackLen) == RxPackLen)                               // Если принята вся посылка, то
//             {
//                 UartMess.DataValid  =  ((buffer == (CRC^0xFF)) & (UartMess.ParityError == OFF)) ? ON : OFF;// Сравниваем CRC
//                 UartMess.RxComplete =   ON;                   // Выставляем флаг завершения приема
//                 PackLen = 0; //RxPackLen;                          // Задаем длину принимаемой посылки
//                 CRC = 0xFF;                                   // Записываем начальное значение CRC
//
//                 SetTask (uart_analisis, low);                 // Ставим в очередь задачу анализа принятой посылки
//
//                 return ;
//             }
//
//             CRC ^= buffer;                                    // Загружаем новое значение для вычисления CRC
//
//             buff_wr_byte (&UART_PROG_RX,buffer);              // Сохраняем принятый байт в буффере
//
//         }; break;
//
//         default: break;
//     }
// }



//-------------------------------------------------------------------
void uart_rx (void)
{
    static T_UART_FSM_STATE UartStateRx = START_BIT;          // Состояние конечного автомата
    static uint8_t buffer;                                    // Промежуточный буффер
    static uint8_t CounterRxBit;                              // Счетчик принятых битов
    static uint8_t CRC = 0xFF;                                // СRC

    static struct                                             //  Структура флагов
    {
        uint8_t CRC     :1;//  Прием CRC
        uint8_t PARITY  :1;// Бит четности
    } Flags = {0};

    uint8_t InData;                                           // Принимаемый бит
    InData = ((UART_PIN & (1<<UART_RX))>>UART_RX);            // Считывание принимаемого бита с порта UART

    switch (UartStateRx)                                      // Реализация конечного автомата
    {
        case (START_BIT):                                     // Прием старт-бита
        {
            if (InData == 1)                                  // Если линия RX в состоянии "1", то пришла помеха, а не данные
            {
                UartTimer[1].Time = 0;                        // Выключаем програмный таймер
                GICR |= (1<<INT0);                            // Включаем внешнее прерывание
                break;
            }

            UartTimer[1].Time = UART_T;                       // Переустанавливаем програмный таймер процедуры приема

            UartStateRx = DATA;                               // Переключаем конечный автомат на прием данных

            UartMess.RxComplete =   OFF;                      // Прием не завершен!

            CounterRxBit    = 0;                              // Обнуляем счетчик принятых битов данных
            buffer          = 0;                              // Обнуляем буффер
            UartMess.ParityError = OFF;
        }; break;

        case (DATA):                                          // Прием данных
        {
            UartTimer[1].Time = UART_T;

            Flags.PARITY ^= InData;                           //  Подсчитываем бит четности
            CRC = (CRC & 0x80) ? (CRC << 1)^0x1D : (CRC<<1);   // вычисляем CRC

            if (UART_PROG_RX.status.empty == ON)
            {
                CRC = 0xFF;
            }

            buffer |= InData<<CounterRxBit;                   // Записываем принятый бит в промежуточный буффер

            if ((++CounterRxBit) == 8)                        // Если приняли все биты, то
            {
                UartStateRx = PARITY;                         // Переходим на принятие бита четности
                UartTimer[1].Time = UART_T;                   // Переустанавливаем програмный таймер
                break;
            }
        }; break;

        case (PARITY):
        {
            UartTimer[1].Time = UART_T;

            if (InData != Flags.PARITY)                       // Если бит четности не совпадает, то ошибка четности
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

            if (UART_PROG_RX.status.full == ON)               // Если принята вся посылка, то
            {
                UartMess.DataValid  =  ((buffer == (CRC^0xFF)) & (UartMess.ParityError == OFF)) ? ON : OFF;// Сравниваем CRC
                UartMess.RxComplete =   ON;                   // Выставляем флаг завершения приема
                CRC = 0xFF;                                   // Записываем начальное значение CRC

                SetTask (uart_analisis, low);                 // Ставим в очередь задачу анализа принятой посылки

                return ;
            }

            CRC ^= buffer;                                    // Загружаем новое значение для вычисления CRC

            buff_wr_byte (&UART_PROG_RX,buffer);              // Сохраняем принятый байт в буффере

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


    if (UartMess.DataValid == ON)                             // Если данные валидны, то начинаем их анализ, иначе запрашиваем данные снова
    {
        switch (buff_read_byte(&UART_PROG_RX))
        {
            case (UART_DATA):                                 // Пришли данные, записываем их в EEPROM и RAM
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

            case (UART_RES):                                  //  Пришел результат вычислений
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
                SetTask (com_task, low);                      // Выводим результат вычислений
                }

            }; break;

            case (UART_SERVICE):                              // Пришло сервисное сообщение
            {
                if (buff_read_byte(&UART_PROG_RX) == 0x01)    // Невалидные данные, необходима повторная отправка
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

    BUFF_CLR(UART_PROG_RX);                                   // Очистка буффера для приема
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
    GICR &= ~(1<<INT0);                                       // Отключаем данное прерывание
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
    TCNT2 = (uint8_t) Timer2_TCNT2;                           // Перезапускаем таймер2

    if ( UartTimer[0].Time != 0)                              // Если таймер не выключен, то декрементируем его, или же выполняем задачу
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

    if ( UartTimer[1].Time != 0)                              // Если таймер не выключен, то декрементируем его, или же выполняем задачу
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