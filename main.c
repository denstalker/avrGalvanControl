#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#define BAUDRATE 9600 // Скорость обмена данными
#define F_CPU 8000000UL // Рабочая частота контроллера
#define LEN 32

void sendData(char*);

int i;
int lng;
uint8_t status[]={0x01,
                  0x0FF,
                  0x01,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0x02,
                  0xFF,
                  0x01,
                  0xFF};
char rc;

//Подпрограмма-обработчик прерывания
//по приходу байта на модуль USART
ISR(USART_RXC_vect)
{
    //int b;

    DDRC = 0xFF;
    PORTC = 0xFF;

    OCR1A = UDR;//Присваиваем переменной b содержимое регистра UDR именно в нем хранится принятый байт
    status[15] = UDR;

    //Программа обработки принятого байта
}



void initPWM (void)
{
    TCCR1A=(1<<COM1A1)|(1<<WGM10); //На выводе OC1A единица, когда OCR1A==TCNT1, восьмибитный ШИМ
    TCCR1B=(1<<CS10);		 //Делитель= /1
    OCR1A=0x00;			//Начальная яркость нулевая

}

void initUART()
{
    // Параметры соединения: 8 бит данные, 1 стоповый бит, нет контроля четности
    // USART Приемник: Включен
    // USART Передатчик: Включен
    // USART Режим: Асинхронный
    // USART Скорость обмена: 9600

    UBRRL = (F_CPU/BAUDRATE/16-1); // Вычисляем скорость обмена данными
    UBRRH = (F_CPU/BAUDRATE/16-1) >> 8;

    UCSRB |= (1 << RXCIE)| // Разрешаем прерывание по завершению приема данных
            (1 << RXEN)|(1 << TXEN); // Включаем приемник и передатчик

    UCSRC |= (1 << URSEL)| // Для доступа к регистру UCSRC выставляем бит URSEL
            (1 << UCSZ1)|(1 << UCSZ0); // Размер посылки в кадре 8 бит
}

void main(void)
{
    // DDRC = 0xFF;
    //PORTC = 0xFF;
    //_delay_ms(1000);
    //PORTC = 0x00;
    DDRB=0x02;   			//Инициализация PB1 (OC1A) как выход

    initUART();
    initPWM();
    sei();

    lng=strlen(status);





    while (1)
    {
                if ((UCSRA & (1<<7)))
                {
                    rc=UDR;

                }





        sendData(status);
        _delay_ms(5);


        //        UDR=OCR1A;
        //        OCR1A++;

        //        UDR = OCR1A;



        //        PORTC = 0x00;

    }

}

void sendData(char* a)
{

    //    _delay_ms(50);
    for (i=0; i < lng; i++)    {
        while ( !( UCSRA & (1<<5)) ) {}
        UDR=a[i];
    }



}
