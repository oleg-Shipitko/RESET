#ifndef _MY_GPIO_INCLUDED_
#define _MY_GPIO_INCLUDED_
#include "stm32f4xx.h"
//В ЭТОЙ СТАТЬЕ НУЖНО БОЛЬШЕ КОММЕНТОВ :)
//GPIO_CRL_CNF3
#define PORTA 0 
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
//Альтернативные функции
#define AF0  0
#define AF1  1
#define AF2  2 
#define AF3  3 
#define AF4  4
#define AF5  5
#define AF6  6
#define AF7  7
#define AF8  8
#define AF9  9
#define AF10 10
#define AF11 11
#define AF12 12
#define AF13 13
#define AF14 14
#define AF15 15

#define GPIO_BASE  (AHB1PERIPH_BASE + 0x0000)
#define GPIO       ((unsigned char *) GPIO_BASE)

//mode
#define INPUT       0
#define GENERAL     1
#define ALTERNATE   2
#define ANALOG      3

//type
#define PUSH_PULL   0
#define OPEN_DRAIN  1

//speed
#define LOW_S       0     //2 MHz
#define MEDIUM_S    1     //25 MHz 
#define FAST_S      2     //50 MHz 
#define HIGH_S      3     //100 MHz

//pull
#define NO_PULL_UP 0
#define PULL_UP    1
#define PULL_DOWN  2

//получение адреса PIN_PORT в памяти
#define pin_id(PIN_PORT, PIN) ( (PIN_PORT<<4)|PIN )
#define GPIO_offset 0x400 
#define GPIO_base(pin) (GPIO_BASE +(pin>>4)*GPIO_offset )
//Назначение пину значения
#define pin_mode(pin, mode) ((uint32_t) ( mode) <<((pin&0xF)<<1))
#define pin_type(pin, type) ((uint32_t) ( type) <<((pin&0xF)))
#define pin_speed(pin, speed) ((uint32_t) ( speed) <<((pin&0xF)<<1))
#define pin_pull(pin, pull) ((uint32_t) ( pull) <<((pin&0xF)<<1))
#define pin_af(pin,af) ((uint32_t) (af) <<((pin&0x7)<<2))
//очистка пина
#define clear_mode(pin) (*((uint32_t *) (GPIO_base(pin))) &= ~pin_mode(pin, 0x3))
#define clear_type(pin) (*((uint32_t *) (GPIO_base(pin)+ 0x04)) &= ~pin_type(pin, 0x1))
#define clear_speed(pin) (*((uint32_t *) (GPIO_base(pin) + 0x08)) &= ~pin_speed(pin, 0x3))
#define clear_pull(pin) (*((uint32_t *) (GPIO_base(pin) + 0x0C)) &= ~pin_pull(pin, 0x3))
//очиска всех регистров для пина
#define clear_conf(pin) (           \
             clear_mode(pin);       \
             clear_type(pin);       \
             clear_speed(pin);      \
             clear_pull(pin) ;      )
//вычисление адреса пина и установка значения
#define set_mode(pin,mode)  (*((uint32_t *) (GPIO_base(pin)))       |= pin_mode(pin, mode))
#define set_type(pin,type)  (*((uint32_t *) (GPIO_base(pin)+ 0x04)) |= pin_mode(pin, type))
#define set_speed(pin,speed)(*((uint32_t *) (GPIO_base(pin)+ 0x08)) |= pin_mode(pin, speed))
#define set_pull(pin,pull)  (*((uint32_t *) (GPIO_base(pin)+ 0x0C)) |= pin_mode(pin, pull)))
//установка всех значений
#define set_conf(pin, mode,type,speed,pull) (\
   set_mode(pin,mode);\
   set_type(pin,type); \
   set_speed(pin,speed);\
   set_pull(pin,pull); \
     )
//очистка и установкка значения
#define conf_mode(pin, mode)  {*((uint32_t *) (GPIO_base(pin)+0x00)) = \
                ((*((uint32_t *) (GPIO_base(pin)+0x00))&(~pin_mode(pin, 0x3)))|pin_mode(pin, mode));}
#define conf_type(pin, type)  {*((uint32_t *) (GPIO_base(pin)+0x04)) = \
  ((*((uint32_t *) (GPIO_base(pin)+0x04))&(~pin_type(pin, 0x1)))|pin_type(pin, type));}
#define conf_speed(pin,speed) {*((uint32_t *) (GPIO_base(pin)+0x08)) = \
  ((*((uint32_t *) (GPIO_base(pin)+0x08))&(~pin_speed(pin, 0x3)))|pin_speed(pin,speed));}
#define conf_pull(pin, pull)  {*((uint32_t *) (GPIO_base(pin)+0x0C)) = \
  ((*((uint32_t *) (GPIO_base(pin)+0x0C))&(~pin_pull(pin, 0x3)))|pin_pull(pin, pull));}
//групповая очистка и установка всех регистров
#define conf_pin(pin, mode, type, speed, pull) {\
conf_mode(pin, mode)  \
conf_type(pin, type)     \
conf_speed(pin, speed)   \
conf_pull(pin, pull)      }
//выбор альтернативной функции
#define conf_af(pin, af)  {*((uint32_t *) (GPIO_base(pin)+0x20+((pin&0x08)>>1))) = \
                ((*((uint32_t *) (GPIO_base(pin)+0x20+((pin&0x08)>>1)))&(~pin_af(pin,0xF)))|pin_af(pin,af));}
//установить 1
#define set_pin(pin) (*((uint32_t *)(GPIO_base(pin) + 0x18))= (uint32_t)1<<(pin&0xf))
//установить 0
#define reset_pin(pin)  (*((uint32_t *)(GPIO_base(pin) + 0x18))= (uint32_t)1<<((pin&0xf)+0x10))
//физическое значение на ножке
#define pin_val(pin) ((*((uint32_t *)(GPIO_base(pin) + 0x10)))&((uint32_t)1<<(pin&0xf)))
//состояние регистра вывода
#define pin_out(pin) ((*((uint32_t *)(GPIO_base(pin) + 0x14)))&((uint32_t)1<<(pin&0xf)))

#define EXTI_offset 0x04
#define AFIO_base(line) (AFIO_BASE+0x08 +(line>>2)*EXTI_offset )
#define AFIO_mode(pin,line) ((uint32_t) (pin&0xF) <<((line&0x3)<<2))
#define _EXTI(line)(*((uint32_t *)AFIO_base(line)))

#endif