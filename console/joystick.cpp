#include "joystick.h"
#include <QDebug>

float JOYSTICK:: jokstick_getx(void)
{
     return x;
}


float JOYSTICK:: jokstick_gety(void)
{
     return y;
}

void JOYSTICK:: jokstick_setxy(qreal px, qreal py)
{
     x = px;
     y = py;
     qDebug()<<"JOYSTICK::jokstick_setxy"<<x<<y;
}
