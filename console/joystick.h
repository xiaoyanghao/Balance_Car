/********************************************************************************
** Form generated from reading UI file 'ui_parm.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QByteArray>
#include <QString>
#include <QObject>

/**
 * @brief  虚拟摇杆类
 */
class JOYSTICK : public QObject
{
public:
   float jokstick_getx(void);
   float jokstick_gety(void);

public slots:
   void jokstick_setxy(qreal px, qreal py);

private:
    float       x;
    float       y;


};








#endif
