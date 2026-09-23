/********************************************************************************
** Form generated from reading UI file 'ui_parm.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef CONSOLE_SETUP_H
#define CONSOLE_SETUP_H
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QUdpSocket>
#include <QFile>

#include "console_protocol.h"
#include "joystick.h"
#include "ui_ui_parm.h"




class CONSOLE : public QObject,public CONSOLE_PROTOCAL
{
 Q_OBJECT
public:
   explicit CONSOLE(QMainWindow *User_MainWindow = nullptr,class Ui_Parm *User_parm = nullptr);
   virtual ~CONSOLE();

   class Ui_Parm  *user_ui;
   QMainWindow    *user_mw;
   QSerialPort    user_port;
   QUdpSocket     udpSocket;
   unsigned char  udp_connet;/*0-未建立连接    1-建立连接*/
   int            local_port;
   QString        taget_ip;
   int            taget_port;
   unsigned char  link_type; /*1-串口         2-UPD*/

   QObject        *joystick_qmlObj;


   QTimer      *user_timer;
   QTimer      *user_timer1;
   QTimer      *user_timer2;
   QTimer      *user_timer3;
   QTimer      *user_timer4;
   QFile       *imu_file;
   QFile       *control_file;

private slots:
   void btn_port_check(void);
   void btn_port_open(void);
   void btn_port_close(void);
   void port_read(const QByteArray data);
   
   void net_read(void);
   void net_trytoconnet(void);
   void btn_net_connet(void);
   void btn_net_disconnet(void);

   void btn_control_dug_set(void);
   void btn_control_dug_req(void);
   void btn_control_dug_apply(void);

   void data_update(void);
   void imu_record(void);
   void control_record(void);

   void joystick_setxy(qreal px, qreal py);
   void joystick_sendvalue(void);

private :
   unsigned char imu_file_open = 0;
   unsigned char control_file_open = 0;
   float  joystick_x = 0.5;
   float  joystick_y = 0.5;


};



#endif // UI_UI_PARM_H
