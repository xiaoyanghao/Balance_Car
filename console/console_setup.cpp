#include "console_setup.h"
#include <QMessageBox>
#include <QTimer>
#include <QDir>
#include <QDateTime>
#include <QString>
#include <QtDebug>
#include <QQuickView>
#include <QQuickItem>

CONSOLE::CONSOLE(QMainWindow *User_MainWindow,class Ui_Parm *User_parm)
{
  user_ui = User_parm;
  user_mw = User_MainWindow;
  user_ui->setupUi(user_mw);
 /*********************协议数据初始**************************************************/
  memset(&protocol_rA, 0x00, sizeof(struct PROTOCOL_RA));
  memset(&protocol_rB, 0x00, sizeof(struct PROTOCOL_RB));
  memset(&protocol_rD, 0x00, sizeof(struct PROTOCOL_RD));
  memset(&protocol_tA, 0x00, sizeof(struct PROTOCO_TA));
  memset(&protocol_tB, 0x00, sizeof(struct PROTOCO_TB));

  data_freq = 0;
  data_freq_count = 0;
  link_type = 0;
  udp_connet = 0;
 /*********************定时器**************************************************/
   user_timer = new QTimer();
   user_timer->setInterval(25);
   user_timer->start();

   user_timer1 = new QTimer();
   user_timer1->setInterval(2000);
   user_timer1->start();

   user_timer2 = new QTimer();
   user_timer2->setInterval(50);
   user_timer2->start();

   user_timer3 = new QTimer();
   user_timer3->setInterval(25);
   user_timer3->start();

   user_timer4 = new QTimer();
   user_timer4->setInterval(100);
   user_timer4->start();

  //定时器 解析数据/LCD显示
   QObject::connect(user_timer,SIGNAL(timeout()), this, SLOT(data_update()));
  //定时器 IMU数据记录
   QObject::connect(user_timer1,SIGNAL(timeout()), this, SLOT(imu_record()));
  //定时器 控制参数数据记录
   QObject::connect(user_timer2,SIGNAL(timeout()), this, SLOT(control_record()));
  //定时器 控制参数数据记录
   QObject::connect(user_timer3,SIGNAL(timeout()), this, SLOT(net_trytoconnet()));
   //定时器 虚拟遥控功能通信
   QObject::connect(user_timer3,SIGNAL(timeout()), this, SLOT(joystick_sendvalue()));

/*********************串口操作**************************************************/
   user_ui->btn_close_serial->setEnabled(false);
   //串口刷新
   QObject::connect(user_ui->btn_serial_check, &QPushButton::clicked, this, [&]() {
      btn_port_check();
   });

   // 串口打开
   QObject::connect(user_ui->btn_open_serail, &QPushButton::clicked, this, [&]() {
      btn_port_open();
   });

   // 串口关闭
   QObject::connect(user_ui->btn_close_serial, &QPushButton::clicked, this, [&]() {
      btn_port_close();
   });

   //串口接收数据
   QObject::connect(&user_port, &QIODevice::readyRead, this, [&]() {
      port_read(user_port.readAll());
   });

  /*********************网络操作**************************************************/
   //目标IP初始化
   user_ui->le_taget_ip->setIP("192.168.4.1");
   user_ui->btn_net_disconnet->setEnabled(false);
   // 网络连接
   QObject::connect(user_ui->btn_net_connet, &QPushButton::clicked, this, [&]() {
      btn_net_connet();
   });
   // 网络断开
   QObject::connect(user_ui->btn_net_disconnet, &QPushButton::clicked, this, [&]() {
      btn_net_disconnet();
   });
   //网络接收数据
   QObject::connect(&udpSocket, &QIODevice::readyRead, this, [&]() {
      net_read();
   });

 /*********************控制参数调试**************************************************/
    //参数设置
    QObject::connect(user_ui->btn_control_dug_set, &QPushButton::clicked, this, [&]() {
       btn_control_dug_set();
    });

    //参数查询
    QObject::connect(user_ui->btn_control_dug_req, &QPushButton::clicked, this, [&]() {
       btn_control_dug_req();
    });

    //参数应用
    QObject::connect(user_ui->btn_control_dug_apply, &QPushButton::clicked, this, [&]() {
       btn_control_dug_apply();
    });
 /********************虚拟摇杆调试*********************************************************/
   joystick_qmlObj = (user_ui->stick_quickWidget->rootObject());
   QObject::connect(joystick_qmlObj,SIGNAL(qmlsendstick(qreal,qreal)),
                    this,SLOT(joystick_setxy(qreal,qreal)));


}

CONSOLE::~CONSOLE()
{
    delete user_mw;
    delete user_ui;
    delete user_timer;
    delete user_timer1;
    delete user_timer2;
    delete user_timer3;
}

/*********************串口操作**************************************************/
//串口检测
void CONSOLE::btn_port_check(void)
{
   user_ui->cmb_serial_port->clear();
   foreach(const QSerialPortInfo &info,QSerialPortInfo :: availablePorts())
   {
      user_ui->cmb_serial_port->addItem(info.portName());
   }
}

//打开串口
void CONSOLE:: btn_port_open(void)
{
    user_port.setPortName(user_ui->cmb_serial_port->currentText());
    if(user_port.open(QIODevice::ReadWrite))
    {
        //设置波特率
        user_port.setBaudRate(user_ui->cmb_baud_rate->currentText().toInt());
        switch (user_ui->cmb_data_bits->currentText().toInt())
        {
          case 8:  user_port.setDataBits(QSerialPort::Data8);       break;
          case 7:  user_port.setDataBits(QSerialPort::Data7);       break;
          case 6:  user_port.setDataBits(QSerialPort::Data6);       break;
          case 5:  user_port.setDataBits(QSerialPort::Data5);       break;
          default: break;
        }
        switch (user_ui->cmb_stop_bits->currentText().toInt())
        {
          case 1:  user_port.setStopBits(QSerialPort::OneStop);       break;
          case 2:  user_port.setStopBits(QSerialPort::TwoStop);       break;
          default: break;
        }
        switch (user_ui->cmb_parity_bits->currentIndex())
        {
          case 0: user_port.setParity(QSerialPort::NoParity);       break;
          case 1: user_port.setParity(QSerialPort::OddParity);      break;
          case 2: user_port.setParity(QSerialPort::EvenParity);     break;
          default: break;
        }
        user_port.setFlowControl(QSerialPort::NoFlowControl);
        user_ui->cmb_serial_port->setEnabled(false);
        user_ui->cmb_baud_rate->setEnabled(false);
        user_ui->cmb_data_bits->setEnabled(false);
        user_ui->cmb_stop_bits->setEnabled(false);
        user_ui->cmb_parity_bits->setEnabled(false);
        user_ui->btn_open_serail->setEnabled(false);
        user_ui->btn_close_serial->setEnabled(true);
        //网络端口不可设置
        user_ui->le_taget_ip->setEnabled(false);
        user_ui->sb_taget_port->setEnabled(false);
        user_ui->sb_local_port->setEnabled(false);
        user_ui->btn_net_connet->setEnabled(false);
        user_ui->btn_net_disconnet->setEnabled(false);
        link_type = 1;
    }
    else
    {
        QMessageBox::about(NULL, "提示", "串口无法打开\r\n不存在或已占用");
    }
}

//关闭串口
void CONSOLE:: btn_port_close(void)
{
    if(user_port.isOpen())
    {
      user_port.close();
      user_ui->cmb_serial_port->setEnabled(true);
      user_ui->cmb_baud_rate->setEnabled(true);
      user_ui->cmb_data_bits->setEnabled(true);
      user_ui->cmb_stop_bits->setEnabled(true);
      user_ui->cmb_parity_bits->setEnabled(true);
      user_ui->btn_open_serail->setEnabled(true);
      user_ui->btn_close_serial->setEnabled(false);
      //网络端口可设置
      user_ui->le_taget_ip->setEnabled(true);
      user_ui->sb_taget_port->setEnabled(true);
      user_ui->sb_local_port->setEnabled(true);
      user_ui->btn_net_connet->setEnabled(true);
      user_ui->btn_net_disconnet->setEnabled(false);
      link_type = 0;
    }
}

//串口接收
void CONSOLE:: port_read(const QByteArray data)
{
  this->readbuf.append(data);
}

/*********************网络操作**************************************************/
// 网络连接
void CONSOLE:: btn_net_connet()
{
    local_port = user_ui->sb_local_port->value();
    taget_ip   = user_ui->le_taget_ip->getIP();
    taget_port = user_ui->sb_taget_port->value();
    if(!udpSocket.bind(QHostAddress::AnyIPv4, local_port,QUdpSocket::ShareAddress))
    {
        QMessageBox::about(NULL, "提示", "本地端口无法绑定\r\n或已占用");
        return; 
    }
    //设置网络按钮不可操作 串口按钮不可操作
    user_ui->cmb_serial_port->setEnabled(false);
    user_ui->cmb_baud_rate->setEnabled(false);
    user_ui->cmb_data_bits->setEnabled(false);
    user_ui->cmb_stop_bits->setEnabled(false);
    user_ui->cmb_parity_bits->setEnabled(false);
    user_ui->btn_open_serail->setEnabled(false);
    user_ui->btn_close_serial->setEnabled(false);
    //网络端口不可设置
    user_ui->le_taget_ip->setEnabled(false);
    user_ui->sb_taget_port->setEnabled(false);
    user_ui->sb_local_port->setEnabled(false);
    user_ui->btn_net_connet->setEnabled(false);
    user_ui->btn_net_disconnet->setEnabled(true);
    link_type = 2;
    udp_connet = 0;

}

// 网络断开
void CONSOLE:: btn_net_disconnet(void)
{
  //关闭网络
  if(link_type == 2){
    udpSocket.close();
   // 串口按钮可操作
    user_ui->cmb_serial_port->setEnabled(true);
    user_ui->cmb_baud_rate->setEnabled(true);
    user_ui->cmb_data_bits->setEnabled(true);
    user_ui->cmb_stop_bits->setEnabled(true);
    user_ui->cmb_parity_bits->setEnabled(true);
    user_ui->btn_open_serail->setEnabled(true);
    user_ui->btn_close_serial->setEnabled(false);
    //网络端口可设置
    user_ui->le_taget_ip->setEnabled(true);
    user_ui->sb_taget_port->setEnabled(true);
    user_ui->sb_local_port->setEnabled(true);
    user_ui->btn_net_connet->setEnabled(true);
    user_ui->btn_net_disconnet->setEnabled(false);
    link_type = 0;
    udp_connet = 0;
  }

}

// 网络接收
void CONSOLE:: net_read(void)
{
    
    QByteArray datagram;
    if(udp_connet == 0){
      udp_connet = 1;
    }
     //设置data的大小为等待处理的数据报的大小，这样才能接收到完整的数据
    datagram.resize(udpSocket.pendingDatagramSize());
     // 接收数据报，将其存放到datagram中
    udpSocket.readDatagram(datagram.data(), datagram.size());
     //处理接收数据
    QString str = datagram.data();//此处data()函数返回char*类型，可以用QString类型接收结果
     //然后可以将QString显示出来，比如显示在QLabel、QLineEdit、QTextEdit或显示在程序输出里
    readbuf+=datagram;
}

// 网络接收
void CONSOLE:: net_trytoconnet(void)
{
  QByteArray datagram;
  datagram.append(0xAA);
  datagram.append(0x55);
 // qDebug()<<link_type;
 // qDebug()<<udp_connet;
  if(link_type == 2 && udp_connet == 0)
  {
    udpSocket.writeDatagram(datagram.data(), 2, QHostAddress(taget_ip), taget_port);
  }
  //帧频计算
  data_freq = data_freq_count;
  data_freq_count=0;

}

/*********************控制参数**************************************************/
//控制参数设置   
void CONSOLE:: btn_control_dug_set(void)
{
  if(user_ui->cb_stick_enable->isChecked() == true){
      QMessageBox::about(NULL, "提示", "请先关闭虚拟遥控功能");
      return;
  }
  protocol_tA.balance_kp = (float)user_ui->sb_balance_kp->value();
  protocol_tA.balance_kd = (float)user_ui->sb_balance_kd->value();
  protocol_tA.vel_kp     = (float)user_ui->sb_spd_kp->value();
  protocol_tA.vel_ki     = (float)user_ui->sb_spd_ki->value();
  protocol_tA.turn_kp    = (float)user_ui->sb_turn_kp->value();
  protocol_tA.turn_kd    = (float)user_ui->sb_turn_kd->value();
  protocol_tA.taget_spd  = (float)user_ui->sb_taget_spd->value();
  protocol_tA.taget_yaw  = (float)user_ui->sb_taget_yaw->value();
  protocol_tA.ifsave = 0;
  protocol_TA_encode();
  switch (link_type) {
    case 1:
       if(user_port.isOpen())
       {
           user_port.write(writebuf);
       }
      break;
    case 2:
       udpSocket.writeDatagram(writebuf.data(), writebuf.size(), QHostAddress(taget_ip), taget_port);
      break;
    default:
      break;
  }
}

//控制参数查询
void CONSOLE:: btn_control_dug_req(void)
{

    user_ui->sb_balance_kp->setValue(protocol_rB.balance_kp);
    user_ui->sb_balance_kd->setValue(protocol_rB.balance_kd);
    user_ui->sb_spd_kp->setValue(protocol_rB.vel_kp);
    user_ui->sb_spd_ki->setValue(protocol_rB.vel_ki);
    user_ui->sb_turn_kp->setValue(protocol_rB.turn_kp);
    user_ui->sb_turn_kd->setValue(protocol_rB.turn_kd);
    user_ui->sb_taget_spd->setValue(protocol_rB.taget_spd);
    user_ui->sb_taget_yaw->setValue(protocol_rB.taget_yaw);

}

//控制参数应用
void CONSOLE:: btn_control_dug_apply(void)
{
    if(user_ui->cb_stick_enable->isChecked() == true){
        QMessageBox::about(NULL, "提示", "请先关闭虚拟遥控功能");
        return;
    }
    protocol_tA.balance_kp = (float)user_ui->sb_balance_kp->value();
    protocol_tA.balance_kd = (float)user_ui->sb_balance_kd->value();
    protocol_tA.vel_kp     = (float)user_ui->sb_spd_kp->value();
    protocol_tA.vel_ki     = (float)user_ui->sb_spd_ki->value();
    protocol_tA.turn_kp    = (float)user_ui->sb_turn_kp->value();
    protocol_tA.turn_kd    = (float)user_ui->sb_turn_kd->value();
    protocol_tA.taget_spd  = (float)user_ui->sb_taget_spd->value();
    protocol_tA.taget_yaw  = (float)user_ui->sb_taget_yaw->value();
    protocol_tA.ifsave     = 1;
    protocol_TA_encode();
    switch (link_type) {
      case 1:
         if(user_port.isOpen())
         {
             user_port.write(writebuf);
         }
        break;
      case 2:
         udpSocket.writeDatagram(writebuf.data(), writebuf.size(), QHostAddress(taget_ip), taget_port);
        break;
      default:
        break;
    }

}

//数据更新
void CONSOLE:: data_update(void)
{
    protocol_decode();

    //数据显示
    user_ui->gaugeCar->setValue(qAbs(protocol_rB.encode_spd));
    user_ui->gaugePlane->setRollValue(protocol_rB.pitch);
    user_ui->gaugeCompassPan->setValue(protocol_rB.yaw);

    if(link_type == 2){
      user_ui->label_frq->setText("网络帧频(HZ):");
    }
    else{
      user_ui->label_frq->setText("串口帧频(HZ):");
    }
    user_ui->lcd_data_frq->display(data_freq);
    user_ui->lcd_cpu_usage->display(protocol_rD.cpu_usage);
    user_ui->lcd_taget_yaw->display(protocol_rB.taget_yaw);

    user_ui->lcd_pitch->display(protocol_rB.pitch);
    user_ui->lcd_pitch_degs->display(protocol_rB.pitch_gyro);
    user_ui->lcd_yaw->display(protocol_rB.yaw);
    user_ui->lcd_yaw_degs->display(protocol_rB.yaw_gyro);

    user_ui->lcd_taget_spd->display(protocol_rB.taget_spd);
    user_ui->lcd_current_spd->display(protocol_rB.encode_spd);
    user_ui->lcd_left_spd->display(protocol_rB.lencode_spd);
    user_ui->lcd_right_spd->display(protocol_rB.rencode_spd);

    if(protocol_rB.l_dir == 0){
        user_ui->le_left_motor_dir->setText("顺");
    }
    else{
        user_ui->le_left_motor_dir->setText("逆");
    }
    if(protocol_rB.r_dir == 0){
        user_ui->le_right_motor_dir->setText("顺");
    }
    else{
        user_ui->le_right_motor_dir->setText("逆");
    }

    user_ui->lcd_lbalance_pwm->display(protocol_rB.balance_pwm);
    user_ui->lcd_lspd_pwm->display(protocol_rB.vel_pwm);
    user_ui->lcd_lturn_pwm->display(protocol_rB.lturn_pwm);
    user_ui->lcd_lfinal_pwm->display(protocol_rB.lfinal_pwm);

    user_ui->lcd_rbalance_pwm->display(protocol_rB.balance_pwm);
    user_ui->lcd_rspd_pwm->display(protocol_rB.vel_pwm);
    user_ui->lcd_rturn_pwm->display(protocol_rB.rturn_pwm);
    user_ui->lcd_rfinal_pwm->display(protocol_rB.rfinal_pwm);

    if(protocol_rB.l_dircmd == 0){
        user_ui->le_lcmd_dir->setText("顺");
    }
    else{
        user_ui->le_lcmd_dir->setText("逆");
    }
    if(protocol_rB.r_dircmd == 0){
        user_ui->le_rcmd_dir->setText("顺");
    }
    else{
        user_ui->le_rcmd_dir->setText("逆");
    }

    //指令显示
    QString string;
    string ="CMD->";
    for(unsigned char index = 0; index < CONTROL_MSG_CMDLEN; index++){
       string.append(protocol_rD.msg_cmd[index]);
    }
    user_ui->le_cmd->setEnabled(true);
    user_ui->le_cmd->setText(string);
    user_ui->le_cmd->setEnabled(false);

}

/*********************IMU数据保存**************************************************/
void CONSOLE:: imu_record(void)
{
   QString imu_file_path;
   QDateTime time = QDateTime::currentDateTime();
   imu_file_path = QDir::currentPath();
   if(user_ui->sw_data_record->getChecked() == true && imu_file_open == 0)
   {
       imu_file_path += "/record/imu/";
       QDir dir(imu_file_path);
       if (!dir.exists("./")) {
           dir.mkpath("./");
       }
       imu_file_path += "imu_";
       imu_file_path += time.toString("yyyy_MM_dd_hh_mm_ss");
       imu_file_path += ".csv";
       imu_file = new QFile(imu_file_path);
       if(!imu_file->open(QIODevice::WriteOnly))
       {
            return;
       }
       imu_file->write("tick,imu_status,imu_tmp,,imu_gyro_rawest[0],imu_gyro_rawest[1],imu_gyro_rawest[2],"
                       "imu_gyro_raw[0],imu_gyro_raw[1],imu_gyro_raw[2],imu_gyro[0],imu_gyro[1],imu_gyro[2],, "
                       " imu_acc_rawest[0],imu_acc_rawest[1],imu_acc_rawest[2],imu_acc_raw[0],imu_acc_raw[1],imu_acc_raw[2],"
                       " imu_acc[0],imu_acc[1],imu_acc[2],,imu_roll,imu_pitch,imu_yaw,\n");
       imu_file_open = 1;
   }

   if(user_ui->sw_data_record->getChecked() == true && imu_file_open == 1)
   {
     QString string;
     QByteArray BYTE;
     string.append(QString::number(protocol_rA.tick)+',');
     string.append(QString::number(protocol_rA.imu_health_status)+',');
     string.append(QString::number(protocol_rA.imu_tmp)+',');
     string.append(',');
     string.append(QString::number(protocol_rA.imu_gyro_rawest[0])+',');
     string.append(QString::number(protocol_rA.imu_gyro_rawest[1])+',');
     string.append(QString::number(protocol_rA.imu_gyro_rawest[2])+',');
     string.append(QString::number(protocol_rA.imu_gyro_raw[0])+',');
     string.append(QString::number(protocol_rA.imu_gyro_raw[1])+',');
     string.append(QString::number(protocol_rA.imu_gyro_raw[2])+',');
     string.append(QString::number(protocol_rA.imu_gyro[0])+',');
     string.append(QString::number(protocol_rA.imu_gyro[1])+',');
     string.append(QString::number(protocol_rA.imu_gyro[2])+',');
     string.append(',');
     string.append(QString::number(protocol_rA.imu_acc_rawest[0])+',');
     string.append(QString::number(protocol_rA.imu_acc_rawest[1])+',');
     string.append(QString::number(protocol_rA.imu_acc_rawest[2])+',');
     string.append(QString::number(protocol_rA.imu_acc_raw[0])+',');
     string.append(QString::number(protocol_rA.imu_acc_raw[1])+',');
     string.append(QString::number(protocol_rA.imu_acc_raw[2])+',');
     string.append(QString::number(protocol_rA.imu_acc[0])+',');
     string.append(QString::number(protocol_rA.imu_acc[1])+',');
     string.append(QString::number(protocol_rA.imu_acc[2])+',');
     string.append(',');
     string.append(QString::number(protocol_rA.imu_roll)+',');
     string.append(QString::number(protocol_rA.imu_pitch)+',');
     string.append(QString::number(protocol_rA.imu_yaw)+',');
     string.append('\n');
     BYTE = string.toUtf8();
     imu_file->write(BYTE.data());
   }

   if(user_ui->sw_data_record->getChecked() == false && imu_file_open == 1)
   {
        imu_file_open = 0;
        imu_file->close();
        delete imu_file;
   }

}

/*********************控制数据保存**************************************************/
void CONSOLE:: control_record(void)
{
    QString control_file_path;
    QDateTime time = QDateTime::currentDateTime();
    control_file_path = QDir::currentPath();
    if(user_ui->sw_data_record->getChecked() == true && control_file_open == 0)
    {
        control_file_path += "/record/control/";
        QDir dir(control_file_path);
        if (!dir.exists("./")) {
            dir.mkpath("./");
        }
        control_file_path += "control_";
        control_file_path += time.toString("yyyy_MM_dd_hh_mm_ss");
        control_file_path += ".csv";
        control_file = new QFile(control_file_path);
        if(!control_file->open(QIODevice::WriteOnly))
        {
             return;
        }
        control_file->write("tick,balance_kp,balance_kd,vel_kp,vel_ki,turn_kp,turn_kd,,"
                            "balance_pwm,vel_pwm,lturn_pwm,lfinal_pwm,rturn_pwm,rfinal_pwm,l_dircmd,r_dircmd,,"
                            "taget_spd,lencode_spd,rencode_spd,l_dir,r_dir,encode_spd,encode_spd_integ,,"
                            "taget_yaw,pitch,pitch_gyro,yaw,yaw_gyro\n");
        control_file_open = 1;
    }

    if(user_ui->sw_data_record->getChecked() == true && control_file_open == 1)
    {
      QString string;
      QByteArray BYTE;
      string.append(QString::number(protocol_rB.tick)+',');
      string.append(QString::number(protocol_rB.balance_kp)+',');
      string.append(QString::number(protocol_rB.balance_kd)+',');
      string.append(QString::number(protocol_rB.vel_kp)+',');
      string.append(QString::number(protocol_rB.vel_ki)+',');
      string.append(QString::number(protocol_rB.turn_kp)+',');
      string.append(QString::number(protocol_rB.turn_kd)+',');
      string.append(',');

      string.append(QString::number(protocol_rB.balance_pwm)+',');
      string.append(QString::number(protocol_rB.vel_pwm)+',');
      string.append(QString::number(protocol_rB.lturn_pwm)+',');
      string.append(QString::number(protocol_rB.lfinal_pwm)+',');
      string.append(QString::number(protocol_rB.rturn_pwm)+',');
      string.append(QString::number(protocol_rB.rfinal_pwm)+',');
      string.append(QString::number(protocol_rB.l_dircmd)+',');
      string.append(QString::number(protocol_rB.r_dircmd)+',');
      string.append(',');

      string.append(QString::number(protocol_rB.taget_spd)+',');
      string.append(QString::number(protocol_rB.lencode_spd)+',');
      string.append(QString::number(protocol_rB.rencode_spd)+',');
      string.append(QString::number(protocol_rB.l_dir)+',');
      string.append(QString::number(protocol_rB.r_dir)+',');
      string.append(QString::number(protocol_rB.encode_spd)+',');
      string.append(QString::number(protocol_rB.encode_spd_integ)+',');  
      string.append(',');  

      string.append(QString::number(protocol_rB.taget_yaw)+',');
      string.append(QString::number(protocol_rB.pitch)+',');
      string.append(QString::number(protocol_rB.pitch_gyro)+',');
      string.append(QString::number(protocol_rB.yaw)+',');
      string.append(QString::number(protocol_rB.yaw_gyro)+','); 
      string.append('\n');
      BYTE = string.toUtf8();
      control_file->write(BYTE.data());
    }

    if(user_ui->sw_data_record->getChecked() == false && control_file_open == 1)
    {
         control_file_open = 0;
         control_file->close();
         delete control_file;
    }

}

/*********************虚拟摇杆**************************************************/
void CONSOLE:: joystick_setxy(qreal px, qreal py)
{
    /*px 0-1    -CONSOLE_MAX_TAGRYAW - 0  CONSOLE_MAX_TAGRYAW - 1*/
    /*py 0-1    -CONSOLE_MAX_TAGSPD  - 1  CONSOLE_MAX_TAGSPD  - 0*/
     joystick_x = px;
     joystick_y = py;
     protocol_tB.taget_spd = (-2*CONSOLE_MAX_TAGSPD)*joystick_y + CONSOLE_MAX_TAGSPD;
     protocol_tB.taget_ryaw = (2*CONSOLE_MAX_TAGRYAW)*joystick_x - CONSOLE_MAX_TAGRYAW;
     qDebug()<<"JOYSTICK::jokstick_setxy"<<protocol_tB.taget_ryaw<<protocol_tB.taget_spd;
}

void CONSOLE:: joystick_sendvalue(void)
{
    if(user_ui->cb_stick_enable->isChecked() == false){
        return;
    }
    protocol_TB_encode();
    switch (link_type) {
      case 1:
         if(user_port.isOpen())
         {
             user_port.write(writebuf);
         }
        break;
      case 2:
         udpSocket.writeDatagram(writebuf.data(), writebuf.size(), QHostAddress(taget_ip), taget_port);
        break;
      default:
        break;
    }

}
