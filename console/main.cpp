#include "ui_ui_parm.h"
#include "console_setup.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QMainWindow my_wm;
    class Ui_Parm  my_ui;
    class CONSOLE  my_console(&my_wm, &my_ui);
    my_wm.show();
    return a.exec();
}
