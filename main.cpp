#include "orc.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication orcApp(argc, argv);
    orc orcObject;
    orcObject.show();

    return orcApp.exec();
}
