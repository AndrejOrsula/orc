#include <QApplication>
#include "orc.h"

int main(int argc, char *argv[])
{
    QApplication orcApp(argc, argv);
    orc orcObject;
    orcObject.show();

    return orcApp.exec();
}
