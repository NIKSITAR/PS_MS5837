#include <QCoreApplication>
#include <ms5837.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    MS5837 ps;
    return a.exec();
}
