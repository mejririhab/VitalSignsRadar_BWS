#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <signal.h>

void handleCrash(int sig)
{
    qDebug() << "Application crashed with signal:" << sig;
    exit(sig);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    // Install crash handler
    signal(SIGSEGV, handleCrash); // Handle segmentation faults
    signal(SIGABRT, handleCrash); // Handle abort signals

    w.show(); // Ensure the main window is displayed

    qDebug() << "Application started, showing main window";
    return a.exec(); // Start the event loop
}
