#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <iostream>
#include <string>
#include <QStandardItemModel>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <QFileInfo>
#include <QTimer>
#include <sys/types.h>
#include <QShortcut>
#include <QScroller>
#include <QStorageInfo>
#include <QAbstractItemModel>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QtWidgets>
int klik_log = 0;
int tampilan_tujuan = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    arduino_is_available = false;
    arduino_port_name = "/dev/ttyUSB0";
    arduino = new QSerialPort;
    qApp->setStyleSheet("QWidget#MainWindow {border-image:url(\"/home/kongkevin/icon/background.jpg\"); background-position: center; background-repeat: round;}");
    QPixmap pix("/home/kongkevin/icon/new-car.png");
    ui->label_2->setStyleSheet("background-color:transparent;");
    ui->label_2->setAlignment(Qt::AlignCenter);
    ui->label_2->setPixmap(pix.scaled(300,300, Qt::KeepAspectRatio));
    ui->pushButton_logdata -> setStyleSheet("background-color:white;");
    ui->pushButton_logdata->setIcon(QIcon("/home/kongkevin/icon/database-icon.png"));
    QPixmap pix2("/home/kongkevin/icon/turn-off-icon.png");
    ui->label_6->setStyleSheet("background-color:transparent;");
    ui->label_6->setAlignment(Qt::AlignCenter);
    ui->label_6->setPixmap(pix2.scaled(131,101, Qt::KeepAspectRatio));

     /*
        qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            qDebug() << "Has vendor ID: " << serialPortInfo.hasVendorIdentifier();
            if(serialPortInfo.hasVendorIdentifier()){
                qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier();
            }
            qDebug() << "Has Product ID: " << serialPortInfo.hasProductIdentifier();
            if(serialPortInfo.hasProductIdentifier()){
                qDebug() << "Product ID: " << serialPortInfo.productIdentifier();
            }
        }

    */
        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){
                if(serialPortInfo.vendorIdentifier() == arduino_uno_vendor_id){
                    if(serialPortInfo.productIdentifier() == arduino_uno_product_id){
                        arduino_port_name = serialPortInfo.portName();
                        arduino_is_available = true;
                    }
                }
            }
        }

        if(arduino_is_available){
            // open and configure the serialport
            arduino->setPortName(arduino_port_name);
            arduino->open(QSerialPort::ReadWrite);
            arduino->setBaudRate(QSerialPort::Baud9600);
            arduino->setDataBits(QSerialPort::Data8);
            arduino->setParity(QSerialPort::NoParity);
            arduino->setStopBits(QSerialPort::OneStop);
            arduino->setFlowControl(QSerialPort::NoFlowControl);
            QObject::connect(arduino, SIGNAL(readyRead()), this, SLOT(readSerial()));
        }else{
            // give error message if not available
            QMessageBox::warning(this, "Port error", "Couldn't find the Arduino!");
        }
}

MainWindow::~MainWindow()
{
    if(arduino->isOpen()){
            arduino->close();
    }
    delete ui;
}

void MainWindow::readSerial()
{
    QStringList buffer_split = serialBuffer.split(",");
    if(buffer_split.length() < 3){
            // no parsed value yet so continue accumulating bytes from serial in the buffer.
            serialData = arduino->readAll();
            serialBuffer = serialBuffer + QString::fromStdString(serialData.toStdString());
            serialData.clear();
        }else{
            // the second element of buffer_split is parsed correctly, update the temperature value on temp_lcdNumber
            serialBuffer = "";
            qDebug() << buffer_split;
            parsed_data = buffer_split[1];
            parsed_data_2 = buffer_split[2];
            MainWindow::updateRange(parsed_data);
            MainWindow::updateKecepatan(parsed_data_2);
        }


}

void MainWindow::updateRange(QString sensor_reading)
{
    //  update the value displayed on the lcdNumber
    ui->lcdNumber->display(sensor_reading);
}

void MainWindow::updateKecepatan(QString sensor_reading)
{
    //  update the value displayed on the lcdNumber
    ui->lcdNumber_2->display(sensor_reading);
}

void MainWindow::on_pushButton_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}

void MainWindow::on_pushButton_2_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}

void MainWindow::on_pushButton_3_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}


void MainWindow::on_pushButton_4_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}

void MainWindow::on_pushButton_5_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}

void MainWindow::on_pushButton_6_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
  tampilan_tujuan = 1;
  deteksiObjek();
  QString s = QString::number(tampilan_tujuan);
  MainWindow::updateTujuan(s);
}

void MainWindow::on_pushButton_logdata_clicked()
{
    if (tampilan_tujuan == 1){
        if (klik_log == 0){
            ui->stackedWidget->setCurrentIndex(2);
            klik_log = 1;

        }
        else{
            ui->stackedWidget->setCurrentIndex(1);
            klik_log = 0;
        }
    }
    else {
        ui->stackedWidget->setCurrentIndex(0);
    }
}

void MainWindow::deteksiObjek()
{
    if (tampilan_tujuan== 0){
         QPixmap pix2("/home/kongkevin/icon/turn-off-icon.png");
         ui->label_6->setStyleSheet("background-color:transparent;");
         ui->label_6->setAlignment(Qt::AlignCenter);
         ui->label_6->setPixmap(pix2.scaled(131,101, Qt::KeepAspectRatio));
     }
    else{
         QPixmap pix2("/home/kongkevin/icon/turn-on-icon.png");
         ui->label_6->setStyleSheet("background-color:transparent;");
         ui->label_6->setAlignment(Qt::AlignCenter);
         ui->label_6->setPixmap(pix2.scaled(131,101, Qt::KeepAspectRatio));
     }
}

void MainWindow::updateTujuan(QString tujuan){
    if(arduino->isWritable()){
            arduino->write(tujuan.toStdString().c_str());
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}
