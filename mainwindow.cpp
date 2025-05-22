#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QtEndian>
#include <QDialog>
#include <QSerialPortInfo>
#include <QFile>
#include <QElapsedTimer>                       // This class provides a fast way to calculate elapsed times
#include "dialogsettings.h"

#define HEART_RATE_LOW_THRESHOLD  60  // BPM
#define HEART_RATE_HIGH_THRESHOLD 100 // BPM
#define BREATHING_RATE_LOW_THRESHOLD  12  // Breaths per minute
#define BREATHING_RATE_HIGH_THRESHOLD 20  // Breaths per minute

#define LENGTH_MAGIC_WORD_BYTES         8  // Length of Magic Word appended to the UART packet from the EVM
#define LENGTH_HEADER_BYTES             40   // Header + Magic Word
#define LENGTH_TLV_MESSAGE_HEADER_BYTES 8
#define LENGTH_DEBUG_DATA_OUT_BYTES     128   // VitalSigns_OutputStats size
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN  32   // The data sent out through the UART has Extra Padding to make it a multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN
#define LENGTH_OFFSET_BYTES             LENGTH_HEADER_BYTES - LENGTH_MAGIC_WORD_BYTES + LENGTH_TLV_MESSAGE_HEADER_BYTES
#define LENGTH_OFFSET_NIBBLES           2*LENGTH_OFFSET_BYTES

#define  INDEX_GLOBAL_COUNT                  6
#define  INDEX_RANGE_BIN_PHASE               1
#define  INDEX_RANGE_BIN_VALUE               2
#define  INDEX_PHASE                         5
#define  INDEX_BREATHING_WAVEFORM            6
#define  INDEX_HEART_WAVEFORM                7
#define  INDEX_HEART_RATE_EST_FFT            8
#define  INDEX_HEART_RATE_EST_FFT_4Hz        9
#define  INDEX_HEART_RATE_EST_FFT_xCorr      10
#define  INDEX_HEART_RATE_EST_PEAK           11
#define  INDEX_BREATHING_RATE_FFT            12
#define  INDEX_BREATHING_RATE_xCorr          13
#define  INDEX_BREATHING_RATE_PEAK           14
#define  INDEX_CONFIDENCE_METRIC_BREATH      15
#define  INDEX_CONFIDENCE_METRIC_BREATH_xCorr 16
#define  INDEX_CONFIDENCE_METRIC_HEART       17
#define  INDEX_CONFIDENCE_METRIC_HEART_4Hz   18
#define  INDEX_CONFIDENCE_METRIC_HEART_xCorr 19
#define  INDEX_ENERGYWFM_BREATH              20
#define  INDEX_ENERGYWFM_HEART               21
#define  INDEX_MOTION_DETECTION              22
#define  INDEX_BREATHING_RATE_HARM_ENERG     23
#define  INDEX_HEART_RATE_HARM_ENERG         24

#define  INDEX_RANGE_PROFILE_START           35

#define  INDEX_IN_GLOBAL_FRAME_COUNT                INDEX_GLOBAL_COUNT*8
#define  INDEX_IN_RANGE_BIN_INDEX                   LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_RANGE_BIN_PHASE*8 + 4
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_HEART_4Hz  LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_HEART_4Hz*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_HEART_xCorr  LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_HEART_xCorr*8
#define  INDEX_IN_DATA_PHASE                        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_PHASE*8
#define  INDEX_IN_DATA_BREATHING_WAVEFORM           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_WAVEFORM*8
#define  INDEX_IN_DATA_HEART_WAVEFORM               LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_WAVEFORM*8
#define  INDEX_IN_DATA_BREATHING_RATE_FFT           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_FFT*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT           LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT_4Hz       LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT_4Hz*8
#define  INDEX_IN_DATA_HEART_RATE_EST_FFT_xCorr     LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_FFT_xCorr*8
#define  INDEX_IN_DATA_BREATHING_RATE_PEAK          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_PEAK*8
#define  INDEX_IN_DATA_HEART_RATE_EST_PEAK          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_HEART_RATE_EST_PEAK*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_BREATH     LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_BREATH*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_HEART      LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_HEART*8
#define  INDEX_IN_DATA_ENERGYWFM_BREATH             LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_ENERGYWFM_BREATH*8
#define  INDEX_IN_DATA_ENERGYWFM_HEART              LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_ENERGYWFM_HEART*8
#define  INDEX_IN_DATA_MOTION_DETECTION_FLAG        LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_MOTION_DETECTION*8
#define  INDEX_IN_DATA_CONFIDENCE_METRIC_BREATH_xCorr  LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_CONFIDENCE_METRIC_BREATH_xCorr*8
#define  INDEX_IN_DATA_BREATHING_RATE_HARM_ENERGY   LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_HARM_ENERG*8
#define  INDEX_IN_DATA_BREATHING_RATE_xCorr         LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_BREATHING_RATE_xCorr*8
#define  INDEX_IN_DATA_RANGE_PROFILE_START          LENGTH_MAGIC_WORD_BYTES + LENGTH_OFFSET_NIBBLES + INDEX_RANGE_PROFILE_START*8

#define NUM_PTS_DISTANCE_TIME_PLOT        (256)
#define HEART_RATE_EST_MEDIAN_FLT_SIZE    (200)
#define HEART_RATE_EST_FINAL_OUT_SIZE     (200)
#define THRESH_HEART_CM                   (0.25)
#define THRESH_BREATH_CM                  (1.0)
#define BACK_THRESH_BPM                   (4)
#define BACK_THRESH_CM                    (0.20)
#define BACK_THRESH_4Hz_CM                (0.15)
#define THRESH_BACK                       (30)
#define THRESH_DIFF_EST                   (20)
#define ALPHA_HEARTRATE_CM                (0.2)
#define ALPHA_RCS                         (0.2)
#define APPLY_KALMAN_FILTER               (0.0)

float BREATHING_PLOT_MAX_YAXIS;
float HEART_PLOT_MAX_YAXIS;

QSerialPort *serialRead, *serialWrite;
bool  FileSavingFlag;
bool  serialPortFound_Flag;
bool  FlagSerialPort_Connected, dataPort_Connected, userPort_Connected;
float thresh_breath, thresh_heart;

enum  gui_status { gui_running, gui_stopped, gui_paused };
gui_status current_gui_status;

QSettings settings("Be Wireless Solutions", "Vital Signs");
QFile outFile("dataOutputFromEVM.bin");

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
qDebug() << "MainWindow initialized, serial ports being configured";

    QPixmap bkgnd(":/new/prefix51/back2.jpg");
    bkgnd = bkgnd.scaled(this->size(), Qt::IgnoreAspectRatio);

    QPalette palette;
    palette.setBrush(QPalette::Window, bkgnd);
    this->setPalette(palette);

    localCount = 0;
    currIndex  = 0;

    qDebug() <<"Vital Signs monitor developped by Be Wireless Solutions";
    qDebug() <<"QT version = " <<QT_VERSION_STR;

    qint32 baudRate = 921600;
    FLAG_PAUSE = false;
    outFile.open(QIODevice::Append);
    AUTO_DETECT_COM_PORTS = ui->checkBox_AutoDetectPorts->isChecked();

    if(AUTO_DETECT_COM_PORTS)
    {
        serialPortFound_Flag = serialPortFind();
        if (serialPortFound_Flag)
        {
            qDebug()<<"Serial Port Found";
            qDebug()<<"Data Port Number is" << dataPortNum;
            qDebug()<<"User Port Number is" << userPortNum;
        }
    }

    serialRead = new QSerialPort(this);
    dataPort_Connected = serialPortConfig(serialRead, baudRate, dataPortNum);
    if (dataPort_Connected)
        printf("Data port succesfully Open\n");
    else
        printf("data port did Not Open\n");

    serialWrite = new QSerialPort(this);
    userPort_Connected = serialPortConfig(serialWrite, 115200, userPortNum);
    if (userPort_Connected)
        printf("User port succesfully Open\n");
    else
        printf("User port did not Open \n");

    connect(serialRead,SIGNAL(readyRead()),this,SLOT(serialRecieved()));

    // Plot Settings
    QFont font;
    font.setPointSize(12);
    QPen myPen;
    myPen.setWidth(1);
    myPen.setColor(Qt::red);
    QFont titleFont;
    titleFont.setPointSize(15);
    titleFont.bold();

    ui->checkBox_LoadConfig->setChecked(true);
    statusBar()->showMessage(tr("Ready"));
    ui->checkBox_displayPlots->setChecked(true);

    xDistTimePlot.resize(NUM_PTS_DISTANCE_TIME_PLOT);
    yDistTimePlot.resize(NUM_PTS_DISTANCE_TIME_PLOT);
    breathingWfmBuffer.resize(NUM_PTS_DISTANCE_TIME_PLOT);
    heartWfmBuffer.resize(NUM_PTS_DISTANCE_TIME_PLOT);


    //QColor plotBackgroundColor = this->palette().color(QPalette::Window); // Use window background color

    // QColor plotBackgroundColor = QColor("#ADD8E6"); //  baby blue
     //QColor plotBackgroundColor = QColor("#D3D3D3"); // Light gray
     //QColor plotBackgroundColor = QColor("#E0E0E0"); //very light gray
    QColor plotBackgroundColor = QColor("#FFFFFF"); // pure white


    ui->plot_RangeProfile->addGraph(0);
    ui->plot_RangeProfile->setBackground(plotBackgroundColor);
    ui->plot_RangeProfile->axisRect()->setBackground(plotBackgroundColor);
    ui->plot_RangeProfile->xAxis->setLabel("Range (m)");
    ui->plot_RangeProfile->xAxis->setLabelFont(font);
    ui->plot_RangeProfile->yAxis->setLabel("Magnitude (a.u.)");
    ui->plot_RangeProfile->yAxis->setLabelFont(font);
    ui->plot_RangeProfile->axisRect()->setupFullAxesBox();
    ui->plot_RangeProfile->xAxis->setRange(0,NUM_PTS_DISTANCE_TIME_PLOT);

    QCPTextElement *myTitle = new QCPTextElement(ui->plot_RangeProfile, "Range Profile");
    myTitle->setFont(titleFont);
    ui->plot_RangeProfile->plotLayout()->insertRow(0);
    ui->plot_RangeProfile->plotLayout()->addElement(0, 0, myTitle);

    ui->phaseWfmPlot->addGraph(0);
    ui->phaseWfmPlot->setBackground(plotBackgroundColor);
    ui->phaseWfmPlot->axisRect()->setBackground(plotBackgroundColor);
    ui->phaseWfmPlot->xAxis->setLabel("Frame (Index)");
    ui->phaseWfmPlot->xAxis->setLabelFont(font);
    ui->phaseWfmPlot->yAxis->setLabel("Displacement (a.u.)");
    ui->phaseWfmPlot->yAxis->setLabelFont(font);
    ui->phaseWfmPlot->axisRect()->setupFullAxesBox();
    ui->phaseWfmPlot->xAxis->setRange(0,NUM_PTS_DISTANCE_TIME_PLOT);
    ui->phaseWfmPlot->graph(0)->setPen(myPen);

    QCPTextElement *myTitle_ChestDisp = new QCPTextElement(ui->phaseWfmPlot, "Chest Displacement");
    myTitle_ChestDisp->setFont(titleFont);
    ui->phaseWfmPlot->plotLayout()->insertRow(0);
    ui->phaseWfmPlot->plotLayout()->addElement(0, 0, myTitle_ChestDisp);

    ui->BreathingWfmPlot->addGraph(0);
    ui->BreathingWfmPlot->setBackground(plotBackgroundColor);
    ui->BreathingWfmPlot->axisRect()->setBackground(plotBackgroundColor);
    ui->BreathingWfmPlot->xAxis->setLabelFont(font);
    ui->BreathingWfmPlot->yAxis->setLabel("Phase (radians)");
    ui->BreathingWfmPlot->yAxis->setLabelFont(font);
    ui->BreathingWfmPlot->axisRect()->setupFullAxesBox();
    ui->BreathingWfmPlot->xAxis->setRange(0,NUM_PTS_DISTANCE_TIME_PLOT);
    ui->BreathingWfmPlot->graph(0)->setPen(myPen);

    QCPTextElement *myTitle_BreathWfm = new QCPTextElement(ui->BreathingWfmPlot, "Breathing Waveform");
    myTitle_BreathWfm->setFont(titleFont);
    ui->BreathingWfmPlot->plotLayout()->insertRow(0);
    ui->BreathingWfmPlot->plotLayout()->addElement(0, 0, myTitle_BreathWfm);

    ui->heartWfmPlot->addGraph(0);
    ui->heartWfmPlot->setBackground(plotBackgroundColor);
    ui->heartWfmPlot->axisRect()->setBackground(plotBackgroundColor);
    ui->heartWfmPlot->xAxis->setLabelFont(font);
    ui->heartWfmPlot->yAxis->setLabel("Phase (Radians)");
    ui->heartWfmPlot->yAxis->setLabelFont(font);
    ui->heartWfmPlot->axisRect()->setupFullAxesBox();
    ui->heartWfmPlot->xAxis->setRange(0,NUM_PTS_DISTANCE_TIME_PLOT);
    ui->heartWfmPlot->graph(0)->setPen(myPen);

    QPalette lcdpaletteBreathing = ui->lcdNumber_Breathingrate->palette();
    lcdpaletteBreathing.setColor(QPalette::Normal, QPalette::Window, Qt::white);

    QPalette lcdpaletteHeart = ui->lcdNumber_HeartRate->palette();
    lcdpaletteHeart.setColor(QPalette::Normal, QPalette::Window, Qt::white);

    QPalette lcdpaletteNotBreathing = ui->lcdNumber_Breathingrate->palette();
    lcdpaletteNotBreathing.setColor(QPalette::Normal, QPalette::Window, Qt::red);

    ui->lcdNumber_HeartRate->setAutoFillBackground(true);
    ui->lcdNumber_HeartRate->setPalette(lcdpaletteHeart);
    ui->lcdNumber_Breathingrate->setAutoFillBackground(true);
    ui->lcdNumber_Breathingrate->setPalette(lcdpaletteBreathing);

    ui->lcdNumber_AbnormalHeart->display("0");
    ui->lcdNumber_AbnormalBreath->display("0");

            // Set default background colors (e.g., white)
            QPalette lcdPaletteNormal = ui->lcdNumber_AbnormalHeart->palette();
            lcdPaletteNormal.setColor(QPalette::Normal, QPalette::Window, Qt::white);
            ui->lcdNumber_AbnormalHeart->setPalette(lcdPaletteNormal);
            ui->lcdNumber_AbnormalBreath->setPalette(lcdPaletteNormal);

    QCPTextElement *myTitle_HeartWfm = new QCPTextElement(ui->heartWfmPlot, "Heart Waveform");
    myTitle_HeartWfm->setFont(titleFont);
    ui->heartWfmPlot->plotLayout()->insertRow(0);
    ui->heartWfmPlot->plotLayout()->addElement(0, 0, myTitle_HeartWfm);

    ui->lineEdit_ProfileBack->setText("xwr1642_profile_VitalSigns_20fps_Back.cfg");
    ui->lineEdit_ProfileFront->setText("xwr1642_profile_VitalSigns_20fps_Front.cfg");

    connect(this,SIGNAL(gui_statusChanged()),this,SLOT(gui_statusUpdate()));
}

void MainWindow::serialRecieved()
{
    QByteArray dataSerial = serialRead->readAll().toHex();
    int dataSize = dataSerial.size();
    qDebug() << "received serial data, size: " << dataSize;
    qDebug() << "Raw data (first 64 bytes): " << dataSerial.left(64);
    dataBuffer.append(dataSerial);
    indexBuffer += dataSize;
    processData();
}

MainWindow::~MainWindow()
{
    serialWrite->write("sensorStop\n");
    serialWrite->waitForBytesWritten(10000);
    serialRead->close();
    serialWrite->close();
    delete ui;
}

void MainWindow::on_pushButton_start_clicked()
{
    localCount = 0;
    if (current_gui_status == gui_paused)
    {
        current_gui_status = gui_running;
        emit gui_statusChanged();
        return;
    }

    FLAG_PAUSE = false;
    AUTO_DETECT_COM_PORTS = ui->checkBox_AutoDetectPorts->isChecked();

    if (AUTO_DETECT_COM_PORTS)
    {
        if (!FlagSerialPort_Connected)
        {
            serialPortFound_Flag = serialPortFind();
            if (!userPort_Connected)
            {
                userPort_Connected = serialPortConfig(serialWrite, 115200, userPortNum);
            }
            if (!dataPort_Connected)
            {
                dataPort_Connected = serialPortConfig(serialRead, 921600, dataPortNum);
            }
        }
    }
    else
    {
        userPortNum = ui->lineEdit_UART_port->text();
        dataPortNum = ui->lineEdit_data_port->text();
        userPort_Connected = serialPortConfig(serialWrite, 115200, userPortNum);
        dataPort_Connected = serialPortConfig(serialRead, 921600, dataPortNum);
    }

    if (ui->checkBox_LoadConfig->isChecked())
    {
        QString profileBack = ui->lineEdit_ProfileBack->text();
        QString profileFront = ui->lineEdit_ProfileFront->text();
        qDebug() << "Configuration File Names - Back:" << profileBack << "Front:" << profileFront;

        DialogSettings myDialogue;
        QString userComPort = myDialogue.getUserComPortNum();
        qDebug() << "User COM Port:" << userComPort;

        QDir currDir = QCoreApplication::applicationDirPath();
        currDir.cdUp();
        QString filenameText = currDir.path() + "/profiles/";
        QString selectedProfile;

        if (ui->radioButton_BackMeasurements->isChecked())
        {
            selectedProfile = profileBack;
            filenameText.append(profileBack);
            qDebug() << "Using Back Measurement Profile. Path is:" << filenameText;
            HEART_PLOT_MAX_YAXIS = myDialogue.getHeartWfm_yAxisMax();
            BREATHING_PLOT_MAX_YAXIS = myDialogue.getBreathWfm_yAxisMax();
            thresh_breath = 1e8; // Adjusted threshold
            thresh_heart = 100;  // Adjusted threshold
        }
        else
        {
            selectedProfile = profileFront;
            filenameText.append(profileFront);
            qDebug() << "Using Front Measurement Profile. Path is:" << filenameText;
            thresh_breath = 1e8; // Adjusted threshold
            thresh_heart = 100;  // Adjusted threshold
            BREATHING_PLOT_MAX_YAXIS = 1.0;
            HEART_PLOT_MAX_YAXIS = 0.5;
        }
        ui->SpinBox_TH_Breath->setValue(thresh_breath);
        ui->SpinBox_TH_Heart->setValue(thresh_heart);

        QFile infile(filenameText);
        QStringList configLines;

        if (infile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qDebug() << "Config file opened successfully for reading:" << filenameText;
            QTextStream inStream(&infile);
            while (!inStream.atEnd())
            {
                QString line = inStream.readLine().trimmed();
                if (!line.isEmpty())
                {
                    configLines.append(line);
                    qDebug() << "Sending line:" << line;
                    serialWrite->write(line.toUtf8() + "\n");
                    if (!serialWrite->waitForBytesWritten(500)) // Reduced timeout to 500ms
                    {
                        qDebug() << "Failed to write line to serial port:" << serialWrite->errorString();
                    }
                    QThread::msleep(50); // Reduced delay to 50ms
                }
            }
            infile.close();
        }
        else
        {
            qDebug() << "Failed to open config file for reading:" << infile.errorString() << "Path:" << filenameText;
            statusBar()->showMessage(tr("Failed to open configuration file"));
            return;
        }

        for (const QString& line : configLines)
        {
            QStringList listArgs = line.split(QRegExp("\\s+"));
            if (line.contains("vitalSignsCfg", Qt::CaseInsensitive) && listArgs.size() >= 6)
            {
                demoParams.rangeStartMeters = listArgs.at(1).toFloat();
                demoParams.rangeEndMeters = listArgs.at(2).toFloat();
                demoParams.AGC_thresh = listArgs.at(5).toFloat();
                qDebug() << "Parsed vitalSignsCfg - rangeStartMeters:" << demoParams.rangeStartMeters
                         << "rangeEndMeters:" << demoParams.rangeEndMeters
                         << "AGC_thresh:" << demoParams.AGC_thresh;
            }
            if (line.contains("profileCfg", Qt::CaseInsensitive) && listArgs.size() >= 12)
            {
                demoParams.stratFreq_GHz = listArgs.at(2).toFloat();
                demoParams.freqSlope_MHZ_us = listArgs.at(8).toFloat();
                demoParams.numSamplesChirp = listArgs.at(10).toFloat();
                demoParams.samplingRateADC_ksps = listArgs.at(11).toInt();
                qDebug() << "Parsed profileCfg - stratFreq_GHz:" << demoParams.stratFreq_GHz
                         << "freqSlope_MHZ_us:" << demoParams.freqSlope_MHZ_us
                         << "numSamplesChirp:" << demoParams.numSamplesChirp
                         << "samplingRateADC_ksps:" << demoParams.samplingRateADC_ksps;
            }
        }

        demoParams.chirpDuration_us = 1e3 * demoParams.numSamplesChirp / demoParams.samplingRateADC_ksps;
        qDebug() << "Chirp Duration in us is:" << demoParams.chirpDuration_us;
        demoParams.chirpBandwidth_kHz = demoParams.freqSlope_MHZ_us * demoParams.chirpDuration_us;
        qDebug() << "Chirp Bandwidth in kHz is:" << demoParams.chirpBandwidth_kHz;
        float numTemp = demoParams.chirpDuration_us * demoParams.samplingRateADC_ksps * 3e8;
        float denTemp = 2 * demoParams.chirpBandwidth_kHz;
        demoParams.rangeMaximum_meters = numTemp / (denTemp * 1e9);
        qDebug() << "Maximum Range in Meters is:" << demoParams.rangeMaximum_meters;
        demoParams.rangeFFTsize = nextPower2(demoParams.numSamplesChirp);
        qDebug() << "Range-FFT size is:" << demoParams.rangeFFTsize;
        demoParams.rangeBinSize_meters = demoParams.rangeMaximum_meters / demoParams.rangeFFTsize;
        qDebug() << "Range-Bin size is:" << demoParams.rangeBinSize_meters;
        demoParams.rangeBinStart_index = demoParams.rangeStartMeters / demoParams.rangeBinSize_meters;
        demoParams.rangeBinEnd_index = demoParams.rangeEndMeters / demoParams.rangeBinSize_meters;
        qDebug() << "Range-Bin Start Index is:" << demoParams.rangeBinStart_index;
        qDebug() << "Range-Bin End Index is:" << demoParams.rangeBinEnd_index;
        demoParams.numRangeBinProcessed = demoParams.rangeBinEnd_index - demoParams.rangeBinStart_index + 1;

        demoParams.totalPayloadSize_bytes = LENGTH_HEADER_BYTES;
        demoParams.totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES + (4 * demoParams.numRangeBinProcessed);
        demoParams.totalPayloadSize_bytes += LENGTH_TLV_MESSAGE_HEADER_BYTES + LENGTH_DEBUG_DATA_OUT_BYTES;
        qDebug() << "Total Payload size from the UART is:" << demoParams.totalPayloadSize_bytes;
        if ((demoParams.totalPayloadSize_bytes % MMWDEMO_OUTPUT_MSG_SEGMENT_LEN) != 0)
        {
            int paddingFactor = ceil((float)demoParams.totalPayloadSize_bytes / (float)MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
            qDebug() << "Padding Factor is:" << paddingFactor;
            demoParams.totalPayloadSize_bytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN * paddingFactor;
        }
        qDebug() << "Total Payload size from the UART is:" << demoParams.totalPayloadSize_bytes;
        demoParams.totalPayloadSize_nibbles = 2 * demoParams.totalPayloadSize_bytes;
        qDebug() << "numRangeBinProcessed:" << demoParams.numRangeBinProcessed;
        qDebug() << "totalPayloadSize_bytes:" << demoParams.totalPayloadSize_bytes;
        qDebug() << "totalPayloadSize_nibbles:" << demoParams.totalPayloadSize_nibbles;
    }

    ui->heartWfmPlot->yAxis->setRange(-HEART_PLOT_MAX_YAXIS, HEART_PLOT_MAX_YAXIS);
    current_gui_status = gui_running;
    emit gui_statusChanged();
}


void MainWindow::on_pushButton_Refresh_clicked()
{
    serialWrite->write("guiMonitor 0 0 0 1\n");
    serialWrite->waitForBytesWritten(10000);
    localCount = 0;
    Sleep(2000);
}

int MainWindow::nextPower2(int num)
{
    int power = 1;
    while(power < num)
        power*=2;
    return power;
}

float MainWindow::parseValueFloat(QByteArray data, int valuePos, int valueSize)
{
    bool ok;
    QByteArray parseData;
    parseData = data.mid(valuePos,valueSize);
    QString strParseData = parseData;
    quint32 temp_int = strParseData.toUInt(&ok,16);
    temp_int = qToBigEndian(temp_int);
    float parseValueOut;
    parseValueOut = *reinterpret_cast<float*>(&temp_int);
    return parseValueOut;
}

quint32 MainWindow::parseValueUint32(QByteArray data, int valuePos, int valueSize)
{
    bool ok;
    QByteArray parseData = data.mid(valuePos, valueSize);
    QString strParseData = parseData;
    qDebug() << "Parsing uint32 at pos: " << valuePos << ", data: " << parseData;
    quint32 tempInt32 = strParseData.toUInt(&ok, 16);
    if (!ok)
    {
        qDebug() << "Failed to parse uint32: " << strParseData;
        return 0;
    }
    quint32 parseValueOut = qToBigEndian(tempInt32);
    return parseValueOut;
}

quint16 MainWindow::parseValueUint16(QByteArray data, int valuePos, int valueSize)
{
    bool ok;
    QByteArray parseData;
    parseData = data.mid(valuePos, valueSize);
    QString strParseData = parseData;
    quint16 parseValueOut = strParseData.toInt(&ok,16);
    parseValueOut = qToBigEndian(parseValueOut);
    return parseValueOut;
}

bool MainWindow::serialPortFind()
{
    DialogSettings dialogBoxSerial;
    QString portNum;
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        portNum = serialPortInfo.portName();
        dialogBoxSerial.setDataComPortNum(portNum);
        dialogBoxSerial.setUserComPortNum(portNum);
    }

    dataPortNum = dialogBoxSerial.getDataComPortNum();
    userPortNum = dialogBoxSerial.getUserComPortNum();
    ui->lineEdit_data_port->setText(dataPortNum);
    ui->lineEdit_UART_port->setText(userPortNum);
    if (portNum.isEmpty())
        return 0;
    else
        return 1;
}

bool MainWindow::serialPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum)
{
    serial->setPortName(dataPortNum);
    qDebug() << "Configuring port: " << dataPortNum << " at baud rate: " << baudRate;
    if (serial->open(QIODevice::ReadWrite))
    {
        FlagSerialPort_Connected = 1;
        serial->setBaudRate(baudRate);
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        qDebug() << "Port " << dataPortNum << " opened successfully";
    }
    else
    {
        FlagSerialPort_Connected = 0;
        qDebug() << "Failed to open port " << dataPortNum << ": " << serial->errorString();
    }
    return FlagSerialPort_Connected;
}



void MainWindow::processData()
{
    QByteArray dataSave;
    QByteArray data;
    static float outHeartNew_CM;
    static float maxRCS_updated;
    bool MagicOk;

    static float Pk = 1;
    static float xk = 0;
    static int updateCounter=0;

    FileSavingFlag = ui->checkBox_SaveData->isChecked();
    localCount = localCount + 1;
    updateCounter++;


    qDebug() << "Using totalPayloadSize_nibbles:" << demoParams.totalPayloadSize_nibbles;
    if (demoParams.totalPayloadSize_nibbles <= 0)
    {
        qDebug() << "Invalid totalPayloadSize_nibbles, using default: 512";
        demoParams.totalPayloadSize_nibbles = 512;
    }
    // Limit dataBuffer size to prevent memory exhaustion
        if (dataBuffer.size() > 10 * demoParams.totalPayloadSize_nibbles)
        {
            qDebug() << "dataBuffer too large, clearing";
            dataBuffer.clear();
            return;
        }
    while (dataBuffer.size() >= demoParams.totalPayloadSize_nibbles)
    {
        QElapsedTimer timer;
        timer.start();

        QByteArray searchStr("0201040306050807");
        int dataStartIndex = dataBuffer.indexOf(searchStr);
        int indexTemp = localCount % NUM_PTS_DISTANCE_TIME_PLOT;

        if (dataStartIndex == -1)
        {
            MagicOk = 0;
            qDebug() << "Magic Word Not Found --- localCount:" << localCount << " DataBufferSize:" << dataBuffer.size();
            dataBuffer.clear();
            break;
        }
        else
        {
            data = dataBuffer.mid(dataStartIndex, demoParams.totalPayloadSize_nibbles);
            if (data.size() == demoParams.totalPayloadSize_nibbles)
            {
                dataBuffer.remove(0, dataStartIndex + demoParams.totalPayloadSize_nibbles);
                qDebug() << "dataBuffer size after removal is:" << dataBuffer.size();
                MagicOk = 1;
                statusBar()->showMessage(tr("Sensor Running"));
            }
            else
            {
                qDebug() << "Invalid frame size:" << data.size() << ", expected:" << demoParams.totalPayloadSize_nibbles;
                dataBuffer.clear();
                MagicOk = 0;
                statusBar()->showMessage(tr("Invalid frame detected, clearing buffer"));
                break;
            }
        }

        if (MagicOk == 1)
        {
            if (FileSavingFlag)
            {
                dataSave = data;
                outFile.write(dataSave.fromHex(data), demoParams.totalPayloadSize_bytes);
            }

            quint32 globalCountOut = parseValueUint32(data, 40, 8);
            qDebug() << "Frame Number is:" << globalCountOut;

            static quint32 lastGlobalCount = 0;
            if (globalCountOut == lastGlobalCount)
            {
                qDebug() << "Skipping duplicate frame:" << globalCountOut;
                continue;
            }
            lastGlobalCount = globalCountOut;

            quint16 rangeBinIndexOut = parseValueUint16(data, INDEX_IN_RANGE_BIN_INDEX, 4);
            float BreathingRate_FFT = parseValueFloat(data, INDEX_IN_DATA_BREATHING_RATE_FFT, 8);
            float BreathingRatePK_Out = parseValueFloat(data, INDEX_IN_DATA_BREATHING_RATE_PEAK, 8);
            float heartRate_FFT = parseValueFloat(data, INDEX_IN_DATA_HEART_RATE_EST_FFT, 8);
            float heartRate_Pk = parseValueFloat(data, INDEX_IN_DATA_HEART_RATE_EST_PEAK, 8);
            float heartRate_xCorr = parseValueFloat(data, INDEX_IN_DATA_HEART_RATE_EST_FFT_xCorr, 8);
            float heartRate_FFT_4Hz = parseValueFloat(data, INDEX_IN_DATA_HEART_RATE_EST_FFT_4Hz, 8) / 2;
            float phaseWfm_Out = parseValueFloat(data, INDEX_IN_DATA_PHASE, 8);
            float breathWfm_Out = parseValueFloat(data, INDEX_IN_DATA_BREATHING_WAVEFORM, 8);
            float heartWfm_Out = parseValueFloat(data, INDEX_IN_DATA_HEART_WAVEFORM, 8);
            float breathRate_CM = parseValueFloat(data, INDEX_IN_DATA_CONFIDENCE_METRIC_BREATH, 8);
            float heartRate_CM = parseValueFloat(data, INDEX_IN_DATA_CONFIDENCE_METRIC_HEART, 8);
            float heartRate_4Hz_CM = parseValueFloat(data, INDEX_IN_DATA_CONFIDENCE_METRIC_HEART_4Hz, 8);
            float heartRate_xCorr_CM = parseValueFloat(data, INDEX_IN_DATA_CONFIDENCE_METRIC_HEART_xCorr, 8);
            float outSumEnergyBreathWfm = parseValueFloat(data, INDEX_IN_DATA_ENERGYWFM_BREATH, 8);
            float outSumEnergyHeartWfm = parseValueFloat(data, INDEX_IN_DATA_ENERGYWFM_HEART, 8);
            float outMotionDetectionFlag = parseValueFloat(data, INDEX_IN_DATA_MOTION_DETECTION_FLAG, 8);
            float BreathingRate_xCorr_CM = parseValueFloat(data, INDEX_IN_DATA_CONFIDENCE_METRIC_BREATH_xCorr, 8);
            float BreathingRate_HarmEnergy = parseValueFloat(data, INDEX_IN_DATA_BREATHING_RATE_HARM_ENERGY, 8);
            float BreathingRate_xCorr = parseValueFloat(data, INDEX_IN_DATA_BREATHING_RATE_xCorr, 8);

            qDebug() << "Parsed Values:";
            qDebug() << "BreathingRate_FFT:" << BreathingRate_FFT;
            qDebug() << "BreathingRatePK_Out:" << BreathingRatePK_Out;
            qDebug() << "heartRate_FFT:" << heartRate_FFT;
            qDebug() << "heartRate_Pk:" << heartRate_Pk;
            qDebug() << "heartRate_xCorr:" << heartRate_xCorr;
            qDebug() << "breathRate_CM:" << breathRate_CM;
            qDebug() << "heartRate_CM:" << heartRate_CM;
            qDebug() << "outSumEnergyBreathWfm:" << outSumEnergyBreathWfm;
            qDebug() << "outSumEnergyHeartWfm:" << outSumEnergyHeartWfm;
            qDebug() << "BreathingRate_xCorr_CM:" << BreathingRate_xCorr_CM;

            unsigned int numRangeBinProcessed = demoParams.rangeBinEnd_index - demoParams.rangeBinStart_index + 1;
            QVector<double> RangeProfile(2*numRangeBinProcessed);
            QVector<double> xRangePlot(numRangeBinProcessed), yRangePlot(numRangeBinProcessed);
            unsigned int indexRange = INDEX_IN_DATA_RANGE_PROFILE_START;

            for (unsigned int index = 0; index < 2*numRangeBinProcessed; index++)
            {
                qint16 tempRange_int = parseValueUint16(data, indexRange, 4);
                RangeProfile[index] = tempRange_int;
                indexRange = indexRange + 4;
            }

            for (unsigned int indexRangeBin = 0; indexRangeBin < numRangeBinProcessed; indexRangeBin++)
            {
                yRangePlot[indexRangeBin] = sqrt(RangeProfile[2*indexRangeBin]*RangeProfile[2*indexRangeBin] + RangeProfile[2*indexRangeBin + 1]*RangeProfile[2*indexRangeBin + 1]);
                xRangePlot[indexRangeBin] = demoParams.rangeStartMeters + demoParams.rangeBinSize_meters*indexRangeBin;
            }
            double maxRCS = *std::max_element(yRangePlot.constBegin(), yRangePlot.constEnd());
            maxRCS_updated = ALPHA_RCS*(maxRCS) + (1-ALPHA_RCS)*maxRCS_updated;

            float BreathingRate_Out, heartRate_Out;
            float diffEst_heartRate, heartRateEstDisplay;

            float heartRate_OutMedian;
            static QVector<float> heartRateBuffer;
            heartRateBuffer.resize(HEART_RATE_EST_MEDIAN_FLT_SIZE);

            static QVector<float> heartRateOutBufferFinal;
            heartRateOutBufferFinal.resize(HEART_RATE_EST_FINAL_OUT_SIZE);

            float outHeartPrev_CM = outHeartNew_CM;
            outHeartNew_CM = ALPHA_HEARTRATE_CM*(heartRate_CM) + (1-ALPHA_HEARTRATE_CM)*outHeartPrev_CM;

            diffEst_heartRate = abs(heartRate_FFT - heartRate_Pk);
            if ((outHeartNew_CM > THRESH_HEART_CM) || (diffEst_heartRate < THRESH_DIFF_EST))
            {
                heartRateEstDisplay = heartRate_FFT;
            }
            else
            {
                heartRateEstDisplay = heartRate_Pk;
            }

            if (ui->checkBox_xCorr->isChecked())
            {
                heartRateEstDisplay = heartRate_xCorr;
            }

            if (ui->checkBox_FFT->isChecked())
            {
                heartRateEstDisplay = heartRate_FFT;
            }

            if (ui->radioButton_BackMeasurements->isChecked())
            {
#ifdef HEAURITICS_APPROACH1
                if (abs(heartRate_xCorr-heartRate_FFT) < THRESH_BACK)
                {
                    heartRateEstDisplay = heartRate_FFT;
                }
                else
                {
                    heartRateEstDisplay = heartRate_xCorr;
                }

                heartRateBuffer.insert(2*(localCount % HEART_RATE_EST_MEDIAN_FLT_SIZE/2), heartRateEstDisplay);

                if (ui->checkBox_FFT)
                {
                    heartRateBuffer.insert(2*(localCount % HEART_RATE_EST_MEDIAN_FLT_SIZE/2)+1, heartRateEstDisplay);
                }
                else
                {
                    heartRateBuffer.insert(2*(localCount % HEART_RATE_EST_MEDIAN_FLT_SIZE/2)+1, heartRate_FFT_4Hz);
                }
#endif

                int IsvalueSelected = 0;

                if (abs(heartRate_xCorr - 2*BreathingRate_FFT) > BACK_THRESH_BPM)
                {
                    heartRateBuffer.insert(currIndex % HEART_RATE_EST_MEDIAN_FLT_SIZE, heartRate_xCorr);
                    IsvalueSelected = 1;
                    currIndex++;
                }
                if (heartRate_CM > BACK_THRESH_CM)
                {
                    heartRateBuffer.insert(currIndex % HEART_RATE_EST_MEDIAN_FLT_SIZE, heartRate_FFT);
                    IsvalueSelected = 1;
                    currIndex++;
                }
                if (heartRate_4Hz_CM > BACK_THRESH_4Hz_CM)
                {
                    heartRateBuffer.insert(currIndex % HEART_RATE_EST_MEDIAN_FLT_SIZE, heartRate_FFT_4Hz);
                    IsvalueSelected = 1;
                    currIndex++;
                }

                if (IsvalueSelected == 0)
                {
                    heartRateBuffer.insert(currIndex % HEART_RATE_EST_MEDIAN_FLT_SIZE, heartRate_Pk);
                    currIndex++;
                }
            }
            else
            {
                heartRateBuffer.insert(localCount % HEART_RATE_EST_MEDIAN_FLT_SIZE, heartRateEstDisplay);
            }

            if (gui_paused != current_gui_status)
            {
                qDebug() << "GUI Status Check - current_gui_status:" << current_gui_status << "gui_paused:" << gui_paused;
                QList<float> heartRateBufferSort = QList<float>::fromVector(heartRateBuffer);
                qSort(heartRateBufferSort.begin(), heartRateBufferSort.end());
                heartRate_OutMedian = heartRateBufferSort.at(HEART_RATE_EST_MEDIAN_FLT_SIZE/2);

                if (APPLY_KALMAN_FILTER)
                {
                    float R;
                    float Q;
                    float KF_Gain;
                    float CM_combined;
                    CM_combined = heartRate_CM + heartRate_4Hz_CM + 10*heartRate_xCorr_CM;
                    R = 1/(CM_combined + 0.0001);
                    Q = 1e-6;
                    KF_Gain = Pk/(Pk + R);
                    xk = xk + KF_Gain*(heartRate_OutMedian - xk);
                    Pk = (1-KF_Gain)*Pk + Q;
                    heartRate_Out = xk;
                }
                else
                {
                    heartRate_Out = heartRate_OutMedian;
                }

                heartRateOutBufferFinal.insert(localCount % (HEART_RATE_EST_FINAL_OUT_SIZE), heartRate_Out);
                const auto mean = std::accumulate(heartRateOutBufferFinal.begin(), heartRateOutBufferFinal.end(), .0) / heartRateOutBufferFinal.size();
                double sumMAD;
                double bufferSTD;
                sumMAD = 0;
                for (int indexTemp=0; indexTemp<heartRateOutBufferFinal.size(); indexTemp++)
                {
                    sumMAD += abs(heartRateOutBufferFinal.at(indexTemp) - mean);
                }
                bufferSTD = sqrt(sumMAD)/heartRateOutBufferFinal.size();
                ui->lcdNumber_ReliabilityMetric->display(bufferSTD);
                qDebug() << "Displayed Reliability Metric:" << bufferSTD;

                float outSumEnergyBreathWfm_thresh = ui->SpinBox_TH_Breath->value();
                float RCS_thresh = ui->SpinBox_RCS->value();
                bool flag_Breathing;

                qDebug() << "Thresholds - outSumEnergyBreathWfm:" << outSumEnergyBreathWfm << "vs thresh:" << outSumEnergyBreathWfm_thresh;
                qDebug() << "Thresholds - maxRCS_updated:" << maxRCS_updated << "vs RCS_thresh:" << RCS_thresh;
                qDebug() << "Thresholds - BreathingRate_xCorr_CM:" << BreathingRate_xCorr_CM << "vs 0.002";

                if ((outSumEnergyBreathWfm < outSumEnergyBreathWfm_thresh) || (maxRCS_updated < RCS_thresh) || (BreathingRate_xCorr_CM <= 0.002))
                {
                    flag_Breathing = 0;
                    BreathingRate_Out = 0;
                    QPalette lcdpaletteNotBreathing = ui->lcdNumber_Breathingrate->palette();
                    lcdpaletteNotBreathing.setColor(QPalette::Normal, QPalette::Window, Qt::red);
                    ui->lcdNumber_Breathingrate->setPalette(lcdpaletteNotBreathing);
                }
                else
                {
                    flag_Breathing = 1;
                    QPalette lcdpaletteBreathing = ui->lcdNumber_Breathingrate->palette();
                    lcdpaletteBreathing.setColor(QPalette::Normal, QPalette::Window, Qt::white);
                    ui->lcdNumber_Breathingrate->setPalette(lcdpaletteBreathing);

                    if (breathRate_CM > THRESH_BREATH_CM)
                    {
                        BreathingRate_Out = BreathingRate_FFT;
                    }
                    else
                    {
                        BreathingRate_Out = BreathingRatePK_Out;
                    }
                }

                float outSumEnergyHeartWfm_thresh = ui->SpinBox_TH_Heart->value();

                qDebug() << "Thresholds - outSumEnergyHeartWfm:" << outSumEnergyHeartWfm << "vs thresh:" << outSumEnergyHeartWfm_thresh;

                if (outSumEnergyHeartWfm < outSumEnergyHeartWfm_thresh || maxRCS_updated < RCS_thresh)
                {
                    heartRate_Out = 0;
                    QPalette lcdpaletteNoHeartRate = ui->lcdNumber_HeartRate->palette();
                    lcdpaletteNoHeartRate.setColor(QPalette::Normal, QPalette::Window, Qt::red);
                    heartWfm_Out = 0;
                    ui->lcdNumber_HeartRate->setPalette(lcdpaletteNoHeartRate);
                }
                else
                {
                    QPalette lcdpaletteHeartRate = ui->lcdNumber_HeartRate->palette();
                    lcdpaletteHeartRate.setColor(QPalette::Normal, QPalette::Window, Qt::white);
                    ui->lcdNumber_HeartRate->setPalette(lcdpaletteHeartRate);
                }

                qDebug() << "Final Rates - BreathingRate_Out:" << BreathingRate_Out;
                qDebug() << "Final Rates - heartRate_Out:" << heartRate_Out;

                if (BreathingRate_Out != 0) // Only check if breathing rate is non-zero (valid)
                        {
                            if (BreathingRate_Out < BREATHING_RATE_LOW_THRESHOLD || BreathingRate_Out > BREATHING_RATE_HIGH_THRESHOLD)
                            {
                                // Abnormal breathing rate detected
                                QString myString_AbnormalBreath = QString::number(BreathingRate_Out, 'f', 0);
                                ui->lcdNumber_AbnormalBreath->setDigitCount(8);
                                ui->lcdNumber_AbnormalBreath->display(myString_AbnormalBreath);
                                qDebug() << "Abnormal Breathing Rate Detected:" << BreathingRate_Out;

                                // Highlight the abnormal breathing rate in red
                                QPalette lcdPaletteAbnormal = ui->lcdNumber_AbnormalBreath->palette();
                                lcdPaletteAbnormal.setColor(QPalette::Normal, QPalette::Window, Qt::red);
                                ui->lcdNumber_AbnormalBreath->setPalette(lcdPaletteAbnormal);
                            }
                        }


                 if (heartRate_Out !=0)
                 {
                     if (heartRate_Out< HEART_RATE_LOW_THRESHOLD || heartRate_Out > HEART_RATE_HIGH_THRESHOLD)
                     {
                         QString myString_AbnormalHeart = QString::number(heartRate_Out,'f',0);
                         ui->lcdNumber_AbnormalHeart->setDigitCount(8);
                         ui->lcdNumber_AbnormalHeart->display(myString_AbnormalHeart);
                         qDebug() << "Abnormal heart rate Detected:" << heartRate_Out;

                         QPalette lcdPaletteAbnormal = ui->lcdNumber_AbnormalHeart->palette();
                         lcdPaletteAbnormal.setColor(QPalette::Normal , QPalette::Window, Qt::red);
                         ui->lcdNumber_AbnormalHeart->setPalette(lcdPaletteAbnormal);
                     }
                 }

                if (ui->checkBox_displayPlots->isChecked()&& updateCounter % 2 == 0)

                {
                    if (indexTemp == 0)
                    {
                        for (unsigned int i = 0; i < NUM_PTS_DISTANCE_TIME_PLOT; i++)
                        {
                            xDistTimePlot[i] = indexTemp;
                            yDistTimePlot[i] = phaseWfm_Out;
                            heartWfmBuffer[i] = heartWfm_Out;
                            breathingWfmBuffer[indexTemp] = breathWfm_Out;
                        }
                    }

                    double max = *std::max_element(breathingWfmBuffer.constBegin(), breathingWfmBuffer.constEnd());
                    double min = *std::min_element(breathingWfmBuffer.constBegin(), breathingWfmBuffer.constEnd());

                    double breathingWfm_display_max, breathingWfm_display_min;

                    if (max < BREATHING_PLOT_MAX_YAXIS)
                        breathingWfm_display_max = BREATHING_PLOT_MAX_YAXIS;
                    else
                        breathingWfm_display_max = max;

                    if (min > -BREATHING_PLOT_MAX_YAXIS)
                        breathingWfm_display_min = -BREATHING_PLOT_MAX_YAXIS;
                    else
                        breathingWfm_display_min = min;

                    xDistTimePlot[indexTemp] = indexTemp;
                    yDistTimePlot[indexTemp] = phaseWfm_Out;
                    breathingWfmBuffer[indexTemp] = breathWfm_Out;
                    heartWfmBuffer[indexTemp] = heartWfm_Out;

                    ui->phaseWfmPlot->yAxis->setRange(-10, 10);
                    ui->phaseWfmPlot->graph(0)->setData(xDistTimePlot, yDistTimePlot);
                    ui->phaseWfmPlot->yAxis->rescale();
                    ui->phaseWfmPlot->replot();

                    ui->BreathingWfmPlot->graph(0)->setData(xDistTimePlot, breathingWfmBuffer);
                    ui->BreathingWfmPlot->yAxis->setRangeLower(breathingWfm_display_min);
                    ui->BreathingWfmPlot->yAxis->setRangeUpper(breathingWfm_display_max);
                    ui->BreathingWfmPlot->replot();

                    ui->heartWfmPlot->graph(0)->setData(xDistTimePlot, heartWfmBuffer);
                    ui->heartWfmPlot->replot();

                    ui->plot_RangeProfile->graph(0)->setData(xRangePlot, yRangePlot);
                    ui->plot_RangeProfile->xAxis->setRange(demoParams.rangeStartMeters, demoParams.rangeEndMeters);

                    if (maxRCS < (ui->SpinBox_RCS->value()))
                    {
                        ui->plot_RangeProfile->yAxis->setRangeUpper(ui->SpinBox_RCS->value());
                    }
                    else
                    {
                        ui->plot_RangeProfile->yAxis->setRangeUpper(maxRCS);
                    }

                    ui->plot_RangeProfile->replot();
                }

                // Update all LCD displays with debug output
                ui->lcdNumber_FrameCount->display((int)globalCountOut);
                qDebug() << "Raw Frame Count:" << globalCountOut << "Displayed Frame Count:" << QString::number((int)globalCountOut);

                QString myString_BreathRate;
                ui->lcdNumber_Breathingrate->setDigitCount(8);
                myString_BreathRate = QString::number(BreathingRate_Out, 'f', 0); // Alternative formatting
                ui->lcdNumber_Breathingrate->display(myString_BreathRate);
                qDebug() << "Raw Breathing Rate:" << BreathingRate_Out << "Displayed Breathing Rate:" << myString_BreathRate;

                QString myString_HeartRate;
                ui->lcdNumber_HeartRate->setDigitCount(3);
                myString_HeartRate = QString::number(heartRate_Out, 'f', 0); // Alternative formatting
                ui->lcdNumber_HeartRate->display(myString_HeartRate);
                qDebug() << "Raw Heart Rate:" << heartRate_Out << "Displayed Heart Rate:" << myString_HeartRate;

                QString myString_RangeBinIndex;
                ui->lcdNumber_Index->setDigitCount(8);
                myString_RangeBinIndex = QString::number(rangeBinIndexOut);
                ui->lcdNumber_Index->display(myString_RangeBinIndex);
                qDebug() << "Raw Range Bin Index:" << rangeBinIndexOut << "Displayed Range Bin Index:" << myString_RangeBinIndex;

                QString myString_BreathingRatePK_Out;
                ui->lcdNumber_Breath_pk->setDigitCount(8);
                myString_BreathingRatePK_Out = QString::number(BreathingRatePK_Out, 'f', 0);
                ui->lcdNumber_Breath_pk->display(myString_BreathingRatePK_Out);
                qDebug() << "Raw Breathing Rate Peak:" << BreathingRatePK_Out << "Displayed Breathing Rate Peak:" << myString_BreathingRatePK_Out;

                QString myString_heartRate_Pk;
                ui->lcdNumber_Heart_pk->setDigitCount(8);
                myString_heartRate_Pk = QString::number(heartRate_Pk, 'f', 0);
                ui->lcdNumber_Heart_pk->display(myString_heartRate_Pk);
                qDebug() << "Raw Heart Rate Peak:" << heartRate_Pk << "Displayed Heart Rate Peak:" << myString_heartRate_Pk;

                QString myString_BreathingRate_FFT;
                ui->lcdNumber_Breath_FT->setDigitCount(8);
                myString_BreathingRate_FFT = QString::number(BreathingRate_FFT, 'f', 0);
                ui->lcdNumber_Breath_FT->display(myString_BreathingRate_FFT);
                qDebug() << "Raw Breathing Rate FFT:" << BreathingRate_FFT << "Displayed Breathing Rate FFT:" << myString_BreathingRate_FFT;

                QString myString_HeartRate_FFT;
                ui->lcdNumber_Heart_FT->setDigitCount(8);
                myString_HeartRate_FFT = QString::number(heartRate_FFT, 'f', 0);
                ui->lcdNumber_Heart_FT->display(myString_HeartRate_FFT);
                qDebug() << "Raw Heart Rate FFT:" << heartRate_FFT << "Displayed Heart Rate FFT:" << myString_HeartRate_FFT;

                QString myString_breathRate_CM;
                ui->lcdNumber_CM_Breath->setDigitCount(8);
                myString_breathRate_CM = QString::number(breathRate_CM, 'f', 3);
                ui->lcdNumber_CM_Breath->display(myString_breathRate_CM);
                qDebug() << "Raw Breath Rate CM:" << breathRate_CM << "Displayed Breath Rate CM:" << myString_breathRate_CM;

                QString myString_heartRate_CM;
                ui->lcdNumber_CM_Heart->setDigitCount(8);
                myString_heartRate_CM = QString::number(heartRate_CM, 'f', 3);
                ui->lcdNumber_CM_Heart->display(myString_heartRate_CM);
                qDebug() << "Raw Heart Rate CM:" << heartRate_CM << "Displayed Heart Rate CM:" << myString_heartRate_CM;

                QString myString_heartRate_4Hz_CM;
                ui->lcdNumber_Display4->setDigitCount(8);
                myString_heartRate_4Hz_CM = QString::number(heartRate_4Hz_CM, 'f', 3);
                ui->lcdNumber_Display4->display(myString_heartRate_4Hz_CM);
                qDebug() << "Raw Heart Rate 4Hz CM:" << heartRate_4Hz_CM << "Displayed Heart Rate 4Hz CM:" << myString_heartRate_4Hz_CM;

                QString myString_Breathing_WfmEnergy;
                ui->lcdNumber_BreathEnergy->setDigitCount(8);
                myString_Breathing_WfmEnergy = QString::number(outSumEnergyBreathWfm, 'f', 3);
                ui->lcdNumber_BreathEnergy->display(myString_Breathing_WfmEnergy);
                qDebug() << "Raw Breathing Waveform Energy:" << outSumEnergyBreathWfm << "Displayed Breathing Waveform Energy:" << myString_Breathing_WfmEnergy;

                QString myString_Heart_WfmEnergy;
                ui->lcdNumber_HeartEnergy->setDigitCount(8);
                myString_Heart_WfmEnergy = QString::number(outSumEnergyHeartWfm, 'f', 3);
                ui->lcdNumber_HeartEnergy->display(myString_Heart_WfmEnergy);
                qDebug() << "Raw Heart Waveform Energy:" << outSumEnergyHeartWfm << "Displayed Heart Waveform Energy:" << myString_Heart_WfmEnergy;

                QString myString_RCS;
                ui->lcdNumber_RCS->setDigitCount(8);
                myString_RCS = QString::number(maxRCS_updated, 'f', 0);
                ui->lcdNumber_RCS->display(myString_RCS);
                qDebug() << "Raw RCS:" << maxRCS_updated << "Displayed RCS:" << myString_RCS;

                QString myString_xCorr;
                ui->lcdNumber_Heart_xCorr->setDigitCount(8);
                myString_xCorr = QString::number(heartRate_xCorr, 'f', 0);
                ui->lcdNumber_Heart_xCorr->display(myString_xCorr);
                qDebug() << "Raw Heart Rate xCorr:" << heartRate_xCorr << "Displayed Heart Rate xCorr:" << myString_xCorr;

                QString myString_FFT_4Hz;
                ui->lcdNumber_Heart_FT_4Hz->setDigitCount(8);
                myString_FFT_4Hz = QString::number(heartRate_FFT_4Hz, 'f', 0);
                ui->lcdNumber_Heart_FT_4Hz->display(myString_FFT_4Hz);
                qDebug() << "Raw Heart Rate FFT 4Hz:" << heartRate_FFT_4Hz << "Displayed Heart Rate FFT 4Hz:" << myString_FFT_4Hz;

                QString myString_Reserved_1;
                ui->lcdNumber_Display3->setDigitCount(8);
                myString_Reserved_1 = QString::number(outMotionDetectionFlag, 'f', 3);
                ui->lcdNumber_Display3->display(myString_Reserved_1);
                qDebug() << "Raw Motion Detection Flag:" << outMotionDetectionFlag << "Displayed Motion Detection Flag:" << myString_Reserved_1;
                if (outMotionDetectionFlag == 1)
                {
                    ui->lcdNumber_Display3->setAutoFillBackground(true);
                    ui->lcdNumber_Display3->setPalette(Qt::red);
                }
                else
                {
                    ui->lcdNumber_Display3->setPalette(Qt::white);
                }

                QString myString_heartRate_FFT_4Hz;
                ui->lcdNumber_Heart_FT_4Hz->setDigitCount(8);
                myString_heartRate_FFT_4Hz = QString::number(heartRate_FFT_4Hz, 'f', 3);
                ui->lcdNumber_Heart_FT_4Hz->display(myString_heartRate_FFT_4Hz);
                qDebug() << "Raw Heart Rate FFT 4Hz (second update):" << heartRate_FFT_4Hz << "Displayed Heart Rate FFT 4Hz (second update):" << myString_heartRate_FFT_4Hz;

                QString myString_CM_heart_xCorr;
                ui->lcdNumber_CM_Heart_xCorr->setDigitCount(8);
                myString_CM_heart_xCorr = QString::number(heartRate_xCorr_CM, 'f', 3);
                ui->lcdNumber_CM_Heart_xCorr->display(myString_CM_heart_xCorr);
                qDebug() << "Raw Heart Rate xCorr CM:" << heartRate_xCorr_CM << "Displayed Heart Rate xCorr CM:" << myString_CM_heart_xCorr;

                QString myString_CM_breath_xCorr;
                ui->lcdNumber_CM_Breath_xCorr->setDigitCount(8);
                myString_CM_breath_xCorr = QString::number(BreathingRate_xCorr_CM, 'f', 3);
                ui->lcdNumber_CM_Breath_xCorr->display(myString_CM_breath_xCorr);
                qDebug() << "Raw Breathing Rate xCorr CM:" << BreathingRate_xCorr_CM << "Displayed Breathing Rate xCorr CM:" << myString_CM_breath_xCorr;

                QString myString_breathRate_harmEnergy;
                ui->lcdNumber_breathRate_HarmEnergy->setDigitCount(8);
                myString_breathRate_harmEnergy = QString::number(BreathingRate_HarmEnergy, 'f', 3);
                ui->lcdNumber_breathRate_HarmEnergy->display(myString_breathRate_harmEnergy);
                qDebug() << "Raw Breathing Rate Harm Energy:" << BreathingRate_HarmEnergy << "Displayed Breathing Rate Harm Energy:" << myString_breathRate_harmEnergy;

                QString myString_breathRate_xCorr;
                ui->lcdNumber_Breath_xCorr->setDigitCount(8);
                myString_breathRate_xCorr = QString::number(BreathingRate_xCorr, 'f', 3);
                ui->lcdNumber_Breath_xCorr->display(myString_breathRate_xCorr);
                qDebug() << "Raw Breathing Rate xCorr:" << BreathingRate_xCorr << "Displayed Breathing Rate xCorr:" << myString_breathRate_xCorr;

                // Force GUI update after all LCD updates
                qApp->processEvents();
            }
        }
    }
}
void MainWindow::on_pushButton_stop_clicked()
{
    serialWrite->write("sensorStop\n");
    ui->checkBox_LoadConfig->setChecked(true);
    current_gui_status = gui_stopped;
    emit gui_statusChanged();
}
void MainWindow::gui_statusUpdate()
{
    if (current_gui_status == gui_running)
    {
        ui->pushButton_start->setStyleSheet("background-color: red");
        ui->pushButton_stop->setStyleSheet("background-color: none");
        ui->pushButton_pause->setStyleSheet("background-color: none");
        statusBar()->showMessage(tr("Sensor Running"));
    }

    if (current_gui_status == gui_stopped)
    {
        ui->pushButton_stop->setStyleSheet("background-color: red");
        ui->pushButton_start->setStyleSheet("background-color: none");
        ui->pushButton_pause->setStyleSheet("background-color: none");
        statusBar()->showMessage(tr("Sensor Stopped"));
        qDebug()<<"Sensor is Stopped";
        statusBar()->showMessage(tr("Sensor Stopped"));
    }
    if (current_gui_status == gui_paused)
    {
        ui->pushButton_stop->setStyleSheet("background-color: none");
        ui->pushButton_start->setStyleSheet("background-color: none");
        ui->pushButton_pause->setStyleSheet("background-color: red");
        statusBar()->showMessage(tr("GUI is Paused Stopped"));
    }
}

void MainWindow::on_pushButton_pause_clicked()
{
    localCount = 0;
    for (unsigned int i=0; i<NUM_PTS_DISTANCE_TIME_PLOT; i++)
    {
        xDistTimePlot[i]=0;
        yDistTimePlot[i]=0;
        heartWfmBuffer[i] = 0;
        breathingWfmBuffer[i] = 0;
    }
    current_gui_status = gui_paused;
    emit gui_statusChanged();
}
void MainWindow::resizeEvent(QResizeEvent *event)
{
    QPixmap bkgnd(":/new/prefix51/back2.jpg");
    bkgnd = bkgnd.scaled(this->size(), Qt::IgnoreAspectRatio);

    QPalette palette;
    palette.setBrush(QPalette::Window, bkgnd);
    this->setPalette(palette);

    QMainWindow::resizeEvent(event);  // Appelle l’implémentation parente
}
