#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/projectrosgui/main_window.hpp"
#include <math.h>

namespace projectrosgui {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
//	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    QObject::connect(&qnode, SIGNAL(recieveIRData()), this, SLOT(updateIRPlot()));
    QObject::connect(&qnode, SIGNAL(recieveUltraData()), this, SLOT(updateUltraPlot()));

    ui.IRPlot_1->addGraph();
    ui.IRPlot_2->addGraph();
    ui.IRPlot_3->addGraph();
    ui.UltraPlot_1->addGraph();
    ui.taskPlot->addGraph();
    ui.taskPlot->addGraph();
    ui.taskPlot->addGraph();
    ui.taskPlot->addGraph();

    this->ir_count = 0;
    this->ul_count = 0;
    this->task_type = 0;

}

MainWindow::~MainWindow() {}

void MainWindow::plotData(QCustomPlot* customPlot, QVector<double> y, QVector<double> t){
    double ymax = 0, ymin = 0;
    for (int i = 0; i < y.size(); i++)
    {
        if (y[i] > ymax)
            ymax = y[i];
        if (y[i] < ymin)
            ymin = y[i];
    }
    customPlot->graph(0)->setData(t, y);
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    customPlot->xAxis->setRange(t.front(), t.back());
    customPlot->yAxis->setRange(ymin * 1.5, ymax * 1.5);
    customPlot->replot();
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::updateTaskPlot(){
    if (this->task_type == 0)
    {
//        for (int i=0; i<50; ++i)
//        {
//          // generate a gaussian distributed random number:
//          double tmp1 = rand()/(double)RAND_MAX;
//          double tmp2 = rand()/(double)RAND_MAX;
//          double r = sqrt(-2*log(tmp1))*cos(2*M_PI*tmp2); // box-muller transform for gaussian distribution
//          // set y1 to value of y0 plus a random gaussian pertubation:
//          x1[i] = (i/50.0-0.5)*30+0.25;
//          y1[i] = sin(x1[i])/x1[i]+r*0.15;
//          x1[i] *= 1000;
//          y1err[i] = 0.15;
//        }
        QVector<double> x(2), y(2);
        x[0] = 0; x[1] = 2;
        y[0] = y[1] = this->qnode.ir1.back();
        ui.taskPlot->graph(0)->setData(x, y);
        x[0] = 2; x[1] = 4;
        y[0] = y[1] = this->qnode.ir2.back();
        ui.taskPlot->graph(1)->setData(x, y);
        x[0] = 4; x[1] = 6;
        y[0] = y[1] = this->qnode.ir3.back();
        ui.taskPlot->graph(2)->setData(x, y);
        x[0] = 0; x[1] = 6;
        y[0] = y[1] = this->qnode.ul1.back();
        ui.taskPlot->graph(3)->setData(x, y);
        ui.taskPlot->xAxis->setRange(0, 6);
        ui.taskPlot->yAxis->setRange(-3, 3);
    }
    ui.taskPlot->replot();
}

void MainWindow::updateIRPlot(){
    this->plotData(ui.IRPlot_1, this->qnode.ir1, this->qnode.ir1_t);
    this->plotData(ui.IRPlot_2, this->qnode.ir2, this->qnode.ir2_t);
    this->plotData(ui.IRPlot_3, this->qnode.ir3, this->qnode.ir3_t);

    updateTaskPlot();

//    qnode.logging_model.insertRows(qnode.logging_model.rowCount(),1);
//    std::stringstream logging_model_msg;
//    this->ir_count += 1;
//    logging_model_msg << "Updated IR Data Frame: " << this->ir_count + 1;//this->qnode.ir1.size() << " " << this->qnode.ir1_t.size();
//    QVariant new_row(QString(logging_model_msg.str().c_str()));
//    qnode.logging_model.setData(qnode.logging_model.index(qnode.logging_model.rowCount()-1),new_row);
//    ui.view_logging->scrollToBottom();
}

void MainWindow::updateUltraPlot(){
    this->plotData(ui.UltraPlot_1, this->qnode.ul1, this->qnode.ul1_t);

    updateTaskPlot();
//    this->plotData(ui.UltraPlot_2, this->qnode.ul2, this->qnode.ul2_t);
//    this->plotData(ui.UltraPlot_3, this->qnode.ul3, this->qnode.ul3_t);

//    qnode.logging_model.insertRows(qnode.logging_model.rowCount(),1);
//    std::stringstream logging_model_msg;
//    this->ul_count += 1;
//    logging_model_msg << "Updated Ultra Data Frame: " << this->ul_count;
//    QVariant new_row(QString(logging_model_msg.str().c_str()));
//    qnode.logging_model.setData(qnode.logging_model.index(qnode.logging_model.rowCount()-1),new_row);
//    ui.view_logging->scrollToBottom();
}

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "projectrosgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "projectrosgui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}
}  // namespace projectrosgui

