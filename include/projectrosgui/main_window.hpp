#ifndef projectrosgui_MAIN_WINDOW_H
#define projectrosgui_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qcustomplot.hpp"
#include "qnode.hpp"

namespace projectrosgui {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

    // My functions
    void plotData(QCustomPlot* customPlot, QVector<double> y, QVector<double> t);

public Q_SLOTS:
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    void updateTaskPlot();
    void updateIRPlot();
    void updateUltraPlot();
//    void updateStatus();

    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    int ir_count, ul_count;
    int task_type;
};

}  // namespace projectrosgui

#endif // projectrosgui_MAIN_WINDOW_H
