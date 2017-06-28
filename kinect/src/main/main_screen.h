#ifndef MAIN_SCREEN_H
#define MAIN_SCREEN_H

#include <QMainWindow>
#include <mutex>
#include <kinect/cloud-tools.h>

namespace Ui {
    class main_screen;
}

class main_screen : public QMainWindow {
    Q_OBJECT

public:
    explicit main_screen(QWidget *parent = 0);
    ~main_screen();

    void set_vars(leo::vars& _var, std::mutex& _mtx);

private slots:
    void on_pushButton_clicked();
    void on_radioButtonVoxel_clicked();
    void on_radioButtonNoFilter_clicked();
    void on_getCloudInstant_clicked();
    void on_getCloudsInstant_clicked();
    void on_pubCloudCheckBox_stateChanged(int is_checked);
    void on_alignCloudsBtn_clicked();
    void on_btnClassify_clicked();
    void on_btnModeling_clicked();
    void on_btnViewCloud_clicked();

    void on_openFolderBtn_clicked();

private:
    Ui::main_screen *ui;
    leo::vars* var;
    std::mutex* mtx;
};

#endif
