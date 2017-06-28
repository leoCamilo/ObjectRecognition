#include <string>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QFileSystemModel>
#include <QTreeView>
#include "main_screen.h"
#include "ui_main_screen.h"

bool show_empty_path_msg (std::string str);

main_screen::main_screen(QWidget *parent):QMainWindow(parent), ui(new Ui::main_screen) {
    ui->setupUi(this);

    ui->cbCloudType->addItem("pcl::PointXYZ");
    ui->cbCloudType->addItem("pcl::PointXYZRGB");
    ui->cbCloudType->addItem("pcl::PointNormal");
    ui->cbCloudType->addItem("pcl::PointXYZRGBNormal");

    for (int i = 0; i < 10; i++)
        ui->cbCloudChannel->addItem("clustering/" + QString::number(i));
}

main_screen::~main_screen() {
    delete ui;
}

void main_screen::set_vars(leo::vars& _var, std::mutex& _mtx) {
    this->var = &_var;
    this->mtx = &_mtx;
}

// ------------- viewer tab events -------------

void main_screen::on_btnViewCloud_clicked() {
    std::string path = ui->txtCloudPath->text().toStdString();
    int type = ui->cbCloudType->currentIndex();

    if (show_empty_path_msg(path))
        return;

    this->mtx->lock();
    this->var->pub_photo = true;
    this->var->str = path;

    switch (type) {
        case 0: this->var->pcd_type = 0; break;
        case 1: this->var->pcd_type = 1; break;
        case 2: this->var->pcd_type = 2; break;
        case 3: this->var->pcd_type = 3; break;
    }

    this->mtx->unlock();
}

// ------------- modeling tab events -------------

void main_screen::on_getCloudInstant_clicked() {
    std::string path = ui->txtOutputPath->text().toStdString();

    if (show_empty_path_msg(path))
        return;

    this->mtx->lock();
    this->var->take_photo = true;
    this->var->str = path;
    this->mtx->unlock();
}

void main_screen::on_getCloudsInstant_clicked() {
    std::string path = ui->txtOutputPath->text().toStdString();

    if (show_empty_path_msg(path))
        return;

    this->mtx->lock();
    this->var->take_photos = true;
    this->var->str = path;
    this->mtx->unlock();
}

// ------------- events -------------

void main_screen::on_pushButton_clicked() {
    if (!system("subl")) {
        QMessageBox msgBox;
        msgBox.setText("Ok");
        msgBox.exec();
    }
}

void main_screen::on_radioButtonNoFilter_clicked() {
    QMessageBox msgBox;
    msgBox.setText("NoFilter");
    msgBox.exec();
}

void main_screen::on_radioButtonVoxel_clicked() {
    QMessageBox msgBox;
    msgBox.setText("Filter");
    msgBox.exec();
}

void main_screen::on_pubCloudCheckBox_stateChanged(int is_checked) {
    std::string path = ui->txtOutputPath->text().toStdString();

    if (show_empty_path_msg(path)) {
        ui->pubCloudCheckBox->setChecked(false);
        return;
    }

    if (is_checked) {
        this->mtx->lock();
        this->var->pub_photo = true;
        this->var->str = path;
        this->mtx->unlock();
    } else {
        this->mtx->lock();
        this->var->pub_photo = false;
        this->mtx->unlock();
    }
}

void main_screen::on_alignCloudsBtn_clicked() {
    std::string path_cloud_1 = ui->pathTextCloud1->text().toStdString();
    std::string path_cloud_2 = ui->pathTextCloud2->text().toStdString();
    std::string path_cloud_out = ui->pathTextCloudOut->text().toStdString();

    if (show_empty_path_msg(path_cloud_1) || show_empty_path_msg(path_cloud_2) || show_empty_path_msg(path_cloud_out))
        return;

    this->mtx->lock();
    this->var->align_clouds = true;
    this->var->str = path_cloud_1 + "|" + path_cloud_2 + "|" + path_cloud_out;
    this->mtx->unlock();
}

void main_screen::on_btnClassify_clicked () {
    std::string ros_topic = ui->topicText->text().toStdString();
    std::string model_path = ui->modelPathText->text().toStdString();

    if (show_empty_path_msg(ros_topic) || show_empty_path_msg(model_path))
        return;

    this->mtx->lock();
    this->var->classify = true;
    this->var->str = ros_topic + "|" + model_path;
    this->mtx->unlock();
}

void main_screen::on_btnModeling_clicked () {
    std::string cloud_model = ui->modelText->text().toStdString();

    if (show_empty_path_msg(cloud_model))
        return;

    this->mtx->lock();
    this->var->get_model = true;
    this->var->str = cloud_model;
    this->mtx->unlock();
}

void main_screen::on_openFolderBtn_clicked () {
    if (!system("nautilus .")) {return;}
}

bool show_empty_path_msg (std::string str) {
    if (str.empty()) {
        QMessageBox msgBox;
        msgBox.setText("file path is empty");
        msgBox.exec();
        return true;
    }

    return false;
}
