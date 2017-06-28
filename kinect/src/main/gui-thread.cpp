#include "main_screen.h"
#include <QApplication>

#include <mutex>
#include <kinect/cloud-tools.h>

namespace leo {
    void gui_main(int argc, char *argv[], vars& var, std::mutex& mtx) {
        QApplication app(argc, argv);
        main_screen window;                               // maybe create a constructor
        window.setFixedSize(500, 400);
        window.set_vars(std::ref(var), std::ref(mtx));
        window.show();                                      // here is the loop
        app.exec();
    }
}
