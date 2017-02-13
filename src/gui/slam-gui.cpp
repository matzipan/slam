//
// Created by matzipan on 06/02/17.
//

#include <src/gui/plotting/PlotController.h>
#include "src/gui/plotting/WindowPlot.h"

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);

    WindowPlot plot;
    plot.show();
    plot.setGeometry(10, 10, 1024, 768);
    plot.plot();

    PlotController controller(&plot);

    controller.start();

    int returnValue = application.exec();

    controller.stop();
    controller.wait();

    return returnValue;
}