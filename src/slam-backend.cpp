#include <string>
#include "SLAMBackendApplication.h"

using namespace std;

int main(int argc, char *argv[]) {
    SLAMBackendApplication application(argc, argv);

    return application.exec();
}
