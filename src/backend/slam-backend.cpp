#include <string>
#include "SLAMBackendApplication.h"

using namespace std;

int main(int argc, char *argv[]) {
    SLAMBackendApplication application(argc, argv);

    application.run();

    return 0;
}
