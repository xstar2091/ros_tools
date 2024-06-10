#include "roshelper.h"

RosHelper *RosHelper::instance()
{
    static RosHelper inst;
    return &inst;
}
