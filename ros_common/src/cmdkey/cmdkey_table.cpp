#include "ros_common/cmdkey/cmdkey_table.h"

CmdkeyTable* CmdkeyTable::inst()
{
    static CmdkeyTable key_table;
    return &key_table;
}
