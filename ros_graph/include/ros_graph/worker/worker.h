#pragma once

class Worker
{
public:
    virtual ~Worker();

public:
    static Worker* create(const char* command);

public:
    virtual void init(int argc, char* argv[]) = 0;
    virtual void run() = 0;
};
