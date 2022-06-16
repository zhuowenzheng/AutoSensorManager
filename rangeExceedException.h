//
// Created by Alex Zheng on 15/6/2022.
//

#ifndef AUTOSENSORMANAGER_RANGEEXCEEDEXCEPTION_H
#define AUTOSENSORMANAGER_RANGEEXCEEDEXCEPTION_H
#include <exception>

class rangeExceedException : public std::exception{
public:
    rangeExceedException() noexcept {}
    ~rangeExceedException() override = default;
    const char* what() const noexcept override{
        return "[EXCEPTION] 参数需设置在规定范围内！";
    }
private:
    char* msg;
};


#endif //AUTOSENSORMANAGER_RANGEEXCEEDEXCEPTION_H
