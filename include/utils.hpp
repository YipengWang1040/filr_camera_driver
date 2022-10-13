#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <sstream>

void _format_string(std::ostream& os, const char* format) // base function
{
    os<<format;
}

template<typename T, typename... Args>
void _format_string(std::ostream& os, const char* format, const T& t, Args... args){
    while(*format!=0){
        if(*format=='%'){
            os<<t;
            _format_string(os,format+1,args...);
            return;
        }
        os<<*format;
        format+=1;
    }
}

template<typename... Args>
std::string strfmt(const char* format, Args... args) // recursive variadic function
{
    std::stringstream out;
    _format_string(out,format,args...);
    return out.str();
}

#endif // UTILS_HPP
